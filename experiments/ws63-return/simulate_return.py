"""Local RSSI return bench. HTTP substitutes for the untested SLE transport."""
import argparse
import http.client
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import socket
import sqlite3
import struct
import threading
import time

from collect_base import parse_report

WIRE = struct.Struct("<BIIQb")
PAGE = """<!doctype html><meta charset="utf-8"><title>RSSI 回传模拟</title>
<style>body{font:18px system-ui;margin:40px;background:#101b2b;color:#e4efff}
strong{color:#ffc66d}canvas{background:#17283c;max-width:100%}pre{font-size:15px}</style>
<h1>RSSI 回传模拟测试</h1><strong>当前展示人工生成的数据，传输使用本机 HTTP。星闪无线尚待实板验证。</strong>
<pre id="status">正在连接</pre><canvas id="plot" width="900" height="280"></canvas>
<script>
async function refresh(){
try {
 const s=await (await fetch('/state',{cache:'no-store'})).json();
 document.getElementById('status').textContent=JSON.stringify(s.summary,null,2);
 const c=document.getElementById('plot'),g=c.getContext('2d');g.clearRect(0,0,c.width,c.height);
 g.fillStyle='#b2cce7';g.fillText('RSSI (dBm), 最近120条模拟样本',20,20);
 g.strokeStyle='#55dac1';g.beginPath();
 s.recent.forEach((r,i)=>{const x=20+i*7,y=35+(-40-r[1])*3;i?g.lineTo(x,y):g.moveTo(x,y)});
 g.stroke();
} catch(e){document.getElementById('status').textContent='模拟服务器未连接';}
}
setInterval(refresh,500);refresh();
</script>"""


class Store:
    def __init__(self, path):
        self.lock = threading.Lock()
        self.db = sqlite3.connect(path, check_same_thread=False)
        self.db.execute("PRAGMA synchronous=FULL")
        self.db.execute("""CREATE TABLE IF NOT EXISTS samples (
            session INTEGER, sequence INTEGER, device_us INTEGER, rssi INTEGER,
            received REAL, PRIMARY KEY(session, sequence))""")
        self.db.commit()
        self.duplicates = self.invalid = 0
        self.drop_ack_used = False

    def put(self, data):
        if len(data) != WIRE.size:
            raise ValueError("invalid length")
        version, session, sequence, device_us, rssi = WIRE.unpack(data)
        if version != 1:
            raise ValueError("invalid version")
        # Exercise the same report validation as the real base serial collector.
        parse_report(f"+BCRX:{session},{sequence},{device_us},{rssi}".encode())
        with self.lock:
            old = self.db.execute("SELECT device_us,rssi FROM samples WHERE session=? AND sequence=?",
                                  (session, sequence)).fetchone()
            if old is not None and old != (device_us, rssi):
                raise ValueError("same key with different sample")
            if old is None:
                self.db.execute("INSERT INTO samples VALUES (?,?,?,?,?)",
                                (session, sequence, device_us, rssi, time.time()))
                self.db.commit()
            else:
                self.duplicates += 1
        return {"session": session, "sequence": sequence, "duplicate": old is not None}

    def snapshot(self):
        with self.lock:
            count = self.db.execute("SELECT count(*) FROM samples").fetchone()[0]
            recent = self.db.execute("SELECT sequence,rssi FROM samples ORDER BY received DESC LIMIT 120").fetchall()
            return {"summary": {"mode": "SIMULATION / HTTP loopback", "stored": count,
                                "duplicates": self.duplicates, "invalid": self.invalid},
                    "recent": list(reversed(recent))}


def make_server(port, store, *, fault_injection=False):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def setup(self):
            super().setup()
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        def log_message(self, *args):
            pass

        def respond(self, code, body, kind="application/json"):
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/":
                self.respond(200, PAGE.encode(), "text/html; charset=utf-8")
            elif self.path == "/state":
                self.respond(200, json.dumps(store.snapshot()).encode())
            else:
                self.respond(404, b"{}")

        def do_POST(self):
            if self.path != "/sample":
                self.close_connection = True
                self.respond(404, b"{}")
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if length != WIRE.size:
                    self.close_connection = True
                    raise ValueError("invalid length")
                self.connection.settimeout(2)
                result = store.put(self.rfile.read(length))
            except (ValueError, socket.timeout) as error:
                with store.lock:
                    store.invalid += 1
                self.respond(400, json.dumps({"error": str(error)}).encode())
                return
            if fault_injection and self.headers.get("X-Test-Drop-Ack") == "1" and not store.drop_ack_used:
                store.drop_ack_used = True
                self.close_connection = True
                self.connection.shutdown(socket.SHUT_RDWR)
                return
            self.respond(200, json.dumps(result).encode())

    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    server.daemon_threads = True
    return server


def send(connection, data, headers=None):
    started = time.perf_counter()
    connection.request("POST", "/sample", data, headers or {})
    response = connection.getresponse()
    result = json.loads(response.read())
    return response.status, result, (time.perf_counter() - started) * 1000


def bench(report_path, db_path):
    if db_path.exists():
        raise ValueError("use a new database path")
    store = Store(db_path)
    server = make_server(0, store, fault_injection=True)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    port = server.server_port
    connection = http.client.HTTPConnection("127.0.0.1", port, timeout=3)
    connection.connect()
    connection.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    checks = []
    latencies = []
    try:
        for i in range(1, 51):
            started = time.perf_counter()
            status, _, delay = send(connection, WIRE.pack(1, 1001, i, i*100000, -85+i%15))
            assert status == 200
            latencies.append(delay)
            time.sleep(max(0, .1 - (time.perf_counter()-started)))
        checks.append("10 Hz / 50 samples persisted")
        for i in range(1, 51):
            assert send(connection, WIRE.pack(1, 1001, i, i*100000, -85+i%15))[1]["duplicate"]
        checks.append("50 retransmissions deduplicated")
        begin = time.perf_counter()
        for i in range(1, 501):
            assert send(connection, WIRE.pack(1, 1002, i, i*1000, -90+i%25))[0] == 200
        duration = time.perf_counter()-begin
        checks.append("500-sample burst persisted")
        assert send(connection, WIRE.pack(2, 1003, 1, 1, -80))[0] == 400
        assert send(connection, WIRE.pack(1, 1003, 1, 1, 127))[0] == 400
        assert send(connection, WIRE.pack(1, 1003, 0, 1, -80))[0] == 400
        assert send(connection, WIRE.pack(1, 1001, 1, 100000, -40))[0] == 400
        checks.append("bad version/RSSI/sequence and conflicting duplicate rejected")
        packet = WIRE.pack(1, 1004, 1, 500, -77)
        try:
            send(connection, packet, {"X-Test-Drop-Ack": "1"})
            raise AssertionError("fault injection did not disconnect")
        except http.client.RemoteDisconnected:
            pass
        connection.close()
        connection = http.client.HTTPConnection("127.0.0.1", port, timeout=3)
        assert send(connection, packet)[1]["duplicate"]
        checks.append("commit followed by lost ACK and reconnect deduplicated")
        for sequence in (3, 1, 2):
            assert send(connection, WIRE.pack(1, 1005, sequence, sequence*1000, -71))[0] == 200
        checks.append("simulator preserves reordered samples")
    finally:
        connection.close()
        server.shutdown()
        server.server_close()
        store.db.close()
    restarted = Store(db_path)
    count = restarted.snapshot()["summary"]["stored"]
    assert count == 554, count
    assert restarted.put(packet)["duplicate"]
    restarted.db.close()
    checks.append("database reopen preserved 554 unique records")
    latencies.sort()
    report = {"mode": "SIMULATION, no SLE radio tested", "passed": checks,
              "unique_records": count, "burst_samples_per_second": round(500/duration, 1),
              "10hz_ack_ms_p50": round(latencies[len(latencies)//2], 3),
              "10hz_ack_ms_p95": round(latencies[int(len(latencies)*.95)-1], 3),
              "latency_scope": "loopback HTTP request through SQLite commit to ACK on this PC"}
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8893)
    parser.add_argument("--db", type=Path, required=True)
    parser.add_argument("--test-report", type=Path)
    parser.add_argument("--demo", action="store_true", help="generate synthetic samples at 10 Hz")
    args = parser.parse_args()
    if args.test_report:
        bench(args.test_report, args.db)
    else:
        store = Store(args.db)
        server = make_server(args.port, store)
        stop = threading.Event()
        def generate():
            connection = http.client.HTTPConnection("127.0.0.1", server.server_port, timeout=2)
            session = int(time.time()) % (2**32 - 1) + 1
            sequence = 0
            started = time.monotonic()
            try:
                while not stop.is_set():
                    sequence += 1
                    elapsed = time.monotonic() - started
                    data = WIRE.pack(1, session, sequence, int(elapsed*1e6),
                                     round(-77 + 14*math.sin(elapsed/4)))
                    try:
                        send(connection, data)
                    except (OSError, http.client.HTTPException):
                        connection.close()
                    stop.wait(.1)
            finally:
                connection.close()
        if args.demo:
            threading.Thread(target=generate, daemon=True).start()
        print(f"SIMULATION http://127.0.0.1:{server.server_port}/", flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            stop.set()
            server.server_close()
            store.db.close()


if __name__ == "__main__":
    main()
