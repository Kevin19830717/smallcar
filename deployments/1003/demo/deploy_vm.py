"""Install the demo package on the fixed-address VM. Does not contact the car."""
import base64
from pathlib import Path
import subprocess


def main():
    here = Path(__file__).resolve().parent
    payload = {}
    for path in here.rglob('*'):
        if path.is_file() and not any(part in ('data','__pycache__','验证记录') for part in path.relative_to(here).parts):
            payload[path.relative_to(here).as_posix()] = base64.b64encode(path.read_bytes()).decode()
    code = """from pathlib import Path
import base64, subprocess
root=Path.home()/'lab_cruise_view/demo'
payload=PAYLOAD
for name,data in payload.items():
    p=root/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(base64.b64decode(data))
    if p.suffix=='.sh':
        p.chmod(0o755);subprocess.run(['bash','-n',str(p)],check=True)
desktop=Path.home()/'Desktop/实验室巡航演示.desktop'
desktop.write_text(chr(10).join(['[Desktop Entry]','Type=Application','Name=实验室巡航演示',
    'Exec=gnome-terminal -- bash /home/wheeltec/lab_cruise_view/demo/start_robot_demo.sh',
    'Icon=utilities-terminal','Terminal=false','']))
desktop.chmod(0o755)
subprocess.run(['gio','set',str(desktop),'metadata::trusted','true'],capture_output=True)
print('Demo package and desktop launcher installed; robot not contacted.')
""".replace('PAYLOAD',repr(payload))
    result = subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=3','r550-vm','python3','-'],
                            input=code,capture_output=True,encoding='utf-8',timeout=25)
    print(result.stdout);print(result.stderr);result.check_returncode()


if __name__ == '__main__':
    main()
