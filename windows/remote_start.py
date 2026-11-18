import fcntl,json,os,re,subprocess,sys,time,urllib.request
from pathlib import Path

profile=sys.argv[1]
home=Path.home()
lock=(home/'.cache/r550_start.lock')
lock.parent.mkdir(exist_ok=True)
handle=lock.open('w')
try:fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
except BlockingIOError:sys.exit('Another startup is running. Wait and retry.')
opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
def state():
    try:
        with opener.open('http://127.0.0.1:8890/api/state',timeout=2) as r:s=json.load(r)
    except (OSError,ValueError):return None
    if s.get('health',{}).get('mode')!='robot':
        sys.exit('Port 8890 is not the live robot panel. No processes were changed.')
    return s
def check(s):
    print('Existing live panel reused. No restart, no motion.' if s else '',flush=True)
    if s:print('Phase:',s.get('phase'),'Ready:',s.get('health',{}).get('ready'),flush=True)
s=state()
if s:check(s);sys.exit(0)
args=subprocess.check_output(['ps','-eo','args'],text=True)
if re.search(r'wheeltec_robot_node|controller_server|[sa]sync_slam_toolbox_node|auto_recharge_ros2|demo/server.py',args):
    sys.exit('Robot/SLAM/navigation processes already exist but panel is unavailable. Refusing duplicate startup. Inspect logs; do not kill or restart while driving.')
if profile=='425':
    folder=home/'r550_field_kit'
    if not (folder/'kit').is_file():sys.exit('425 kit not found. Check robot/profile.')
    command=['bash','-lc','cd "$HOME/r550_field_kit" && ./kit nav']
else:
    script=home/'lab_cruise_ws/demo/start_demo.sh'
    if not script.is_file():sys.exit('1003 deployment not found. Check robot/profile.')
    command=['bash',str(script)]
with (home/'.cache/r550_start.log').open('wb') as log:
    process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    for _ in range(120):
        s=state()
        if s:check(s);sys.exit(0)
        if process.poll() not in (None,0):
            sys.exit('Startup failed. Inspect ~/.cache/r550_start.log. No automatic restart performed.')
        time.sleep(1)
sys.exit('Panel not ready after 120 seconds. Inspect ~/.cache/r550_start.log; do not repeatedly start.')
