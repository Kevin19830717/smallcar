"""ROS bag plus a timestamped operator/state timeline for navigation diagnosis."""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tarfile
import threading
import time
import uuid
from retention import LIMIT,RESERVE,SESSION_LIMIT,size,prune,remove

TOPICS = r'^/(cmd_vel|nav_cmd_vel|odom|odom_combined|joint_states|imu/data_raw|imu/data_filtered|tf|tf_static|amcl_pose|scan|scan_raw|map|plan|plan_smoothed|local_plan|rosout|parameter_events|robot_.*|PowerVoltage|(global|local)_costmap/(costmap|costmap_updates|published_footprint)|(navigate_to_pose|follow_path|spin|backup)/_action/(status|feedback))$'
EXCLUDED = 'Whitelist recording; debug visualization is excluded'


class FlightRecorder:
    def __init__(self, source, directory, backend):
        self.source=Path(source)
        self.directory=Path(directory)
        self.directory.mkdir(parents=True,exist_ok=True)
        self.backend=backend
        self.lock=threading.RLock()
        self.process=None
        self.folder=None
        archives=list(self.directory.glob('*.tar.gz'))
        self.archive=max(archives,key=lambda p:p.stat().st_mtime) if archives else None
        marker=self.directory/'.latest_session'
        if marker.is_file():
            candidate=self.directory/(Path(marker.read_text().strip()).name+'.tar.gz')
            if candidate.is_file():self.archive=candidate
        self.finalizing=False
        self.log=None
        self.error=None
        self.last_sample=0.
        self.quiet_since=None
        self.log_offsets={}
        self.rotate_pending=False
        prune(self.directory)
        if self.archive and not self.archive.exists():self.archive=None

    def status(self):
        # Do not block the UI/stop command while rosbag flushes or gzip runs.
        process=self.process
        recording=process is not None and process.poll() is None
        return {'quota_bytes':LIMIT,'recording':recording,'session':self.folder.name if self.folder else None,
                    'error':self.error,'download_ready':self.archive is not None,
                    'finalizing':self.finalizing,
                    'message':self.error or ('正在保存诊断包…' if self.finalizing else '诊断记录中；总预算2GB，旧记录自动替换' if recording else
                        '诊断包已保存，可下载' if self.archive else '开始任务时自动记录')}

    def event(self, kind, **fields):
        with self.lock:
            if self.folder is None:
                return
            row={'wall_utc':datetime.now(timezone.utc).isoformat(),
                 'monotonic_ns':time.monotonic_ns(),'event':kind,**fields}
            with (self.folder/'timeline.jsonl').open('a',encoding='utf-8') as f:
                f.write(json.dumps(row,ensure_ascii=False,allow_nan=False,default=str)+'\n')

    def begin(self):
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                self.quiet_since=None
                return
            prune(self.directory,LIMIT-RESERVE)
            if self.archive and not self.archive.exists():self.archive=None
            if shutil.disk_usage(self.directory).free<512_000_000:
                raise ValueError('小车磁盘可用空间不足512MB，请检查其它文件占用')
            if self.log:
                self.log.close()
                self.log=None
            name=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:8]
            self.folder=self.directory/name
            self.folder.mkdir()
            (self.directory/'.latest_session').write_text(name,encoding='ascii')
            self.archive=None
            self.error=None
            self.log_offsets={p:max(0,p.stat().st_size-65536)
                              for p in (Path.home()/'.ros/log').glob('lab_demo*.log') if p.is_file()}
            config=self.folder/'configuration'
            config.mkdir()
            files=list(self.source.glob('*.py'))+list(self.source.glob('*.yaml'))
            files+=list((self.source/'assets').glob('*'))
            files+=list((self.source/'web').glob('*'))
            files+=list((self.source/'data/ros').glob('*.json'))
            checksums={}
            for p in files:
                if not p.is_file():
                    continue
                rel=p.relative_to(self.source)
                target=config/rel
                target.parent.mkdir(parents=True,exist_ok=True)
                shutil.copy2(p,target)
                checksums[str(rel)]=hashlib.sha256(p.read_bytes()).hexdigest()
            manifest={'session':name,'created_utc':datetime.now(timezone.utc).isoformat(),
                      'monotonic_ns':time.monotonic_ns(),'configuration_sha256':checksums,
                      'excluded_topic_regex':EXCLUDED,'recorded_topic_regex':TOPICS,
                      'notes':['Raw images and point clouds are excluded to limit recording load.',
                               'Compressed camera topics are recorded only if already published.',
                               'ROS data cannot recover unpublished raw motor-controller serial bytes.',
                               'Action goals/results may use services; operator commands, bridge state and ROS logs supplement the bag.'],
                      'topics_at_start':self.backend.get_topic_names_and_types()}
            (self.folder/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
            self.log=(self.folder/'recorder.log').open('wb')
            cmd=['ros2','bag','record','--include-hidden-topics','-e',TOPICS,
                 '--max-cache-size','33554432','-b','64000000','-o',str(self.folder/'rosbag')]
            self.process=subprocess.Popen(cmd,stdout=self.log,stderr=subprocess.STDOUT,
                                          start_new_session=True)
            deadline=time.monotonic()+8
            while time.monotonic()<deadline:
                if self.process.poll() is not None:
                    self.error='ROS录包进程未能启动，请查看recorder.log'
                    raise ValueError(self.error)
                text=(self.folder/'recorder.log').read_text(errors='replace')
                if 'Recording...' in text:
                    self.event('recording_started',command=cmd)
                    self.quiet_since=None
                    return
                time.sleep(.1)
            self._stop_locked()
            self.error='录包启动确认超时，本次任务未启动'
            raise ValueError(self.error)

    def observe(self, state):
        with self.lock:
            if self.process is None:
                if self.rotate_pending:
                    self.rotate_pending=False
                    if state["phase"] not in ("idle","completed","failed","docked"):
                        self.begin()
                return
            if self.process.poll() is not None:
                self.error=self.error or '录包进程已退出，请检查诊断记录'
                return
            now=time.monotonic()
            if now-self.last_sample>=.5:
                compact=dict(state)
                compact['health']={k:v for k,v in state.get('health',{}).items() if k!='laser'}
                compact['events']=state.get('events',[])[:20]
                self.event('state',state=compact)
                self.last_sample=now
            if size(self.folder)>=SESSION_LIMIT:
                self.finish()
                self.rotate_pending=state['phase'] not in ('idle','completed','failed','docked')
                return
            if shutil.disk_usage(self.directory).free<256_000_000:
                self.error='可用空间不足256MB，诊断记录已停止'
                self.finish()
                return
            quiet=(state['phase'] in ('idle','completed','failed','docked')
                   and state.get('health',{}).get('stationary',False))
            self.quiet_since=(self.quiet_since or now) if quiet else None
            if self.quiet_since is not None and now-self.quiet_since>=10:
                self.finish()

    def _stop_locked(self):
        if self.process is not None and self.process.poll() is None:
            os.killpg(self.process.pid,signal.SIGINT)
            try:
                self.process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid,signal.SIGTERM)
                self.process.wait(timeout=3)
                self.error='录包未正常结束，需检查bag元数据'
        self.process=None
        if self.log:
            self.log.close()
            self.log=None

    def finish(self):
        with self.lock:
            self.finalizing=True
            try:
                return self._finish_locked()
            finally:
                self.finalizing=False

    def _finish_locked(self):
            self._stop_locked()
            if self.folder is None:
                raise ValueError('尚无诊断记录')
            self.event('recording_finished')
            if not (self.folder/'rosbag/metadata.yaml').exists():
                result=subprocess.run(['ros2','bag','reindex',str(self.folder/'rosbag')],
                                      stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=45)
                (self.folder/'reindex.log').write_bytes(result.stdout)
                if result.returncode or not (self.folder/'rosbag/metadata.yaml').exists():
                    self.error='录包元数据不完整，原始文件已保留'
                    raise ValueError(self.error)
                self.event('metadata_reindexed',note='Inspect recorder.log for shutdown interruption; last buffered messages may be missing.')
            logs=self.folder/'process_logs'
            logs.mkdir(exist_ok=True)
            for p,offset in self.log_offsets.items():
                try:
                    with p.open('rb') as f:
                        f.seek(offset)
                        with (logs/p.name).open('wb') as out:
                            shutil.copyfileobj(f,out)
                except OSError as error:
                    self.event('log_copy_error',file=str(p),error=str(error))
            runtime=self.folder/'runtime_parameters'
            runtime.mkdir(exist_ok=True)
            for name in ('controller_server','behavior_server','local_costmap/local_costmap',
                         'global_costmap/global_costmap','velocity_smoother'):
                try:
                    values=self.runtime_parameters(name)
                    (runtime/(name.replace('/','_')+'.json')).write_text(
                        json.dumps(values,ensure_ascii=False,indent=2),encoding='utf-8')
                except Exception as error:
                    self.event('parameter_dump_error',node=name,error=str(error))
            target=self.directory/(self.folder.name+'.tar.gz')
            tmp=target.with_suffix('.tmp')
            with tarfile.open(tmp,'w:gz',compresslevel=1) as t:
                t.add(self.folder,arcname=self.folder.name)
            tmp.replace(target)
            self.archive=target
            # The verified archive is the retained copy; discard duplicate raw data.
            with tarfile.open(target,'r:gz') as check:
                if not any(n.endswith('/rosbag/metadata.yaml') for n in check.getnames()):
                    raise ValueError('诊断包缺少回放元数据，保留原始文件')
            remove(self.directory,self.folder)
            self.folder=None
            prune(self.directory,LIMIT,protected=(target,))
            return target

    def runtime_parameters(self, name):
        from rcl_interfaces.srv import ListParameters, GetParameters
        from rosidl_runtime_py.convert import message_to_ordereddict
        clients=[]
        def request(service, suffix, value):
            client=self.backend.create_client(service,'/'+name+'/'+suffix)
            clients.append(client)
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError(name+' parameter service unavailable')
            future=client.call_async(value)
            deadline=time.monotonic()+5
            while not future.done() and time.monotonic()<deadline:
                time.sleep(.05)
            if not future.done():
                future.cancel()
                raise RuntimeError(name+' parameter service timed out')
            return future.result()
        try:
            names=request(ListParameters,'list_parameters',ListParameters.Request()).result.names
            values=request(GetParameters,'get_parameters',GetParameters.Request(names=list(names))).values
            return dict(zip(names,(message_to_ordereddict(v) for v in values)))
        finally:
            for client in clients:
                self.backend.destroy_client(client)

    def close(self):
        with self.lock:
            self._stop_locked()
