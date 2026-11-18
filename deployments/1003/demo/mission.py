"""Waypoint/dwell/docking sequencing. No localization or motion-control algorithms."""
from collections import deque
import json
import math
import os
from pathlib import Path
import threading
import time
import uuid


TERMINAL = {'succeeded', 'failed', 'canceled'}
QUIET = {'idle', 'completed', 'failed', 'docked'}
MAX_DOCK_ATTEMPTS = 5
MAX_DOCK_RETRIES = MAX_DOCK_ATTEMPTS - 1


class Mission:
    def __init__(self, grid, backend, directory, clock=time.monotonic):
        self.grid, self.backend, self.clock = grid, backend, clock
        self.directory = Path(directory);self.directory.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.points = [];self.events = deque(maxlen=100)
        self.phase = 'idle';self.message = '请添加导航点，核对定位后开始'
        self.run = [];self.index = 0;self.operation = None;self.pending = None
        self.until = 0;self.remaining = 0;self.auto_return = False;self.low_since = None
        self.dock_failure_reconcile=False;self.dock_retry_count=0;self.dock_retry_pending=False;self.dock_retry_started=None;self.dock_direct_retry_streak=0
        self._load()
        from collection import Collection
        self.collection = Collection(self.directory,grid)

    def _load(self):
        p = self.directory / 'waypoints.json'
        if p.exists():
            try:
                data = json.loads(p.read_text(encoding='utf-8'))
                if data['map_id'] != self.grid.map_id:
                    self.message = '地图版本已改变，旧点位未加载，请重新核对点位'
                    return
                self.points = self.validate_points(data['points'])
            except (ValueError, KeyError, TypeError):
                self.message = '点位文件无效，请重新设置'

    def event(self, text):
        item = {'time': time.strftime('%H:%M:%S'), 'text': text}
        self.events.appendleft(item)
        with (self.directory / 'events.jsonl').open('a', encoding='utf-8') as f:
            f.write(json.dumps(item, ensure_ascii=False)+'\n')

    def validate_points(self, points):
        if not isinstance(points, list) or len(points) > 100:
            raise ValueError('最多设置 100 个导航点')
        result = [];ids = set()
        for i, point in enumerate(points):
            if not isinstance(point, dict):
                raise ValueError('点位格式错误')
            values = [float(point[k]) for k in ('x','y','yaw','wait')]
            if not all(math.isfinite(v) for v in values) or not 0 <= values[3] <= 3600:
                raise ValueError('坐标与朝向必须有效，停留时间须在 0～3600 秒内')
            self.grid.validate(values[0], values[1])
            identity = str(point.get('id') or uuid.uuid4().hex[:12])
            if identity in ids or len(identity) > 64:
                raise ValueError('点位编号重复或无效')
            ids.add(identity)
            result.append({'id': identity, 'name': str(point.get('name') or f'点位 {i+1}')[:40],
                           'x': values[0], 'y': values[1],
                           'yaw': math.atan2(math.sin(values[2]), math.cos(values[2])),
                           'wait': values[3], 'enabled': bool(point.get('enabled', True))})
            if 'survey_path' in point:
                from survey_path import checked_path
                result[-1]['survey_path']=checked_path(self.grid,point['survey_path'],values[:2])
            if 'survey_pass_radius' in point:
                radius=float(point['survey_pass_radius'])
                if not math.isfinite(radius) or not .1<=radius<=.5:raise ValueError('采集引导范围无效')
                result[-1].update(survey_pass_radius=radius,survey_optional=bool(point.get('survey_optional',False)))
        return result

    def save_points(self, points):
        with self.lock:
            if self.phase not in QUIET:
                raise ValueError('请先停止当前任务，再修改点位')
            self.points = self.validate_points(points)
            data = json.dumps({'schema': 1, 'map_id': self.grid.map_id, 'points': self.points}, ensure_ascii=False, indent=2)
            temporary = self.directory / 'waypoints.tmp'
            temporary.write_text(data, encoding='utf-8');os.replace(temporary, self.directory/'waypoints.json')
            return self.points

    def require_ready(self):
        health = self.backend.snapshot()
        if not health['ready']:
            raise ValueError(health.get('readiness') or '导航或定位尚未就绪')
        if not health.get('localization_confirmed'):
            raise ValueError('请先核对小车位置、朝向和红色雷达，并确认定位正确')

    def start(self, identity=None, auto_return=False, start_from=0):
        with self.lock:
            if self.phase not in QUIET:
                raise ValueError('已有任务运行，请先停止或暂停')
            self.require_ready()
            self.dock_retry_count=0;self.dock_retry_pending=False;self.dock_retry_started=None;self.dock_direct_retry_streak=0
            chosen = [p for p in self.points if (p['id'] == identity if identity else p['enabled'])]
            if start_from:
                if identity or self.collection.snapshot().get('profile')!='moving_loop_demo' or not isinstance(start_from,int) or not 0<=start_from<len(chosen):raise ValueError('无效续跑引导点')
                chosen=chosen[start_from:]
            if not chosen:
                raise ValueError('请至少选择一个导航点')
            if any(p['id'].startswith('north-loop-') for p in chosen) and self.collection.snapshot().get('profile')!='moving_loop_demo':
                raise ValueError('快速回环引导点请使用“广播 RSSI 快速回环”模式，避免按普通精确停靠逐点执行')
            if any(p.get('survey_path') for p in chosen) and self.collection.snapshot().get('profile')!='moving_loop_demo':raise ValueError('连续采集路线仅用于广播采集模式')
            self.collection.prepare(chosen)
            if self.collection.snapshot()['mode']=='broadcast':
                self.backend.survey_speed_limit=.12 if self.collection.snapshot().get('profile')=='moving_loop_demo' else .14
            if hasattr(self.backend,'set_survey_planning'):
                self.backend.set_survey_planning(self.collection.snapshot().get('profile')=='moving_loop_demo')
            self.run = [dict(p) for p in chosen];self.index = 0
            self.auto_return = bool(auto_return) if identity is None else False
            self.pending = None;self.low_since = None
            self.event('开始单点前往' if identity else f'开始巡航，共 {len(chosen)} 个点')
            health=self.backend.snapshot()
            if health.get('charging') or health.get('dock_position_confirmed'):
                self._depart()
            else:
                self._navigate_current()

    def _depart(self):
        x,y,heading = self.backend.dock_pose
        distance = getattr(self.backend,'departure_distance',.30)
        point = {'x':x+distance*math.cos(heading),
                 'y':y+distance*math.sin(heading),'yaw':heading}
        self.operation = self.backend.navigate(point,'departure')
        self.phase = 'departing';self.message = '先直行驶离充电桩，再前往导航点'

    def _navigate_current(self):
        point = self.run[self.index]
        moving=self.collection.snapshot().get('profile')=='moving_loop_demo'
        self.operation = self.backend.navigate(point, 'survey' if moving else 'waypoint')
        self.phase = 'navigating';self.message = f'正在前往 {point["name"]}'

    def _begin_arrival(self, returning):
        now = self.clock()
        if returning:
            self.backend.begin_docking();self.until = now+getattr(self.backend,'docking_timeout',90.)
            self.phase = 'docking';self.message = '桩前到达，等待红外对接及充电反馈'
        else:
            p = self.run[self.index]
            if self.collection.snapshot().get('profile')=='moving_loop_demo':
                self._advance();return
            if self.collection.snapshot()['mode'] in ('ws63','broadcast'):
                self.collection.begin(p,self.backend.snapshot())
                self.phase='collecting';self.message=f'{p["name"]} 已停稳，等待 WS63 本轮采集完成'
                self.event(self.message)
                return
            self.until = now+p['wait']
            self.phase = 'dwelling';self.message = f'{p["name"]} 已停稳，停留 {p["wait"]:g} 秒'
            self.event(self.message)

    def _can_handoff_dock(self, health):
        """Use IR only in the observed stopped capture area, never mid-route."""
        if health.get('mode')=='robot' and not health.get('dock_sensors_fresh',False):return False
        pose = health.get('pose');approach = self.backend.dock_approach
        if not (health.get('ready') and health.get('localization_confirmed')
                and health.get('stationary') and health.get('ir') and pose):
            return False
        angle = math.atan2(math.sin(pose[2]-self.backend.dock_pose[2]),
                           math.cos(pose[2]-self.backend.dock_pose[2]))
        home=self.backend.dock_pose;dx=pose[0]-home[0];dy=pose[1]-home[1]
        forward=dx*math.cos(home[2])+dy*math.sin(home[2])
        near_home=math.hypot(dx,dy)<=.15 and forward>=-.08
        return ((math.hypot(pose[0]-approach['x'],pose[1]-approach['y']) <= .10 or near_home)
                and abs(angle) <= math.radians(30))

    def configure_collection(self, mode, device='auto'):
        with self.lock:
            if self.phase not in QUIET: raise ValueError('请先停止当前任务，再切换采集模式')
            if mode in ('ws63','broadcast') and self.backend.snapshot().get('mode')=='offline':
                raise ValueError('离线演练不能启动真实 WS63 采集，请使用实车入口')
            self.collection.configure(mode,device)
            self.event('切换到 WS63 采集停留' if mode=='ws63' else '切换到原有定时停留')

    def _advance(self):
        self.index += 1
        if self.index < len(self.run): self._navigate_current()
        elif self.auto_return: self._return()
        else:
            if self.collection.broadcast:self.collection.broadcast.finish_drive()
            self.phase='completed';self.message='所有导航点与停留任务已完成';self.event(self.message)

    def return_to_dock(self, automatic=False):
        with self.lock:
            if not automatic:self.dock_retry_count=0;self.dock_retry_pending=False;self.dock_retry_started=None;self.dock_direct_retry_streak=0
            if (self.phase in ('collecting','paused_collection','canceling_collection')
                    or self.collection.snapshot().get('pending') or self.collection.snapshot()['stage']=='canceling'):
                self.collection.cancel();self.backend.stop()
                self.phase='canceling_collection';self.message='等待 WS63 取消当前采集，再主动回充'
                return
            health=self.backend.snapshot()
            if health.get('charging_stable' ,health.get('charging')) and (not self.operation or self.backend.operation(self.operation)['status'] in TERMINAL):
                self.backend.stop();self.phase = 'docked';self.message = '已在充电桩，反馈正在充电'
                return
            self.require_ready()
            if self.operation and self.backend.operation(self.operation)['status'] not in TERMINAL:
                self.backend.cancel(self.operation);self.pending = 'return'
                self.phase = 'canceling';self.message = '正在停止当前任务，随后回充'
            else:
                self._return()

    def _return(self):
        if hasattr(self.backend,'set_survey_planning'):self.backend.set_survey_planning(False)
        if self.collection.broadcast:self.collection.broadcast.finish_drive()
        self.backend.survey_speed_limit=None
        health=self.backend.snapshot()
        if health.get('charging_stable',health.get('charging')):
            self.backend.stop();self.phase = 'docked';self.message = '已在充电桩，反馈正在充电'
            self.operation = None;self.pending = None
            return
        if self._start_direct_docking(health,'回充到位'):return
        self.operation = self.backend.navigate(self.backend.dock_approach, 'return')
        self.phase = 'returning';self.message = '正在导航至充电桩前方'
        self.pending = None;self.event('开始主动回充')

    def _start_direct_docking(self,health,source):
        # This is the same stopped IR handoff for manual and automatic returns.
        if not self._can_handoff_dock(health):return False
        self.operation=None;self.pending=None
        self._begin_arrival(True)
        self.message=f'{source}：有效红外接管，直接对接（第{self.dock_retry_count+1}/{MAX_DOCK_ATTEMPTS}次）'
        self.event(self.message)
        return True

    def pause(self):
        with self.lock:
            if self.phase == 'collecting':
                self.phase='paused_collection';self.message='巡航已暂停，当前站采集继续，完成后也不离站'
            elif self.phase == 'dwelling':
                self.remaining = max(0, self.until-self.clock());self.phase = 'paused_dwell'
                self.message = '停留计时已暂停';self.event(self.message)
            elif self.phase in ('navigating','returning','departing'):
                self.pending = 'pause_return' if self.phase == 'returning' else 'pause_departure' if self.phase == 'departing' else 'pause_navigation'
                self.backend.cancel(self.operation);self.phase = 'canceling';self.message = '正在停车'
            elif self.phase == 'docking':
                self.backend.stop();self.phase = 'paused_dock';self.message = '红外对接已暂停'
            elif self.phase in ('settling','settling_return','settling_departure'):
                self.phase = 'paused_'+self.phase;self.message = '停稳确认已暂停'

    def resume(self):
        with self.lock:
            self.require_ready()
            if self.phase == 'paused_collection':
                self.phase='collecting';self.message='继续等待 WS63 采集确认'
            elif self.phase == 'paused_dwell':
                self.until = self.clock()+self.remaining;self.phase = 'dwelling';self.message = '继续停留计时'
            elif self.phase == 'paused_navigation':
                self._navigate_current()
            elif self.phase == 'paused_return':
                self._return()
            elif self.phase == 'paused_departure':
                self._depart()
            elif self.phase == 'paused_dock':
                self._return()
            elif self.phase in ('paused_settling','paused_settling_return','paused_settling_departure'):
                self.phase = self.phase[len('paused_'):];self.until = self.clock()+10
                self.message = '等待底盘停稳'
            else:
                raise ValueError('当前没有可以继续的暂停任务')

    def stop(self):
        with self.lock:
            if hasattr(self.backend,'set_survey_planning'):self.backend.set_survey_planning(False)
            self.dock_retry_pending=False;self.dock_failure_reconcile=False
            self.backend.survey_speed_limit=None
            self.collection.cancel()
            self.backend.stop()
            if self.operation and self.backend.operation(self.operation)['status'] not in TERMINAL:
                self.backend.cancel(self.operation);self.phase = 'canceling';self.pending = 'stop'
                self.message = '停止中，等待导航确认取消'
            else:
                self.operation = None;self.phase = 'idle';self.pending = None;self.message = '任务已停止'
            self.event('停止任务')

    def _fail(self, message):
        if hasattr(self.backend,'set_survey_planning'):self.backend.set_survey_planning(False)
        self.dock_failure_reconcile=self.phase=='docking'
        retryable=(self.phase=='docking' and self.dock_retry_count<MAX_DOCK_RETRIES and
                   any(t in message for t in ('桩旁轮程已达到保护上限但未检测到充电','对接超时，未收到充电反馈','原生对接达到行程或时间上限但未充电','桩旁连续对接未取得充电反馈')))
        if self.dock_retry_started is not None and self.clock()-self.dock_retry_started>=180:retryable=False
        self.dock_retry_pending=retryable
        self.backend.survey_speed_limit=None
        self.collection.cancel()
        self.backend.stop()
        if self.dock_failure_reconcile:
            self.operation = None
            self.dock_review_until = self.clock() + 6.0
            self.dock_review_error = message
            self.message = '停车复核中，等待停稳并确认充电'
            self.event(self.message)
            return
        if self.operation and self.backend.operation(self.operation)['status'] not in TERMINAL:
            self.backend.cancel(self.operation);self.pending = 'fail'
            self.phase = 'canceling'
        else:
            self.phase = 'failed';self.operation = None
        self.message = message;self.event(message)

    def _retry_docking_if_needed(self,health,now):
        if self.phase != 'docking' or not self.dock_failure_reconcile:
            return False
        stationary = health.get('stationary', False)
        if stationary and health.get('charging_stable', health.get('charging')):
            self.dock_retry_pending = self.dock_failure_reconcile = False
            self.phase = 'docked'
            self.message = '停车复核已稳定充电，回充完成'
            self.event(self.message)
            return True
        if now < self.dock_review_until:
            return True
        retry = self.dock_retry_pending and stationary and health.get('ready') and health.get('localization_confirmed')
        self.dock_retry_pending = self.dock_failure_reconcile = False
        if retry:
            self.dock_retry_count += 1
            if self.dock_retry_started is None:
                self.dock_retry_started=now
                self.dock_retry_start_travel=getattr(self.backend,'wheel_distance',None)
            try:
                if getattr(self,'dock_direct_retry_streak',0)<1 and self._start_direct_docking(health,'自动复核重试'):
                    self.dock_direct_retry_streak=getattr(self,'dock_direct_retry_streak',0)+1
                    return True
                self.event('直接重试已使用或红外接管条件不足，检查退出空间后重新接近')
                self.operation=self.backend.begin_dock_retry_exit()
                self.phase='dock_retry_exit';self.message=f'回充第{self.dock_retry_count+1}/{MAX_DOCK_ATTEMPTS}次尝试：确认前方空间后低速退出10厘米'
                self.event(self.message)
            except Exception as error:
                self.phase='docking';self.dock_failure_reconcile=True;self.dock_retry_pending=False
                self.dock_review_until=now+6.
                self.dock_review_error='回充退出检查未通过，保持停车：'+str(error)
                self.message=self.dock_review_error+'；再复核6秒充电反馈'
                self.event(self.message)
        else:
            self.phase = 'failed'
            self.message = self.dock_review_error
            self.event(self.message)
        return True

    def tick(self):
        with self.lock:
            self.backend.tick()
            health = self.backend.snapshot();now = self.clock()
            initial_travel=getattr(self,'dock_retry_start_travel',None)
            if (self.dock_retry_started is not None and self.dock_retry_count>0
                and self.phase in ('returning','docking','dock_retry_exit')
                and initial_travel is not None and getattr(self.backend,'wheel_distance',initial_travel)-initial_travel>=.8):
                self._fail('回充自动恢复累计轮程达到0.8米上限，已停车');return
            if (self.dock_retry_started is not None and self.dock_retry_count>0
                and self.phase in ('returning','docking') and not self.dock_failure_reconcile
                and now-self.dock_retry_started>=180):
                self._fail('回充自动恢复达到180秒上限，已停车');return
            if self.phase=='failed' and health.get('charging_stable') and health.get('stationary'):
                self.backend.stop();self.operation=None;self.phase='docked'
                self.message='此前任务失败，现已确认持续充电；原失败记录保留'
                self.event(self.message);return
            if self.phase=='dock_retry_exit':
                if health.get('stationary') and health.get('charging_stable',False):
                    self.backend.stop();self.operation=None;self.phase='docked';self.message='退出前复核已稳定充电，取消重试';self.event(self.message);return
                if self.dock_retry_started is not None and now-self.dock_retry_started>=180:
                    self.backend.stop();self.phase='failed';self.message='回充自动恢复达到180秒上限，已停车';self.event(self.message);return
                result=self.backend.operation(self.operation)
                if result['status'] not in TERMINAL:return
                if result['status']!='succeeded':
                    self.backend.stop();self.phase='failed';self.message=result.get('error') or '退出未完成，停止自动重试';self.event(self.message);return
                if not health.get('stationary'):return
                self.dock_direct_retry_streak=0
                self.operation=None;self.return_to_dock(automatic=True);return
            if self._retry_docking_if_needed(health,now):return
            self.collection.observe({**health,'survey_active':self.phase in ('departing','navigating','settling_departure','settling')})
            if self.phase in QUIET or self.phase in ('returning','docking'):
                self.backend.survey_speed_limit=None
            if self.collection.snapshot()['mode']=='broadcast' and self.phase=='navigating':
                pose=health.get('pose')
                if pose and not (-.15<=pose[0]<=5.4 and -7.9<=pose[1]<=.6):
                    self._fail('已超出北区采集边界，保持停车并保留数据，请核对路线');return
            calibrated=health.get('dock_localization',{}).get('calibrations',0)
            if calibrated>getattr(self,'dock_calibrations_seen',0):
                self.dock_calibrations_seen=calibrated
                text='已按保存的充电桩位自动校准定位，无需再次确认'
                self.event(text)
                if self.phase in QUIET and self.phase!='failed':
                    self.message=('正在充电；' if health.get('charging') else '')+text
            if self.phase in ('navigating','returning','departing','dwelling','collecting','paused_collection','settling','settling_return','settling_departure') and not health['ready']:
                self._fail('定位或传感器未就绪，已停止：'+health['readiness']);return
            if self.phase in ('navigating','departing','dwelling','collecting','paused_collection','settling','settling_departure'):
                voltage = health.get('voltage')
                self.low_since = (self.low_since if self.low_since is not None else now) if voltage is not None and voltage < 10.0 else None
                if self.low_since is not None and now-self.low_since >= 20:
                    self.event('电压持续低于 10.0 V，结束任务并回充');self.return_to_dock();return
            if self.phase == 'canceling':
                if self.operation and self.backend.operation(self.operation)['status'] not in TERMINAL:
                    return
                action = self.pending;self.pending = None;self.operation = None
                if action == 'return':
                    if health['ready']:
                        self._return()
                    else:
                        self._fail('已停车，导航未就绪，未开始回充')
                elif action == 'dock_handoff':
                    if self._can_handoff_dock(health):
                        self.event('导航取消已确认，切换厂家红外对接')
                        self._begin_arrival(True)
                    else:
                        self._fail('桩前接管条件已改变，保持停车，请核对位置与红外')
                elif action in ('pause_navigation','pause_return','pause_departure'):
                    self.phase = 'paused_navigation' if action == 'pause_navigation' else 'paused_return' if action == 'pause_return' else 'paused_departure'
                    self.message = '已停车，可以继续';self.event(self.message)
                elif action == 'fail':
                    self.phase = 'failed'
                else:
                    self.phase = 'idle';self.message = '任务已停止'
            elif self.phase in ('navigating','returning','departing'):
                result = self.backend.operation(self.operation)
                if self.phase=='navigating':
                    name=self.run[self.index]['name'];stage=result.get('stage')
                    self.message=(f'{name}：'+(result.get('detail') or '等待通路恢复，随后自动继续') if stage=='waiting' else
                                  f'{name}：正在低速收敛位置与朝向' if stage=='terminal' else
                                  f'{name} 位置已到，正在对正朝向' if stage=='heading' else
                                  f'正在前往 {name}，跟踪可通行路线')
                if self.phase == 'returning' and self._can_handoff_dock(health):
                    if result['status'] not in TERMINAL:
                        self.backend.cancel(self.operation);self.pending = 'dock_handoff'
                        self.phase = 'canceling';self.message = '已到红外接管区域，停止导航并等待确认'
                    else:
                        self.operation = None;self.event('已停在桩前红外接管区域')
                        self._begin_arrival(True)
                    return
                if result['status'] not in TERMINAL:
                    return
                if result['status'] != 'succeeded':
                    self._fail(result.get('error') or '目标导航失败，已停车');return
                if self.phase=='navigating' and result.get('quality')=='near':
                    self.event(f'{self.run[self.index]["name"]} 已在附近停稳，位置误差 {result["position_error"]*100:.1f} cm、朝向误差 {result["yaw_error_deg"]:.1f}°，按可用精度继续任务')
                self.operation = None
                if self.phase == 'departing':
                    if health.get('stationary',False):
                        if health.get('charging'):
                            self._fail('驶离桩后仍反馈充电，请核对是否真正移动');return
                        self._navigate_current()
                    else:
                        self.phase = 'settling_departure';self.until = now+10;self.message = '离桩后等待底盘停稳'
                    return
                returning = self.phase == 'returning'
                if health.get('stationary',False):
                    self._begin_arrival(returning)
                else:
                    self.phase = 'settling_return' if returning else 'settling'
                    self.until = now+10;self.message = '导航已到达，等待底盘停稳'
            elif self.phase in ('settling','settling_return','settling_departure'):
                if health.get('stationary',False):
                    if self.phase == 'settling_departure':
                        if health.get('charging'):
                            self._fail('驶离桩后仍反馈充电，请核对是否真正移动')
                        else:
                            self._navigate_current()
                    else:
                        self._begin_arrival(self.phase == 'settling_return')
                elif now >= self.until:
                    self._fail('已到达但未确认底盘停稳，任务已停止')
            elif self.phase == 'dwelling' and now >= self.until:
                p = self.run[self.index];self.event(f'{p["name"]} 停留结束')
                self._advance()
            elif self.phase in ('collecting','paused_collection'):
                status=self.collection.snapshot()
                if not health.get('stationary'):
                    self._fail('采集期间底盘未停稳，本轮已作废');return
                if status['stage']=='error':
                    self._fail('WS63 采集未完成：'+str(status['error']));return
                if status['stage']=='done' and self.phase=='collecting':
                    self.event(f'{self.run[self.index]["name"]} WS63 采集完成')
                    self._advance()
            elif self.phase=='canceling_collection':
                status=self.collection.snapshot()
                if status['stage'] in ('canceled','done'):
                    self._return()
                elif status['stage']=='error':
                    self._fail('串口取消尚未确认，保持停车：'+str(status['error']))
            elif self.phase == 'docking':
                if health.get('charging_stable' ,health.get('charging')):
                    self.backend.stop();self.phase = 'docked';self.message = '回充完成：底盘已反馈正在充电'
                    self.event(self.message)
                elif self.backend.docking_error:
                    self._fail(self.backend.docking_error)
                elif now >= self.until:
                    self._fail('对接超时，未收到充电反馈，已停止')
                elif health.get('docking_contact_stage')=='charging_observed':
                    self.message='检测到电流，厂家程序仍在完成贴合并等待停轮'
                elif health.get('docking_contact_stage')=='verify':
                    self.message='已检测充电接触，已停车，正在确认持续充电'
                elif health.get('docking_contact_stage')=='reseat':
                    self.message='触点未压实，正在由厂家红外程序短距离补贴合'
            elif self.phase=='docked' and health.get('charging_stable') is False:
                if not hasattr(self,'charge_lost_since'):self.charge_lost_since=now
                if now-self.charge_lost_since>=2:
                    self.phase='idle';self.message='充电接触已中断，小车保持停稳，请检查触点或主动回充'
                    self.event(self.message);del self.charge_lost_since
            elif hasattr(self,'charge_lost_since'):
                del self.charge_lost_since

    def state(self):
        with self.lock:
            health = self.backend.snapshot()
            remaining = max(0, self.until-self.clock()) if self.phase == 'dwelling' else self.remaining if self.phase == 'paused_dwell' else None
            return {'phase': self.phase, 'message': self.message, 'points': self.points,
                    'run': self.run, 'index': self.index, 'remaining': remaining,
                    'auto_return': self.auto_return, 'health': health, 'collection':self.collection.snapshot(),
                    'dock': self.backend.dock_pose,
                    'events': list(self.events), 'map': self.grid.info()}
