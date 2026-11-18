"""Bounded travel recovery using the native collision-checked BackUp action."""
import math
import time
from recovery_policy import rear_clear, path_progress


class TravelRecoveryMixin:
    def schedule_travel_recovery(self, op, message):
        now = time.monotonic()
        op.setdefault('recovery_since', now)
        attempts = op.get('travel_retries', 0)
        if attempts >= 2 or now-op['recovery_since'] >= 30:
            detail=message.replace('，等待可通行后自动继续','')
            op.update(status='failed', error='恢复次数或等待时间已用尽，任务已停止。请检查通道后重新开始：'+detail)
            self.zero_motion()
            return
        op.update(travel_retries=attempts+1, recovery_message='遇阻恢复中：'+message,
                  retry_at=now+2., started=now, timeout=35., handle=None,
                  status='travel_recovery_wait')
        self.zero_motion()
        refresh=getattr(self, 'refresh_recovery_costmaps', None)
        if refresh:
            refresh(op)

    def refresh_recovery_costmaps(self, op):
        now=time.monotonic()
        if self.stationary_since is None or now-self.stationary_since < .7 or now-self.last_wheel > .4:
            return
        from nav2_msgs.srv import ClearEntireCostmap
        if not hasattr(self, '_recovery_costmap_clients'):
            self._recovery_costmap_clients=[
                self.create_client(ClearEntireCostmap, '/'+name+'_costmap/clear_entirely_'+name+'_costmap')
                for name in ('global','local')]
        op['costmap_refresh_pending']=0
        op['costmap_refresh_at']=now
        for client in self._recovery_costmap_clients:
            if not client.service_is_ready():
                continue
            op['costmap_refresh_pending']+=1
            future=client.call_async(ClearEntireCostmap.Request())
            def done(future, operation=op):
                with self.lock:
                    operation['costmap_refresh_pending']-=1
                    try:
                        future.result()
                    except Exception:
                        operation['costmap_refresh_error']=True
                        operation['recovery_message']='障碍层刷新失败，将按现有实时地图重新规划'
                    operation['retry_at']=max(operation.get('retry_at',0.),time.monotonic()+1.5)
            future.add_done_callback(done)


    def manual_refresh_obstacles(self):
        with self.lock:
            health=self.snapshot()
            if not health.get('ready') or not health.get('stationary'):
                raise ValueError('请等待导航就绪并确认小车停稳')
            op={}
            self.refresh_recovery_costmaps(op)
            if ('costmap_refresh_pending' not in op or
                    not all(c.service_is_ready() for c in self._recovery_costmap_clients)):
                raise ValueError('障碍地图刷新服务未就绪')
        deadline=time.monotonic()+6.
        while time.monotonic()<deadline:
            with self.lock:
                if op.get('costmap_refresh_error'):
                    raise ValueError('障碍地图刷新失败，请检查导航服务')
                if not op['costmap_refresh_pending'] and time.monotonic()>=op.get('retry_at',0.):
                    health=self.snapshot()
                    if not health.get('stationary') or time.monotonic()-self.last_scan>.5:
                        raise ValueError('刷新后小车未停稳或雷达数据过期，请检查')
                    return
            time.sleep(.05)
        raise ValueError('刷新等待超时，请检查导航与雷达连接')

    def recovery_replan(self, op):
        from nav2_msgs.action import ComputePathToPose
        now = time.monotonic()
        op.update(status='planning', started=now, timeout=20., cancel=False, handle=None)
        goal = ComputePathToPose.Goal()
        goal.goal = self.stamped(op['goal'])
        goal.planner_id = 'WaypointGrid' if op['sequential'] else 'GridBased'
        goal.use_start = False
        identity = self.active
        self.plan.send_goal_async(goal).add_done_callback(lambda f: self.plan_accepted(f, identity))

    def rear_recovery_safe(self, moving=False):
        scan = getattr(self, 'rear_scan', None)
        now = time.monotonic()
        if (scan is None or now-getattr(self, 'rear_scan_at', 0.) > .4 or
                now-self.last_wheel > .4 or self.pose is None or not self.confirmed or
                (not moving and (self.stationary_since is None or now-self.stationary_since < .7)) or
                self.charging or self.charging_stable):
            return False
        if not self.snapshot().get('ready'):
            return False
        if abs(time.time()-scan.header.stamp.sec-scan.header.stamp.nanosec/1e9) > .5:
            return False
        try:
            from rclpy.time import Time
            t = self.tf.lookup_transform('base_footprint', scan.header.frame_id,
                                         Time.from_msg(scan.header.stamp)).transform
            angle = math.atan2(2*(t.rotation.w*t.rotation.z+t.rotation.x*t.rotation.y),
                               1-2*(t.rotation.y*t.rotation.y+t.rotation.z*t.rotation.z))
            return rear_clear(scan, t.translation.x, t.translation.y, angle,
                              self.recovery_half_length, self.recovery_half_width)
        except Exception:
            return False

    def travel_recovery_tick(self, op, now):
        status = op['status']
        if op.get('kind') not in ('waypoint', 'return'):
            return False
        if status == 'navigating' and self.pose is not None:
            distance = math.hypot(self.pose[0]-op['goal']['x'], self.pose[1]-op['goal']['y'])
            if path_progress(op, self.pose, self.wheel_distance, self.wheel_turn, now, self.path):
                op.pop('recovery_since', None)
            if now-op.get('recovery_since', now) >= 30:
                op['error'] = '恢复后30秒仍无有效进展，已停车'
                self.cancel(self.active)
                return True
            if distance > .20 and now-op['progress_at'] >= 30:
                op.update(cancel=True, status='canceling', settle_after_cancel='travel_recovery_wait',
                          retry_at=now+2., recovery_message='30秒沿路径无进展且朝向未改善，停车后重新规划')
                # Charge this recovery attempt only after the native action has stopped.
                op['recovery_needs_schedule'] = True
                if op['handle']:
                    op['handle'].cancel_goal_async()
                self.zero_motion()
                return True
        if status not in ('travel_recovery_wait', 'backup_wait', 'backing_up', 'travel_recovery_settle'):
            return False
        if op['cancel']:
            self.zero_motion()
            return True
        if now-op.get('recovery_since', now) >= 30:
            op['error'] = '遇阻恢复30秒仍无有效进展，已停车'
            self.cancel(self.active)
            return True
        if status in ('backup_wait', 'backing_up'):
            # Recheck current rear observations during movement. Stationarity is
            # checked before dispatch; native BackUp owns collision checking in motion.
            if now-op['started'] > 6. or not self.rear_recovery_safe(moving=True):
                op['error'] = '后退恢复超时或后方雷达过期，已停车'
                self.cancel(self.active)
            return True
        self.zero_motion()
        if self.stationary_since is None or now-self.stationary_since < .7 or now-self.last_wheel > .4:
            return True
        if status == 'travel_recovery_settle':
            self.recovery_replan(op)
            return True
        if op.pop('recovery_needs_schedule', False):
            self.schedule_travel_recovery(op, op.get('recovery_message', '导航停滞'))
            return True
        if op.get('costmap_refresh_pending',0):
            if now-op['costmap_refresh_at']>4:
                op.update(status='failed',error='障碍地图刷新响应超时，任务已停止')
            return True
        if now < op['retry_at']:
            return True
        if op.get('travel_retries') == 2 and self.rear_recovery_safe() and self.backup_action.server_is_ready():
            from nav2_msgs.action import BackUp
            goal = BackUp.Goal()
            goal.target.x = -.12
            goal.speed = .04
            goal.time_allowance.sec = 6
            op.update(status='backup_wait', started=now, timeout=6.,
                      recovery_message='后方检查通过，低速后退12厘米后重新规划')
            identity = self.active
            self.backup_action.send_goal_async(goal).add_done_callback(
                lambda f: self.backup_accepted(f, identity))
        else:
            op['recovery_message'] = '停车复核完成，重新计算可通行路线'
            self.recovery_replan(op)
        return True

    def backup_accepted(self, future, identity):
        with self.lock:
            op = self.ops[identity]
            try:
                handle = future.result()
                if not handle.accepted:
                    if op['cancel']:
                        op['status'] = 'canceled'
                    else:
                        op.update(status='travel_recovery_settle', handle=None)
                    return
                op['handle'] = handle
                handle.get_result_async().add_done_callback(lambda f: self.backup_done(f, identity))
                if op['cancel']:
                    handle.cancel_goal_async()
                else:
                    op['status'] = 'backing_up'
            except Exception as error:
                op.update(status='failed', error='后退动作响应异常：'+str(error))
                self.zero_motion()

    def backup_done(self, future, identity):
        with self.lock:
            op = self.ops[identity]
            try:
                result = future.result()
                if op['cancel']:
                    op['status'] = 'failed' if op.get('error') else 'canceled'
                elif result.status == 4:
                    op.update(status='travel_recovery_settle', handle=None)
                else:
                    op.update(status='failed', error='后退受到阻挡或未完成，请检查后方')
            except Exception as error:
                op.update(status='failed', error='后退动作异常：'+str(error))
            self.zero_motion()
