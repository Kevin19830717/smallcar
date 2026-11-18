"""Native Spin must finish before a large-angle return can follow its path."""
import math
import time


def route_heading(path, pose, ahead=.45):
    if len(path)<2:
        raise ValueError('路线不足两个点，无法定向')
    travelled=0.
    target=path[-1]
    for a,b in zip(path,path[1:]):
        travelled+=math.hypot(b[0]-a[0],b[1]-a[1])
        if travelled>=ahead:
            target=b
            break
    return math.atan2(target[1]-pose[1],target[0]-pose[0])


class RouteAlignmentMixin:
    def begin_route_alignment(self, identity):
        op=self.ops[identity]
        if op['kind']!='return' or self.pose is None:
            return False
        if math.hypot(self.pose[0]-op['goal']['x'],self.pose[1]-op['goal']['y'])<=.2:
            return False
        heading=route_heading(self.path,self.pose)
        error=math.atan2(math.sin(heading-self.pose[2]),math.cos(heading-self.pose[2]))
        if abs(error)<=math.radians(45):
            return False
        if op.get('route_alignment_attempts',0)>=3:
            self.schedule_travel_recovery(op,'定向多次未完成')
            return True
        op.update(status='route_align_prepare',handle=None,started=time.monotonic(),
                  timeout=35.,route_heading=heading,route_alignment_failed=False,
                  recovery_message='先停车，再用原生Spin完成车头对齐')
        self.zero_motion()
        return True

    def route_alignment_tick(self, op, now):
        if not op['status'].startswith('route_align_'):
            return False
        if op['cancel']:
            self.zero_motion()
            return True
        if op['status'] in ('route_align_wait','route_align_running'):
            if now-op['started']>30 or now-self.last_scan>.5 or now-self.last_wheel>.5:
                op.update(cancel=True,status='canceling',settle_after_cancel='route_align_settle',
                          route_alignment_failed=True)
                if op['handle']:
                    op['handle'].cancel_goal_async()
                self.zero_motion()
            return True
        self.zero_motion()
        if now-op['started']>35:
            op.update(status='failed',error='定向停车确认超时')
            return True
        if self.stationary_since is None or now-self.stationary_since<.7 or now-self.last_wheel>.5:
            return True
        if self.pose is None or not self.snapshot()['ready']:
            op.update(status='failed',error='定向时定位或雷达未就绪')
            return True
        error=math.atan2(math.sin(op['route_heading']-self.pose[2]),
                         math.cos(op['route_heading']-self.pose[2]))
        if op['status']=='route_align_settle':
            if op.get('route_alignment_failed') or abs(error)>math.radians(15):
                self.schedule_travel_recovery(op,'原生定向受阻或未达到15度门限')
            else:
                # Successful measured alignment is real recovery progress.
                # Do not charge the following drive for time spent in Spin.
                op.pop('recovery_since',None)
                op['recovery_message']='车头已对齐，重新规划并重新开始行驶进度计时'
                self.recovery_replan(op)
            return True
        if not self.spin_action.server_is_ready():
            self.schedule_travel_recovery(op,'原生Spin服务未就绪')
            return True
        from nav2_msgs.action import Spin
        goal=Spin.Goal()
        goal.target_yaw=float(error)
        goal.time_allowance.sec=30
        op.update(status='route_align_wait',started=now,timeout=35.,
                  route_alignment_attempts=op.get('route_alignment_attempts',0)+1,
                  recovery_message='原生碰撞检查定向中，尚未放行路径跟踪')
        identity=self.active
        self.get_logger().info('ROUTE_ALIGN_GOAL '+str({
            'operation':identity,'heading':op['route_heading'],'relative_yaw':error}))
        self.spin_action.send_goal_async(goal).add_done_callback(
            lambda f:self.route_spin_accepted(f,identity))
        return True

    def route_spin_accepted(self, future, identity):
        with self.lock:
            op=self.ops[identity]
            try:
                handle=future.result()
                if not handle.accepted:
                    if op['cancel']:
                        op['status']='canceled'
                    else:
                        op.update(status='route_align_settle',route_alignment_failed=True)
                    return
                op['handle']=handle
                handle.get_result_async().add_done_callback(lambda f:self.route_spin_done(f,identity))
                if op['cancel']:
                    handle.cancel_goal_async()
                else:
                    op['status']='route_align_running'
            except Exception as error:
                op.update(status='failed',error='原生定向响应异常：'+str(error))
                self.zero_motion()

    def route_spin_done(self, future, identity):
        with self.lock:
            op=self.ops[identity]
            try:
                result=future.result()
                self.get_logger().info('ROUTE_ALIGN_RESULT '+str({
                    'operation':identity,'status':result.status}))
                if op['cancel']:
                    self.finish_native_cancellation(op)
                else:
                    op.update(status='route_align_settle',handle=None,started=time.monotonic(),
                              route_alignment_failed=result.status!=4)
            except Exception as error:
                op.update(status='failed',error='原生定向结果异常：'+str(error))
            self.zero_motion()
