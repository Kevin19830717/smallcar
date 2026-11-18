"""Bounded forward separation before another OEM docking attempt."""
import math,time,uuid
from recovery_policy import rear_clear
class DockExitMixin:
    def dock_exit_safe(self,moving=False):
        scan=getattr(self,'rear_scan',None);now=time.monotonic()
        def deny(reason):self.dock_exit_block_reason=reason;return False
        if scan is None or now-self.rear_scan_at>.4:return deny('原始雷达超过0.4秒未更新')
        if now-self.last_wheel>.4:return deny('底盘反馈超过0.4秒未更新')
        if not self.pose or not self.confirmed:return deny('定位未确认或位置不可用')
        if self.charging or self.charging_stable:return deny('仍有充电标志，禁止退出；需要继续复核触点与电流')
        if not self.snapshot()['ready']:return deny('导航或传感器未就绪')
        if not moving and (self.stationary_since is None or now-self.stationary_since<.7):return deny('尚未连续停稳0.7秒')
        if abs(time.time()-scan.header.stamp.sec-scan.header.stamp.nanosec/1e9)>.5:return deny('雷达时间戳偏差超过0.5秒')
        try:
            from rclpy.time import Time
            t=self.tf.lookup_transform('base_footprint',scan.header.frame_id,Time.from_msg(scan.header.stamp)).transform
            q=t.rotation;angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            safe=rear_clear(scan,-t.translation.x,-t.translation.y,angle+math.pi,self.recovery_half_length,self.recovery_half_width)
            if not safe:return deny('前向雷达覆盖不足或退出扫掠区有障碍，保持停车')
            self.dock_exit_block_reason=None;return True
        except Exception as error:return deny('退出坐标变换不可用：'+str(error))
    def begin_dock_retry_exit(self):
        from nav2_msgs.action import DriveOnHeading
        from rclpy.action import ActionClient
        with self.lock:
            if not self.dock_exit_safe():raise ValueError(getattr(self,'dock_exit_block_reason','桩前退出检查未通过'))
            if self.active and self.ops[self.active]['status'] not in ('succeeded','failed','canceled'):
                raise ValueError('上一动作尚未结束，保持停车')
            if not hasattr(self,'dock_exit_action'):self.dock_exit_action=ActionClient(self,DriveOnHeading,'/drive_on_heading')
            if not self.dock_exit_action.server_is_ready():raise ValueError('原生直线退出服务未就绪，保持停车')
            now=time.monotonic();identity=uuid.uuid4().hex;p=self.pose
            self.ops[identity]={'status':'dock_exit_wait','kind':'departure','goal':{'x':p[0]+.10*math.cos(p[2]),'y':p[1]+.10*math.sin(p[2]),'yaw':p[2]},
                'started':now,'created':now,'timeout':12.,'cancel':False,'handle':None,'heading_attempts':0,'arrival_retries':0,
                'xy_tolerance':.03,'yaw_tolerance':.2,'sequential':False,'exit_wheel_start':self.wheel_distance}
            self.active=identity;self.path=[]
            goal=DriveOnHeading.Goal();goal.target.x=.10;goal.speed=.04;goal.time_allowance.sec=10
            self.dock_exit_action.send_goal_async(goal).add_done_callback(lambda f:self.dock_exit_accepted(f,identity))
            return identity
    def dock_exit_accepted(self,future,identity):
        with self.lock:
            op=self.ops[identity]
            try:
                handle=future.result()
                if not handle.accepted:raise ValueError('原生退出动作被拒绝')
                op['handle']=handle
                handle.get_result_async().add_done_callback(lambda f:self.dock_exit_done(f,identity))
                if op['cancel']:handle.cancel_goal_async()
                else:op['status']='dock_exit_running'
            except Exception as error:op.update(status='failed',error=str(error));self.zero_motion()
    def dock_exit_done(self,future,identity):
        with self.lock:
            op=self.ops[identity]
            try:
                result=future.result()
                if op['cancel']:self.finish_native_cancellation(op)
                elif result.status==4:op.update(status='succeeded',handle=None)
                else:op.update(status='failed',error='退出动作未完成，停止自动重试')
            except Exception as error:op.update(status='failed',error=str(error))
            self.zero_motion()
    def dock_exit_tick(self,op,now):
        if not op['status'].startswith('dock_exit_'):return False
        if (now-op['started']>10. or self.wheel_distance-op['exit_wheel_start']>.16 or not self.dock_exit_safe(moving=True)):
            op['error']='退出受阻、反馈中断或超过退出限额，已停车';self.cancel(self.active)
        return True
