"""Depth obstacle candidate pipeline. No motor publishers or motion actions."""
import json,math,time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image,CameraInfo,PointCloud2,PointField
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener,StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

from depth_geometry import project_depth, transform_points, filter_points

class DepthObstacles(Node):
    def __init__(self):
        super().__init__('low_obstacle_depth')
        for name,value in [('height_correction',0.),('lens_height',.15),('floor_height',0.),('min_height',.025),('pitch_correction_deg',0.),('calibration_verified',False),('enabled',False)]:
            self.declare_parameter(name,value)
        self.image=None;self.info=None;self.frames=0;self.started=time.monotonic()
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.origin_broadcaster=StaticTransformBroadcaster(self);self.origin_sent=False
        self.create_subscription(Image,'/camera/depth/image_raw',self.receive,qos_profile_sensor_data)
        self.create_subscription(CameraInfo,'/camera/depth/camera_info',lambda m:setattr(self,'info',m),qos_profile_sensor_data)
        self.obstacles=self.create_publisher(PointCloud2,'/vision/obstacles',qos_profile_sensor_data)
        self.clearing=self.create_publisher(PointCloud2,'/vision/clearing',qos_profile_sensor_data)
        self.candidates=self.create_publisher(PointCloud2,'/vision/obstacles_candidate',qos_profile_sensor_data)
        self.status=self.create_publisher(String,'/vision/status',10)
        self.create_timer(.2,self.process)
    def receive(self,m):self.image=m
    def cloud(self,points,stamp):
        a=np.asarray(points,dtype='<f4').reshape(-1,3)
        m=PointCloud2();m.header.frame_id='base_footprint';m.header.stamp=stamp
        m.height=1;m.width=len(a);m.is_dense=True;m.is_bigendian=False;m.point_step=12;m.row_step=12*len(a)
        m.fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,name in enumerate(('x','y','z'))]
        m.data=a.tobytes();return m
    def process(self):
        started=time.monotonic()
        result={'enabled':self.get_parameter('enabled').value,'ready':False}
        try:
            m=self.image;c=self.info
            if m is None or c is None:raise ValueError('等待深度图和内参')
            age=(self.get_clock().now().nanoseconds-m.header.stamp.sec*10**9-m.header.stamp.nanosec)/1e9
            if not 0<=age<.5:raise ValueError('深度图过期或时间戳异常')
            if c.k[0]<=0 or c.k[4]<=0:raise ValueError('相机内参无效')
            if m.encoding in ('16UC1','mono16'):
                dtype='>u2' if m.is_bigendian else '<u2';scale=.001;item=2
            elif m.encoding=='32FC1':
                dtype='>f4' if m.is_bigendian else '<f4';scale=1.;item=4
            else:raise ValueError('不支持的深度编码：'+m.encoding)
            depth=np.ndarray((m.height,m.width),dtype=dtype,buffer=bytes(m.data),strides=(m.step,item)).astype(float)*scale
            raw=project_depth(depth,c.k)
            if len(raw)<100:raise ValueError('有效深度覆盖不足')
            t=self.buffer.lookup_transform('camera_link',m.header.frame_id,Time.from_msg(m.header.stamp)).transform
            t.translation.x+=.10709;t.translation.y+=.00032
            t.translation.z=self.get_parameter('lens_height').value
            if not self.origin_sent:
                origin=TransformStamped();origin.header.frame_id='base_footprint';origin.child_frame_id='vision_depth_origin'
                origin.header.stamp=m.header.stamp;origin.transform.translation.x=t.translation.x
                origin.transform.translation.y=t.translation.y
                origin.transform.translation.z=t.translation.z
                origin.transform.rotation.w=1.;self.origin_broadcaster.sendTransform(origin);self.origin_sent=True
            p=transform_points(raw,t,0.,
                               self.get_parameter('pitch_correction_deg').value)
            obs,clear=filter_points(p,self.get_parameter('floor_height').value,self.get_parameter('min_height').value)
            self.candidates.publish(self.cloud(obs,m.header.stamp))
            calibrated=self.get_parameter('calibration_verified').value
            if result['enabled'] and not calibrated:
                raise ValueError('安装坐标尚未通过静态校准，禁止接入避障')
            if result['enabled'] and calibrated:
                self.obstacles.publish(self.cloud(obs,m.header.stamp));self.clearing.publish(self.cloud(clear,m.header.stamp))
            self.frames+=1
            low=obs[(abs(obs[:,1])<.20)&(obs[:,2]<.10)]
            result.update(ready=True,obstacle_points=len(obs),valid_points=len(raw),processed_frames=self.frames,
                          age_seconds=round(age,3),height_correction=self.get_parameter('height_correction').value,
                          calibration_verified=calibrated,
                          pitch_correction_deg=self.get_parameter('pitch_correction_deg').value,
                          minimum_obstacle_height=self.get_parameter('min_height').value,
                          forward_low_points=len(low),
                          forward_low_nearest_m=round(float(low[:,0].min()),3) if len(low) else None,
                          processing_ms=round((time.monotonic()-started)*1000,2))
        except Exception as error:result['error']=str(error)
        msg=String();msg.data=json.dumps(result,ensure_ascii=False);self.status.publish(msg)

def main():
    rclpy.init();node=DepthObstacles()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()

