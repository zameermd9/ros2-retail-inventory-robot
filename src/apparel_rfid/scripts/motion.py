import math
import time
import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

class Motion(Node):
    def __init__(self):
        super().__init__('apparel_rfid_patrol')
        self.nav=ActionClient(self,NavigateToPose,'navigate_to_pose')
        self.buffer=Buffer()
        self.listener=TransformListener(self.buffer,self)
        self.active=None
        self.speed=math.inf
        self.odom_time=-math.inf
        self.create_subscription(Odometry,'/odom',self.odom,qos_profile_sensor_data)

    def now(self):
        return self.get_clock().now().nanoseconds/1e9

    def odom(self,msg):
        twist=msg.twist.twist
        self.speed=max(abs(twist.linear.x),abs(twist.angular.z))
        self.odom_time=self.now()

    def pose(self):
        try:
            tf=self.buffer.lookup_transform("map","base_footprint",Time())
        except TransformException:
            return None
        stamp=tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9
        if abs(self.now()-stamp)>2.:
            return None
        v,q=tf.transform.translation,tf.transform.rotation
        return v.x,v.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

    def at_pad(self):
        p=self.pose()
        return (p is not None and math.hypot(p[0],p[1])<=.20 and abs(p[2])<=.17
                and self.now()-self.odom_time<.6 and self.speed<.035)

    def wait(self,condition,wall_seconds):
        end=time.monotonic()+wall_seconds
        while rclpy.ok() and time.monotonic()<end:
            if condition():
                return True
            rclpy.spin_once(self,timeout_sec=.1)
        return False

    def go(self,x,y,yaw):
        goal=NavigateToPose.Goal()
        pose=goal.pose
        pose.header.frame_id="map"
        pose.header.stamp=self.get_clock().now().to_msg()
        pose.pose.position.x=float(x)
        pose.pose.position.y=float(y)
        pose.pose.orientation.z=math.sin(yaw/2)
        pose.pose.orientation.w=math.cos(yaw/2)
        future=self.nav.send_goal_async(goal)
        if not self.wait(future.done,30):
            raise RuntimeError("Nav2 did not acknowledge the goal. Stop the launch before retrying.")
        self.active=future.result()
        if not self.active.accepted:
            self.active=None
            return False
        done=self.active.get_result_async()
        start=self.now()
        self.wait(lambda:done.done() or self.now()-start>150,1200)
        if not done.done():
            self.active.cancel_goal_async()
            if not self.wait(done.done,30):
                raise RuntimeError("Nav2 cancellation unconfirmed. Stop the simulation.")
        ok=done.result().status==GoalStatus.STATUS_SUCCEEDED
        self.active=None
        return ok
