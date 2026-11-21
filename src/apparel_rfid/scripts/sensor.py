#!/usr/bin/env python3
"""Gazebo RFID approximation: tag coordinates, four antenna lobes and finite range."""
import json
import math
from pathlib import Path
import rclpy
from ament_index_python.packages import get_package_share_directory as share
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

# Locations relative to the chassis centre, z measured from base_footprint.
ANTENNAS = (('front', .33, 0., 0.), ('left', 0., .28, math.pi/2),
            ('rear', -.33, 0., math.pi), ('right', 0., -.28, -math.pi/2))
ANTENNA_Z = .50
RANGE = 2.20

def visible(tags, x, y, yaw):
    reads = []
    c, s = math.cos(yaw), math.sin(yaw)
    for tag in tags:
        best = None
        for name, ax, ay, heading in ANTENNAS:
            dx = tag['x'] - (x+c*ax-s*ay)
            dy = tag['y'] - (y+s*ax+c*ay)
            d = math.sqrt(dx*dx + dy*dy + (tag['z']-ANTENNA_Z)**2)
            projection = dx*math.cos(yaw+heading) + dy*math.sin(yaw+heading)
            if d <= RANGE and projection >= math.hypot(dx,dy)*math.cos(math.radians(53)):
                if best is None or d < best[0]:
                    best = d, name
        if best:
            reads.append([tag['epc'],best[1],round(best[0],3)])
    return reads

class Reader(Node):
    def __init__(self):
        super().__init__('garment_rfid_reader')
        self.tags=json.loads((Path(share('apparel_rfid'))/'config/garments.json').read_text())
        self.pub=self.create_publisher(String,'/rfid/reads',5)
        self.sub=self.create_subscription(Odometry,'/rfid/model_pose',self.update,qos_profile_sensor_data)

    def update(self,m):
        p=m.pose.pose
        q=p.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        reads=visible(self.tags,p.position.x,p.position.y,yaw)
        speed=max(abs(m.twist.twist.linear.x),abs(m.twist.twist.linear.y),abs(m.twist.twist.angular.z))
        msg=String()
        msg.data=json.dumps(dict(time=stamp, dock=(math.hypot(p.position.x,p.position.y)<.20
                            and abs(yaw)<.20 and speed<.035), reads=reads))
        self.pub.publish(msg)

if __name__=='__main__':
    rclpy.init()
    n=Reader()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally:
        n.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
