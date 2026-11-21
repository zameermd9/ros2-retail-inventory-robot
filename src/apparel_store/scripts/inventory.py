#!/usr/bin/env python3
"""Run one inventory patrol; count only matching payloads decoded from camera pixels."""
import csv
import json
import math
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from pyzbar.pyzbar import ZBarSymbol, decode
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener, TransformException


class Inventory(Node):
    def __init__(self):
        super().__init__("apparel_inventory")
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.expected = None
        self.result = None
        self.last_frame = None
        self.speed = math.inf
        self.odom_time = -math.inf
        self.active = None
        self.create_subscription(Image,"/camera/image_raw",self.image,qos_profile_sensor_data)
        self.create_subscription(Odometry,"/odom",self.odom,qos_profile_sensor_data)
        manifest=Path(get_package_share_directory("apparel_store"))/"config/stations.json"
        self.stations=json.loads(manifest.read_text())
        self.found=set()
        folder=Path.home()/"store_inventory"/datetime.now().strftime("%Y%m%d-%H%M%S-%f")
        folder.mkdir(parents=True)
        self.folder=folder
        self.file=(folder/"inventory.csv").open("x",newline="")
        self.writer=csv.writer(self.file)
        self.writer.writerow(["sim_s","sku","garment","size","colour","price_eur","label_quantity","raw_qr"])
        self.file.flush()
        self.get_logger().info(f"Run report: {folder}")

    def now(self):
        return self.get_clock().now().nanoseconds/1e9

    def odom(self,msg):
        twist=msg.twist.twist
        self.speed=max(abs(twist.linear.x),abs(twist.angular.z))
        self.odom_time=self.now()

    def image(self,msg):
        if self.expected is None or self.result is not None:
            return
        stamp=msg.header.stamp.sec+1e-9*msg.header.stamp.nanosec
        if stamp<self.read_start or stamp==self.last_frame or not 0<=self.now()-stamp<1.:
            return
        self.last_frame=stamp
        channels={"rgb8":3,"bgr8":3,"rgba8":4,"bgra8":4,"mono8":1}.get(msg.encoding.lower())
        if not channels or msg.step<msg.width*channels or len(msg.data)!=msg.step*msg.height:
            return
        pixels=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)
        gray=np.ascontiguousarray(pixels[:,:msg.width*channels:channels])
        for qr in decode(gray,symbols=[ZBarSymbol.QRCODE]):
            raw=qr.data.decode("utf-8",errors="replace")
            fields=raw.split("|")
            if (len(fields)==6 and fields[0]==self.expected and
                    fields[4].isdigit() and fields[5].isdigit()):
                self.result=fields
                return

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

    def inspect(self,tag,attempt):
        # Alternate view on retry, aimed at the same physical label.
        lateral=.22 if attempt else 0.
        yaw=tag["yaw"]
        x=tag["x"]+lateral
        target_x=tag["x"]
        target_y=tag["y"]+1.34*math.sin(yaw)
        if not self.go(x,tag["y"],math.atan2(target_y-tag["y"],target_x-x)):
            return "navigation_failed"
        self.result=None
        self.read_start=self.now()
        self.expected=tag["sku"]
        self.wait(lambda:self.result is not None or self.now()-self.read_start>7,90)
        self.expected=None
        if self.result is None:
            return "unread"
        sku,style,size,colour,cents,qty=self.result
        self.writer.writerow([round(self.now(),2),sku,style,size,colour,
                              f"{int(cents)/100:.2f}",qty,"|".join(self.result)])
        self.file.flush()
        self.found.add(sku)
        return "verified"

    def run(self):
        self.get_logger().info("Waiting for Nav2, map localization and standstill on the home pad")
        if not self.wait(lambda:self.nav.server_is_ready() and self.at_pad(),180):
            raise RuntimeError("No localized, stationary robot at home. Check /scan, /odom, /map and Nav2.")
        last={}
        for attempt in range(2):
            pending=[s for s in self.stations if s["sku"] not in self.found]
            while pending:
                p=self.pose()
                if p is None:
                    if not self.wait(lambda:self.pose() is not None,20):
                        raise RuntimeError("Lost map localization")
                    p=self.pose()
                tag=min(pending,key=lambda s:math.hypot(s["x"]-p[0],s["y"]-p[1]))
                pending.remove(tag)
                self.get_logger().info(f"Visit {tag['sku']} (pass {attempt+1})")
                last[tag["sku"]]=self.inspect(tag,attempt)
                self.get_logger().info(f"{tag['sku']}: {last[tag['sku']]} ({len(self.found)}/{len(self.stations)})")
        self.get_logger().info("Returning to home charging pad")
        reached=self.go(0.,0.,0.) or self.go(0.,0.,0.)
        docked=False
        if reached and self.wait(self.at_pad,30):
            start=self.now()
            # Require a continuous stationary interval at the dock.
            while self.at_pad() and self.now()-start<.7:
                if not self.wait(lambda:self.now()-start>=.7 or not self.at_pad(),15):
                    break
            docked=self.at_pad() and self.now()-start>=.7
        missing={k:v for k,v in last.items() if k not in self.found}
        report={"expected":len(self.stations),"verified":len(self.found),"missing":missing,
                "stationary_at_home_pad":docked,"complete":docked and not missing}
        (self.folder/"summary.json").write_text(json.dumps(report,indent=2))
        self.get_logger().info(json.dumps(report))
        return report["complete"]


def main():
    rclpy.init()
    node=Inventory()
    success=False
    try:
        success=node.run()
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        node.get_logger().error(str(exc))
    finally:
        if node.active and node.active.accepted and rclpy.ok():
            cancel=node.active.cancel_goal_async()
            node.wait(cancel.done,5)
        node.file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__=="__main__":
    main()
