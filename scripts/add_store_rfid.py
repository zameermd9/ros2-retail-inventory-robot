#!/usr/bin/env python3
"""Install a scheduled, simulated RFID mission alongside the existing apparel_store package."""
import argparse
import ast
import copy
import json
import math
import os
import re
import shutil
import sys
import tempfile
from pathlib import Path
import xml.etree.ElementTree as ET

def put(root, name, source):
    path=root/name
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(source)
    if path.suffix=='.py':
        compile(source,str(path),'exec')
        path.chmod(0o755)

def write_xml(root,path):
    ET.indent(root)
    root_tree=ET.ElementTree(root)
    root_tree.write(path,encoding='utf-8',xml_declaration=True)

def replace(s,old,new):
    if s.count(old)!=1:
        raise ValueError(f'Expected one occurrence of {old!r} in store.launch.py. Check the source version.')
    return s.replace(old,new)

def build(out,src):
    store=src/'apparel_store'
    amr=src/'warehouse_amr'
    for folder in ('config','launch','scripts','urdf','worlds'):
        (out/folder).mkdir(parents=True)
    # Use the user's working navigation settings, collision handling, and visualization.
    launch=(store/'launch/store.launch.py').read_text()
    if 'scratch = TemporaryDirectory(' in launch:
        launch=replace(launch,'scratch = TemporaryDirectory(prefix="apparel_nav_")',
            'import atexit, shutil, tempfile\nscratch = tempfile.mkdtemp(prefix="apparel_nav_")\natexit.register(shutil.rmtree,scratch,ignore_errors=True)')
        launch=replace(launch,'Path(scratch.name)','Path(scratch)')
    launch=replace(launch,'world=str(store/"worlds/store.sdf")',
        'rfid = Path(share("apparel_rfid"))\n    world=str(rfid/"worlds/store.sdf")')
    launch=replace(launch,'urdf=robot/"urdf/warehouse_amr.urdf"','urdf=rfid/"urdf/rfid.urdf"')
    launch=replace(launch,'        navlaunch,','''        Node(package="ros_gz_bridge",executable="parameter_bridge",name="rfid_pose_bridge",
             arguments=["/rfid/model_pose@nav_msgs/msg/Odometry[gz.msgs.Odometry"],parameters=[{"use_sim_time":True}]),
        Node(package="apparel_rfid",executable="sensor.py",parameters=[{"use_sim_time":True}],output="screen"),
        navlaunch,''')
    put(out,'launch/store.launch.py',launch)
    robot=ET.parse(amr/'urdf/warehouse_amr.urdf').getroot()
    for name,x,y,yaw in [('front',.33,0.,0.),('left',0.,.28,math.pi/2),
                         ('rear',-.33,0.,math.pi),('right',0.,-.28,-math.pi/2)]:
        link=ET.SubElement(robot,'link',name=f'rfid_{name}_link')
        visual=ET.SubElement(link,'visual')
        ET.SubElement(ET.SubElement(visual,'geometry'),'box',size='.024 .14 .095')
        ET.SubElement(ET.SubElement(visual,'material',name=f'panel_{name}'),'color',rgba='.05 .38 .48 1')
        joint=ET.SubElement(robot,'joint',name=f'rfid_{name}_mount',type='fixed')
        ET.SubElement(joint,'parent',link='base_footprint')
        ET.SubElement(joint,'child',link=link.get('name'))
        ET.SubElement(joint,'origin',xyz=f'{x} {y} .50',rpy=f'0 0 {yaw}')
    plugin=ET.SubElement(ET.SubElement(robot,'gazebo'),'plugin',
        filename='gz-sim-odometry-publisher-system',name='gz::sim::systems::OdometryPublisher')
    for k,v in [('dimensions','2'),('odom_topic','/rfid/model_pose'),('odom_frame','world'),
                ('robot_base_frame','rfid_model'),('tf_topic','/rfid/sim_tf'),('odom_publish_frequency','5')]:
        ET.SubElement(plugin,k).text=v
    write_xml(robot,out/'urdf/rfid.urdf')
    sdf=ET.parse(store/'worlds/store.sdf').getroot()
    colors=['Navy','Coral','Olive','Plum','Teal','Gold']
    tags=[]
    stops=[]
    prices=[1299,2999,3499,2499,5499,1999]
    for rack,style in enumerate(['Tee','Jeans','Dress','Hoodie','Jacket','Skirt']):
        model=sdf.find(f".//model[@name='{style}_rack']")
        rx,ry,_,roll,pitch,yaw=map(float,model.findtext('pose').split())
        if any(abs(v)>1e-6 for v in (roll,pitch,yaw)): raise ValueError('Expected axis-aligned apparel racks')
        link=model.find('link')
        hangers=[v for v in link.findall('visual') if re.fullmatch(r'hanger[0-9]+',v.get('name',''))]
        if len(hangers)!=12: raise ValueError(f'Expected 12 garment hangers at {style}')
        for hanger in hangers:
            i=int(hanger.get('name')[6:])
            x,y,*_=map(float,hanger.findtext('pose').split())
            epc=f'E200341201234567{len(tags)+1:08X}'
            color=colors[(i+rack)%6]
            size=['S','M','L','XL'][i%4]
            tags.append(dict(epc=epc,rack=style,style=style,sku=f'{style}-{color}-{size}',colour=color,
                             size=size,cents=prices[rack],x=rx+x,y=ry+y+.14,z=1.38))
            vis=ET.SubElement(link,'visual',name='rfid_'+epc)
            ET.SubElement(vis,'pose').text=f'{x} {y+.14} 1.38 0 0 0'
            ET.SubElement(ET.SubElement(ET.SubElement(vis,'geometry'),'box'),'size').text='.065 .018 .085'
            mat=ET.SubElement(vis,'material')
            for k in ('ambient','diffuse'): ET.SubElement(mat,k).text='.95 .94 .81 1'
        for side in (-1,1):
            for dx in (-.9,.9):
                stops.append(dict(rack=style,x=rx+dx,y=ry+side*1.72,yaw=-side*math.pi/2))
    write_xml(sdf,out/'worlds/store.sdf')
    put(out,'config/garments.json',json.dumps(tags,indent=2))
    put(out,'config/stations.json',json.dumps(stops,indent=2))
    # Reuse the tested-in-project Nav2 action client and TF methods; no second motion controller.
    source=(store/'scripts/inventory.py').read_text()
    cls=next(n for n in ast.parse(source).body if isinstance(n,ast.ClassDef) and n.name=='Inventory')
    methods={m.name:ast.get_source_segment(source,m) for m in cls.body if isinstance(m,ast.FunctionDef)}
    import textwrap
    motion=MOTION
    for method in ('now','odom','pose','at_pad','wait','go'):
        # ast extracts the first line without indentation; all following lines retain it.
        motion+='\n    '+methods[method]+'\n'
    put(out,'scripts/motion.py',motion)
    put(out,'scripts/sensor.py',SENSOR)
    put(out,'scripts/patrol.py',PATROL.replace('from motion import Motion','import sys\nsys.path.insert(0,str(Path(__file__).resolve().parent))\nfrom motion import Motion'))
    put(out,'CMakeLists.txt',"""cmake_minimum_required(VERSION 3.8)
project(apparel_rfid)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch urdf worlds config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/sensor.py scripts/patrol.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/motion.py DESTINATION lib/${PROJECT_NAME})
ament_package()
""")
    manifest=ET.parse(store/'package.xml').getroot()
    manifest.find('name').text='apparel_rfid'
    manifest.find('description').text='Scheduled item-level RFID store patrol'
    for dep in ('apparel_store','std_msgs','python3-pil'):
        ET.SubElement(manifest,'exec_depend').text=dep
    write_xml(manifest,out/'package.xml')

def schedule(src, when):
    if not re.fullmatch(r'(?:[01]\d|2[0-3]):[0-5]\d',when):
        raise ValueError('Use local 24-hour time HH:MM')
    unit=Path.home()/'.config/systemd/user'
    unit.mkdir(parents=True,exist_ok=True)
    import shlex
    runner=Path.home()/'.local/bin/apparel-rfid-run'
    runner.parent.mkdir(parents=True,exist_ok=True)
    runner.write_text('#!/bin/bash\nset -e\nsource /opt/ros/jazzy/setup.bash\nsource '+
        shlex.quote(str(src.parent/'install/setup.bash'))+'\nexec ros2 run apparel_rfid patrol.py --ros-args -p use_sim_time:=true\n')
    runner.chmod(0o755)
    # Match the sourced launch terminal's ROS discovery environment.
    keys=('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','ROS_AUTOMATIC_DISCOVERY_RANGE','ROS_STATIC_PEERS','RMW_IMPLEMENTATION','CYCLONEDDS_URI','FASTRTPS_DEFAULT_PROFILES_FILE')
    envfile=unit/'apparel-rfid.env'
    envfile.write_text(''.join(k+'="'+os.environ[k].replace('\\','\\\\').replace('"','\\"')+'"\n' for k in keys if k in os.environ))
    (unit/'apparel-rfid.service').write_text('[Unit]\nDescription=Apparel RFID patrol\n[Service]\nType=oneshot\nEnvironmentFile=%h/.config/systemd/user/apparel-rfid.env\nExecStart=%h/.local/bin/apparel-rfid-run\nKillSignal=SIGINT\nTimeoutStartSec=infinity\n')
    (unit/'apparel-rfid.timer').write_text('[Unit]\nDescription=Daily apparel store inventory\n[Timer]\nOnCalendar=*-*-* '+when+':00\nAccuracySec=1s\nPersistent=false\n[Install]\nWantedBy=timers.target\n')

def main():
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--src',type=Path,default=Path.home()/'warehouse_ws/src')
    parser.add_argument('--schedule',metavar='HH:MM',help='write or update user timer configuration')
    a=parser.parse_args()
    src=a.src.expanduser().resolve()
    dest=src/'apparel_rfid'
    if not dest.exists():
        with tempfile.TemporaryDirectory(dir=src,prefix='.rfid-build-') as temp:
            root=Path(temp)/'apparel_rfid'
            build(root,src)
            root.rename(dest)
        print('Created',dest)
    elif not a.schedule:
        raise SystemExit(f'{dest} already exists; keeping your edits.')
    if a.schedule:
        schedule(src,a.schedule)
        print('Timer written. Run: systemctl --user daemon-reload && systemctl --user enable --now apparel-rfid.timer && systemctl --user restart apparel-rfid.timer')

MOTION = r'''import math
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
'''

SENSOR = r'''#!/usr/bin/env python3
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
'''

PATROL = r'''#!/usr/bin/env python3
"""Scheduled dock-to-dock inventory. SQLite commits every unique radio read."""
import csv
from datetime import datetime
import fcntl
import json
import math
import sqlite3
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from PIL import Image
from ament_index_python.packages import get_package_share_directory as share
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from motion import Motion

HOME=Path.home()/'store_inventory'/'rfid'

class Patrol(Motion):
    def __init__(self):
        super().__init__()
        root=Path(share('apparel_rfid'))
        self.tags={t['epc']:t for t in json.loads((root/'config/garments.json').read_text())}
        self.stops=json.loads((root/'config/stations.json').read_text())
        self.db=sqlite3.connect(HOME/'inventory.sqlite3')
        self.db.executescript("""
          CREATE TABLE IF NOT EXISTS runs(id TEXT PRIMARY KEY, completed TEXT);
          CREATE TABLE IF NOT EXISTS reads(run TEXT, epc TEXT, antenna TEXT, metres REAL,
               sim_s REAL, PRIMARY KEY(run,epc));
        """)
        self.run_id=None
        self.seen=set()
        self.dock_since=None
        self.last_packet=-math.inf
        self.grid=None
        self.create_subscription(String,'/rfid/reads',self.read_packet,5)
        self.create_subscription(OccupancyGrid,'/survey/map',self.map_cb,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def map_cb(self,msg):
        if msg.info.width and msg.info.height:
            self.grid=msg

    def read_packet(self,msg):
        packet=json.loads(msg.data)
        stamp=packet['time']
        if abs(self.now()-stamp)>1.5:
            return
        if packet['dock']:
            if self.dock_since is None or stamp-self.last_packet>1.5 or stamp<self.last_packet:
                self.dock_since=stamp
        else:
            self.dock_since=None
        self.last_packet=stamp
        if self.run_id is None: return
        fresh=[(self.run_id,e,a,d,stamp) for e,a,d in packet['reads'] if e in self.tags and e not in self.seen]
        if fresh:
            with self.db:
                self.db.executemany('INSERT OR IGNORE INTO reads VALUES (?,?,?,?,?)',fresh)
            self.seen.update(row[1] for row in fresh)
            self.get_logger().info(f"RFID {len(self.seen)}/{len(self.tags)}; last read {fresh[-1][1]}")

    def on_pad(self):
        return (self.dock_since is not None and self.last_packet-self.dock_since>=.7
                and abs(self.now()-self.last_packet)<1.5 and self.at_pad())

    def export(self):
        if not self.run_id: return
        folder=HOME/self.run_id
        folder.mkdir(exist_ok=True)
        dst=folder/'inventory.csv'
        tmp=folder/'inventory.csv.tmp'
        with tmp.open('w',newline='') as stream:
            w=csv.writer(stream)
            w.writerow(['epc','sku','garment','size','colour','price_eur','rack','antenna','range_m','sim_s'])
            for epc,a,d,t in self.db.execute('SELECT epc,antenna,metres,sim_s FROM reads WHERE run=? ORDER BY sim_s,epc',(self.run_id,)):
                tag=self.tags[epc]
                w.writerow([epc,tag['sku'],tag['style'],tag['size'],tag['colour'],
                            format(tag['cents']/100,'.2f'),tag['rack'],a,d,t])
        tmp.replace(dst)
        (folder/'missing.txt').write_text('\n'.join(sorted(self.tags.keys()-self.seen)))

    def save_map(self):
        msg=self.grid
        p=msg.info.origin
        q=p.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        grid=np.asarray(msg.data).reshape(msg.info.height,msg.info.width)[::-1]
        img=np.where(grid<0,205,np.where(grid>=65,0,np.where(grid<=25,254,205))).astype('uint8')
        folder=HOME/self.run_id
        Image.fromarray(img).save(folder/'survey.pgm')
        (folder/'survey.yaml').write_text(yaml.safe_dump(dict(image='survey.pgm',mode='trinary',
            resolution=float(msg.info.resolution),origin=[p.position.x,p.position.y,yaw],
            negate=0,occupied_thresh=.65,free_thresh=.25)))

    def run(self):
        old=self.db.execute('SELECT id FROM runs WHERE completed IS NULL ORDER BY id LIMIT 1').fetchone()
        self.get_logger().info('Waiting for navigation, localization, live RFID and SLAM map')
        ready=lambda:(self.nav.server_is_ready() and self.pose() is not None and self.grid is not None
                      and abs(self.now()-self.last_packet)<1.5 and (old or self.on_pad()))
        if not self.wait(ready,240):
            raise RuntimeError('Startup timeout. Check /rfid/model_pose, /rfid/reads, /survey/map, and home-pad pose.')
        self.run_id=old[0] if old else datetime.now().strftime('%Y%m%d-%H%M%S-%f')
        with self.db:
            self.db.execute('INSERT OR IGNORE INTO runs VALUES (?,NULL)',(self.run_id,))
        self.seen={r[0] for r in self.db.execute('SELECT epc FROM reads WHERE run=?',(self.run_id,))}
        self.get_logger().info(f'Run {self.run_id}, starting with {len(self.seen)} verified EPCs')
        self.export()
        round_no=0
        while self.tags.keys()-self.seen:
            round_no+=1
            stops=list(self.stops)
            if round_no%2==0: stops.reverse()
            for stop in stops:
                needed=self.tags.keys()-self.seen
                if not any(self.tags[e]['rack']==stop['rack'] for e in needed): continue
                # Retry from both sides; progressively alternate the viewing offset.
                x=stop['x']+(.20 if round_no%2==0 else 0.)
                self.get_logger().info(f"Pass {round_no}: approach {stop['rack']} at ({x:.2f},{stop['y']:.2f})")
                if self.go(x,stop['y'],stop['yaw']):
                    end=self.now()+1.0
                    self.wait(lambda:self.now()>=end or not self.tags.keys()-self.seen,20)
                else:
                    self.get_logger().warn('Goal unavailable; trying another approach')
                self.export()
            if self.tags.keys()-self.seen:
                self.get_logger().warn(f'{len(self.tags.keys()-self.seen)} EPCs pending. Will retry after 10 seconds.')
                self.wait(lambda:not self.tags.keys()-self.seen,10)
        while not self.on_pad():
            self.get_logger().info('All garments verified. Returning to the original charging pad.')
            reached=self.go(0.,0.,0.)
            if not self.wait(self.on_pad,10 if reached else 15):
                self.get_logger().warn('Dock unconfirmed. Retrying the dock approach.')
        self.export()
        self.save_map()
        (HOME/self.run_id/'summary.json').write_text(json.dumps({
            'verified_items':len(self.seen),'expected_items':len(self.tags),
            'stationary_on_original_pad':True,'run':self.run_id},indent=2))
        with self.db:
            self.db.execute('UPDATE runs SET completed=? WHERE id=?',(datetime.now().isoformat(),self.run_id))
        self.get_logger().info(f'COMPLETE: {len(self.seen)} unique EPCs, stationary on pad. Reports: {HOME/self.run_id}')

def main():
    HOME.mkdir(parents=True,exist_ok=True)
    lock=(HOME/'patrol.lock').open('a')
    try: fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError: raise SystemExit('Another RFID patrol is already running')
    rclpy.init()
    node=Patrol()
    try: node.run()
    except KeyboardInterrupt: pass
    finally:
        try:
            if node.active and node.active.accepted and rclpy.ok():
                f=node.active.cancel_goal_async()
                node.wait(f.done,5)
            node.export()
        finally:
            node.db.close()
            node.destroy_node()
            if rclpy.ok(): rclpy.shutdown()
            lock.close()

if __name__=='__main__': main()
'''

if __name__=='__main__': main()
