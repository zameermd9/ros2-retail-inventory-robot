#!/usr/bin/env python3
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
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
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
