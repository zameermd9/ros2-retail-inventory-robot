# QR warehouse inspection
Requires the existing warehouse_amr package for Jazzy. This add-on provides a separate Gazebo world with three QR inspection stations, a QR dock sign and a Gazebo contact switch. The mission begins at the default spawn in the main aisle.

Install in Ubuntu:
```bash
sudo apt update
sudo apt install python3-pyzbar python3-numpy libzbar0
unzip ~/Downloads/warehouse_inspection.zip -d ~/warehouse_ws/src
cd ~/warehouse_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select warehouse_inspection
```

Stop the previous simulator, avoidance node and manual velocity publishers. In a fresh terminal:
```bash
source /opt/ros/jazzy/setup.bash
source ~/warehouse_ws/install/setup.bash
ros2 launch warehouse_inspection warehouse.launch.py gazebo_gui:=false
```
Gazebo runs its physics and sensors in the background. RViz shows the robot, laser and camera. To see both windows, omit gazebo_gui:=false. Use the default full camera resolution for QR decoding.

With the robot spawned, start in another terminal on the same ROS domain:
```bash
source /opt/ros/jazzy/setup.bash
source ~/warehouse_ws/install/setup.bash
ros2 run warehouse_inspection patrol.py --ros-args -p use_sim_time:=true
```

The route uses wheel odometry to reach inspection positions in the known clear aisle. It faces the shelf and requires two fresh frames decoding the expected payload before advancing. At the dock it steers using the horizontal QR image error and creeps forward until the dock contact switch triggers. After standstill it emits a simulated charging event and saves the CSV report under ~/warehouse_missions/. Charging is a simulated status event, without an electrical or battery capacity model. Fresh starts require relaunching the inspection world to reset odometry and the dock trigger.

A blocked aisle stops motion. If the obstacle clears within 12 simulated seconds, travel resumes. Persistent blockage or a missing QR ends the run with a logged failure. This version follows a fixed route and has no alternate-aisle planner. The horizontal laser detects obstacles intersecting its plane. Ctrl+C in the patrol terminal stops the mission. The existing watchdog handles command timeout.

If no station scan appears, verify the RViz camera image shows a crisp black-and-white board when the robot faces a station. Use the full camera profile. Check the launch log for unresolved STL resources if signs appear blank. If odometry/scan/clock are missing, use the same sourced ROS environment and domain in both terminals.

Validation performed: parsed the package and world, resolved all four mesh references, decoded images reconstructed from the delivered mesh triangles with ZBar, and exercised the mission state machine using synthetic feedback for success, missing QR, persistent obstruction and missing dock contact. Live rendering, wheel slip, camera decoding and dock contact remain to be tested in a running Gazebo instance.
