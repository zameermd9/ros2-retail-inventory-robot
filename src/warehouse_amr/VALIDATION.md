# Validation status

This package was generated and statically checked in an environment without
ROS 2, Gazebo or RViz executables. No live Gazebo launch, physical-motion test,
RViz visual test, `check_urdf`, or libsdformat conversion has been executed here.
Do not interpret the static checks as a runtime certification.

Static checks performed are recorded below after the validation run. The
package includes a runtime verifier for use on the user's Ubuntu 24.04 VM.

## Required live acceptance checks

- Build with colcon; run check_urdf and gz sdf -k.
- Start sim.launch.py; robot spawns without plugin or rendering errors.
- verify_runtime.py passes all sensor, odometry, clock, wheel and TF checks.
- RViz visibly displays the robot, scans and live image.
- Robot moves forward and turns; wheels rotate; scan and image change.
- Stopping the command publisher causes the robot to stop.

Remaining project work: autonomous navigation, tag detection, missions,
return-to-dock, docking controller and simulated battery/charging confirmation.

## Static validation completed

- Parsed URDF, world SDF and package XML successfully.
- Checked 9 unique links, one rooted acyclic joint tree, and all parent/child references.
- Checked positive masses and positive diagonal inertia tensors satisfying the triangle inequalities.
- Matched wheel radius, separation and ground contact heights to drive configuration.
- Checked sensor frame IDs and camera optical-axis rotation.
- Checked all eight ROS/Gazebo topic mappings and RViz sensor subscriptions.
- Parsed launch and node Python syntax; checked YAML configurations.
- Confirmed world Sensors plugin selects Ogre 2.

These checks do not execute Gazebo plugins or ROS nodes.

Isolated watchdog logic tests passed for speed limits, stale-command stop and non-finite rejection using a fake clock/publisher. This is not a live ROS test.
