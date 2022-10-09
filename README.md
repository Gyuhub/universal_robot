# Universal Robot
## Position Control of the UR5e

### ur_example_controller
# Dependencies
- universal_robot
- kdl (kinematics and dynamics library)
- controller_interface
- controller_manager
- pluginlib
- realtime_tools
- roscpp
- rospy
- std_msgs
- geometry_msgs
- sensor_msgs
- trajectory_msgs
- std_srvs
- kdl_parser
- tf2_kdl

**Please** don't worry about that there are so many dependencies.
You can download with *rosdep*!

# Installation
```
cd ~/$(your_workspace)
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

# Example
```
roslaunch ur_e_gazebo ur5e.launch 
```