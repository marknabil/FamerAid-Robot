# FamerAid-Robot

ROS robotics project developed on ROS HYDRO in the robotics platform in Universite De Bourgogne in France as a deliverable.


##Hardware used :

Turtlebot2 Kobuki base
Kinect
RPlidar


##Dependencies : 

https://github.com/roboticslab-fr/rplidar-turtlebot2

rbx1 used in the book ROS By Example Hydro volume 1

https://github.com/pirobot/rbx1

rbx2 used in the book ROS By Example Hydro volume 2

https://github.com/pirobot/rbx2

##for testing the code : 
###first build the package  on the turtlebot netbook:

launch the project main launch file by the following command

roslaunch farm_aid project.launch
### Instead of the previous one launch file , use the following split launch files 
####in the turtlebot laptop:
roslaunch turtlebot_le2i remap_rplidar_minimal.launch

roslaunch rbx1_nav tb_demo_amcl.launch map:=my_map3_edited.yaml

roslaunch rbx2_vision openni_node.launch publish_tf:=false

OR TRY THIS INSTEAD OF THE OPENNI NODE 

roslaunch turtlebot_bringup 3dsensor.launch

roslaunch rbx2_ar_tags ar_large_markers_kinect.launch

####in the workstation : 
roslaunch turtlebot_rviz_launchers view_navigation.launch

roslaunch turtlebot_rviz_launchers view_navigation_marker.launch

roslaunch turtlebot_teleop keyboard_teleop.launch



then 

rosrun farm_aid farm_aid.py



##Some Useful terminal commands : 

To know the position :  rostopic echo /amcl_pose

To move to the map zero position : 

rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \
'{ header: { frame_id: "map" }, pose: { position: { x: 0.0, y: 0, z:
0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } }'
