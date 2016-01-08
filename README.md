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

then 

rosrun farm_aid farm_aid.py

##Some Useful terminal commands : 

To know the position :  rostopic echo /amcl_pose

To move to the map zero position : 

rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \
'{ header: { frame_id: "map" }, pose: { position: { x: 0.0, y: 0, z:
0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } }'
