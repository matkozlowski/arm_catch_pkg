To launch gazebo:  
$ roslaunch ur_e_gazebo ur5e_joint_limited.launch  

Start MoveIt Node:  
$ roslaunch ur5_e_moveit_config ur5_e_moveit_planning_execution.launch sim:=true limited:=true  
Should say "You can start planning now!" in the terminal  

Start Rviz (Optional):  
$ roslaunch ur5_e_moveit_config moveit_rviz.launch config:=true  

Publish transfrom between base of arm and Kinect:  
$ rosrun tf static_transform_publisher 0 0.25 0.05 0 0 -1.57 base_link camera_link 50  

Start node that listens to topic to move arm:  
$ rosrun arm_catch_pkg arm_subscriber  

Publish to topic to get arm to move into a desired position (The specific numbers can be changed):  
rostopic pub /arm_goapose geometry_msgs/Pose "position:  
  x: 0.0  
  y: 0.4  
  z: 0.25  
orientation:  
  x: 0.0  
  y: 0.0  
  z: 0.0  
  w: 1.0"  

Start node that detects ball:  
$ rosrun arm_catch_pkg ball_detection_node  

Start node that uses ball detection data to predict ball trajectory and move arm:  
$ rosrun arm_catch_pkg vision_trajectory  
Make sure that the ball is not being detected by the camera before running this node  

Launch Ball (Arguments are: x y z position, x y z velocity):  
$ rosrun arm_catch_pkg ball_launch 0 0 0 0 0 0  
