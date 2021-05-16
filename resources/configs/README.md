**ur5e.launch**  
Replace the file in `src/fmauch_universal_robot/ur_e_gazebo/launch` with this one  
The modified file creates the gazebo world correctly with the arm, table, and kinect
<<<<<<< HEAD
<<<<<<< HEAD



Gripper Stuff:

1) Connecting the Robotiq 2F-140 gripper to the arm and rendering in Gazebo

	- Clone the Robotiq industrial repository into your workspace, then build packages (https://github.com/ros-industrial/robotiq)
	- Replace the file found at `src/fmauch_universal_robot/ur_e_description/urdf/ur5e_joint_limited_robot.urdf.xacro` with the one in this directory (this connects the gripper to the arm)
	- Clone this repo into your workspace: (https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins) and build with catkin
	- Replace the file found at `src/robotiq/robotiq_2f_140_gripper_visualization/urdf/robotiq_arg2f_transmission.xacro` with the one in this directory 

2) Controlling the gripper via Action Client

	- Add the `gripper_controller_robotiq.yaml` found in this directory to the `src/fmauch_universal_robot/ur_e_gazebo/controller` directory
	- Control gripper via Action Client
	- TODO: Will likely need this plugin to work around funky gazebo physics when grasping objects (https://github.com/JenniferBuehler/gazebo-pkgs/tree/master/gazebo_grasp_plugin)
=======
>>>>>>> parent of 769ae22... update README and configuration files
=======
>>>>>>> parent of 769ae22... update README and configuration files
