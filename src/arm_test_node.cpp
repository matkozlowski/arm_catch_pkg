#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "arm_test_node");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	static const std::string PLANNING_GROUP = "manipulator";

	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	 // Visualization
	// ^^^^^^^^^^^^^
	//
	// The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
	// and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
	namespace rvt = rviz_visual_tools;
	moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
	visual_tools.deleteAllMarkers();

	// Remote control is an introspection tool that allows users to step through a high level script
	// via buttons and keyboard shortcuts in RViz
	visual_tools.loadRemoteControl();

	// RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
	Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
	text_pose.translation().z() = 1.75;
	visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

	// Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
	visual_tools.trigger();

	// Getting Basic Information
	// ^^^^^^^^^^^^^^^^^^^^^^^^^
	//
	// We can print the name of the reference frame for this robot.
	ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

	// We can also print the name of the end-effector link for this group.
	ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

	// We can get a list of all the groups in the robot:
	ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
	std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
				std::ostream_iterator<std::string>(std::cout, ", "));

	// Start the demo
	// ^^^^^^^^^^^^^^^^^^^^^^^^^
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");



	

	Eigen::MatrixXd zAxis(3,1);
	zAxis(0,0) = 0;
	zAxis(1,0) = 0;
	zAxis(2,0) = 1;

	Eigen::Quaterniond rot(1, 0, 0, 0);
	Eigen::AngleAxisd rZ(3.14159 / 2, zAxis);
	Eigen::Quaterniond rotated = rot * rZ;

	// We can plan a motion for this group to a desired pose for the
	// end-effector.
	geometry_msgs::Pose target_pose1;
	target_pose1.orientation.w = rotated.w();
	target_pose1.orientation.x = rotated.x();
	target_pose1.orientation.y = rotated.y();
	target_pose1.orientation.z = rotated.z();
	target_pose1.position.x = 0.0;
	target_pose1.position.y = 0.3;
	target_pose1.position.z = 0.5;
	move_group.setPoseTarget(target_pose1);

	// Now, we call the planner to compute the plan and visualize it.
	// Note that we are just planning, not asking move_group
	// to actually move the robot.
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

	ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");





	// We can also visualize the plan as a line with markers in RViz.
	ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
	visual_tools.publishAxisLabeled(target_pose1, "pose1");
	visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
	visual_tools.trigger();
	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

	move_group.execute(my_plan);

	return 0;
}