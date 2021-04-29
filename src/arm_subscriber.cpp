#include "arm_test_pkg/arm_subscriber.h"


ArmSubscriber::ArmSubscriber(ros::NodeHandle &n) : _PLANNING_GROUP("manipulator"),
	_move_group(_PLANNING_GROUP), _visual_tools("base_link"){

	_joint_model_group = _move_group.getCurrentState()->getJointModelGroup(_PLANNING_GROUP);
	_pose_sub = n.subscribe("arm_goal_pose", 1000, &ArmSubscriber::poseCallback, this);
	_visual_tools.deleteAllMarkers();
}

void ArmSubscriber::poseCallback(const geometry_msgs::Pose &msg){
	ROS_INFO("Received pose");

	geometry_msgs::Pose target_pose;
	target_pose.orientation.w = msg.orientation.w;
	target_pose.orientation.x = msg.orientation.x;
	target_pose.orientation.y = msg.orientation.y;
	target_pose.orientation.z = msg.orientation.z;
	target_pose.position.x = msg.position.x;
	target_pose.position.y = msg.position.y;
	target_pose.position.z = msg.position.z;

	_move_group.setPoseTarget(target_pose);

	moveit::planning_interface::MoveGroupInterface::Plan plan;

	ROS_INFO("Planning starting...");
	bool success = false;
	int tries = 0;
	while(!success && tries < 3){
		success = (_move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		tries++;
	}
	ROS_INFO("Planning: %s", success ? "successful" : "unsuccessful");

	_visual_tools.publishAxisLabeled(target_pose, "target_pose");
	_visual_tools.publishTrajectoryLine(plan.trajectory_, _joint_model_group);

	ROS_INFO("Executing...");
	if(success){
		_move_group.execute(plan);
		ROS_INFO("Plan executed");
	}
	else{
		ROS_INFO("No plan to execute");
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "arm_subscriber");
	ros::NodeHandle node_handle;

	// Spin two threads, necessary or else planner gets stuck in callback
	ros::AsyncSpinner spinner(2);
	spinner.start();

	ArmSubscriber armSub(node_handle);

	ros::waitForShutdown();

	return 0;
}