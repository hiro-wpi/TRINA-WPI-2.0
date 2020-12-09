#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void test_moveit(moveit::planning_interface::MoveGroupInterface& group) {
  moveit::core::RobotStatePtr current_state = group.getCurrentState();
  std::vector<double> joint_group_positions;
  const moveit::core::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup("right_arm");
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // Move one joint
  int joint_to_move = 0;
  float move_by = 0.3;
  joint_group_positions[joint_to_move] = joint_group_positions[joint_to_move] - move_by;  // radians
  group.setJointValueTarget(joint_group_positions);
  bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("move", "Planning to joint space goal %s", success ? "" : "FAILED");
  if (success) {
    success = (group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }
  ROS_INFO_NAMED("move", "Executing plan to joint space goal... %s", success ? "" : "FAILED");
  
 

  
  // Move to pose 
  geometry_msgs::PoseStamped curr_pose = group.getCurrentPose();
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = curr_pose.pose.position.x;
  target_pose1.position.y = curr_pose.pose.position.y;
  target_pose1.position.z = curr_pose.pose.position.z + 0.5;
  group.setPoseTarget(target_pose1);

  success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("move", "Planning to pose goal %s", success ? "" : "FAILED");
  if (success) {
    success = (group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trina_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();


  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("right_arm"); 
  group.setPlanningTime(45.0);
  ros::WallDuration(1.0).sleep();

  test_moveit(group);

  ros::waitForShutdown();
  return 0;
}