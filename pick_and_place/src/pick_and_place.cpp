/* This code began life as a the MoveIt Pick and Place tutorial with the Panda robot arm:
 * http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/pick_place/pick_place_tutorial.html
 */

// ROS
#include <ros/ros.h>
#include <ros/console.h>
// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "pick_and_place/GraspSrv.h"

static const std::string PLANNING_GROUP = "left_arm";
bool use_grasp_generator = true; // The demo file can also be run without the grasp generator node

// Creates the JointTrajectory message corresponding to an open gripper for encoding in a Grasp
void openGripper(trajectory_msgs::JointTrajectory &posture)
{

  // Add all joints of right gripper move group.

  posture.joint_names.resize(6);
  posture.joint_names[0] = "trina2_1/left_arm_finger_joint";
  posture.joint_names[1] = "trina2_1/left_arm_left_inner_finger_joint";
  posture.joint_names[2] = "trina2_1/left_arm_left_inner_knuckle_joint";
  posture.joint_names[3] = "trina2_1/left_arm_right_inner_knuckle_joint";
  posture.joint_names[4] = "trina2_1/left_arm_right_outer_knuckle_joint";
  posture.joint_names[5] = "trina2_1/left_arm_right_inner_finger_joint";

  // Set them as open.
  posture.points.resize(1);
  posture.points[0].positions.resize(6);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].positions[2] = 0.00;
  posture.points[0].positions[3] = 0.00;
  posture.points[0].positions[4] = 0.00;
  posture.points[0].positions[5] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
}
// Creates the JointTrajectory message corresponding to an closed gripper for encoding in a Grasp
void closedGripper(trajectory_msgs::JointTrajectory &posture)
{
  // Add all joints of the left gripper move group 
  posture.joint_names.resize(6);
  posture.joint_names[0] = "trina2_1/left_arm_finger_joint";
  posture.joint_names[1] = "trina2_1/left_arm_left_inner_finger_joint";
  posture.joint_names[2] = "trina2_1/left_arm_left_inner_knuckle_joint";
  posture.joint_names[3] = "trina2_1/left_arm_right_inner_knuckle_joint";
  posture.joint_names[4] = "trina2_1/left_arm_right_outer_knuckle_joint";
  posture.joint_names[5] = "trina2_1/left_arm_right_inner_finger_joint";
  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(6);
  posture.points[0].positions[0] = 0.8;
  posture.points[0].positions[1] = 0.8;
  posture.points[0].positions[2] = 0.00;
  posture.points[0].positions[3] = 0.00;
  posture.points[0].positions[4] = 0.00;
  posture.points[0].positions[5] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

// Currently this routine creates one grasp to try.  We will need to adapt this for the optimal grasp of a
// cube to use in stacking/ manipulation.  We may create additional grasps later.
void pick(moveit::planning_interface::MoveGroupInterface &move_group, ros::NodeHandle *nh)
{

  // Create a vector of grasps to be attempted.
  // Multiple grasps can be passed to the move_group pick routine for evaluation.  This can be used with 
  // a grasp generator to generate and test multiple grasps.  Currently only one grasp is passed in.
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  if (use_grasp_generator)  // Get the recommended grasp via a call to the grasp generator service using the default demo object
  {
    ros::ServiceClient grasp_client = nh->serviceClient<pick_and_place::GraspSrv>("/trina2_1/trina_collab_grasp");
    pick_and_place::GraspSrv grasp_srv;
    grasp_srv.request.object = "demo_object";
    if(grasp_client.call(grasp_srv)) {
      grasps[0] = grasp_srv.response.grasp;
      std::cout << "Grasp returned: " << grasps[0] << std::endl;
    }
    else {
      ROS_ERROR("Failed to get grasp");
      return;
    }
  }
  else  // Get the default grasp for the auto-generated object
  {
    // This sets the grasping pose based on an offset between the object and the end effector link that is used by the move_group 
    grasps[0].grasp_pose.header.frame_id = "trina2_1/base_link";
    tf2::Quaternion orientation;
    orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = 0.415;
    grasps[0].grasp_pose.pose.position.y = 0;
    grasps[0].grasp_pose.pose.position.z = 0.5;

    // Pre-grasp approach is set based on the robot's base frame
    grasps[0].pre_grasp_approach.direction.header.frame_id = "trina2_1/base_link";
    /* Direction is set as positive x axis */
    grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    // Setting post-grasp retreat
    grasps[0].post_grasp_retreat.direction.header.frame_id = "trina2_1/base_link";
    /* Direction is set as positive z axis */
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;
  }
  // Setting posture of eef before grasp
  openGripper(grasps[0].pre_grasp_posture);

  // Setting posture of eef during grasp
  // +++++++++++++++++++++++++++++++++++
  closedGripper(grasps[0].grasp_posture);

  // Set support surface as table1.  This allows collisions between the object being grasped and "table1".  
  move_group.setSupportSurfaceName("table1");
  // Call pick to pick up the object using the grasps given
  move_group.pick("object", grasps);
}


void place(moveit::planning_interface::MoveGroupInterface &group)
{
  // Again, tries a single placement location.  It is possible to pass a vector of positions to increase the likelihood of finding at least one 
// valid location to put the object down.
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Placement pose represents the desired location of the object center
  place_location[0].place_pose.header.frame_id = "trina2_1/base_link";
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, M_PI / 2);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  place_location[0].place_pose.pose.position.x = 0;
  place_location[0].place_pose.pose.position.y = 0.5;
  place_location[0].place_pose.pose.position.z = 0.5;

  // Set approach to placement
  place_location[0].pre_place_approach.direction.header.frame_id = "trina2_1/base_link";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  // Set post-grasp retreat
  place_location[0].post_place_retreat.direction.header.frame_id = "trina2_1/base_link";
  /* Direction is set as negative y axis */
  place_location[0].post_place_retreat.direction.vector.y = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  // Set posture of eef after placing object
  openGripper(place_location[0].post_place_posture);

  // Set support surface as table2.
  group.setSupportSurfaceName("table2");
  // Call place to place the object using the place locations given.
  group.place("object", place_location);

}

// This routine (directly imported from the MoveIt tutorials, with a few modifications to use TRINA's frame)
// creates collision objects for the robot to interact with in RViz.  This is much simpler than creating "physical" 
// objects in Gazebo and works well for early testing and demonstrations.
void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(3);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "trina2_1/base_link";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.2;
  collision_objects[0].primitives[0].dimensions[1] = 0.4;
  collision_objects[0].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.2;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;

  // BEGIN_SUB_TUTORIAL table2
  // Add the second table where we will be placing the cube.
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "trina2_1/base_link";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.4;
  collision_objects[1].primitives[0].dimensions[1] = 0.2;
  collision_objects[1].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the table. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0;
  collision_objects[1].primitive_poses[0].position.y = 0.5;
  collision_objects[1].primitive_poses[0].position.z = 0.2;
  // END_SUB_TUTORIAL

  collision_objects[1].operation = collision_objects[1].ADD;

  // BEGIN_SUB_TUTORIAL object
  // Define the object that we will be manipulating
  collision_objects[2].header.frame_id = "trina2_1/base_link";
  collision_objects[2].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.02;
  collision_objects[2].primitives[0].dimensions[1] = 0.02;
  collision_objects[2].primitives[0].dimensions[2] = 0.2;

  /* Define the pose of the object. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.5;
  collision_objects[2].primitive_poses[0].position.y = 0;
  collision_objects[2].primitive_poses[0].position.z = 0.5;
  // END_SUB_TUTORIAL

  collision_objects[2].operation = collision_objects[2].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

// Demo function to try moving the arm manually
void test_moveit(moveit::planning_interface::MoveGroupInterface &group)
{
  moveit::core::RobotStatePtr current_state = group.getCurrentState();
  std::vector<double> joint_group_positions;
  const moveit::core::JointModelGroup *joint_model_group = group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  joint_group_positions[0] = joint_group_positions[0] - 0.3; // radians
  group.setJointValueTarget(joint_group_positions);
  bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
  if (success)
  {
    success = (group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }
  ROS_INFO_NAMED("tutorial", "Executing plan (joint space goal) %s", success ? "" : "FAILED");
  std::vector<std::string> remembered = group.getNamedTargets();
  std::cout << remembered.front();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trina_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface arm_group(PLANNING_GROUP); 
  arm_group.setPlanningTime(45.0);

  // Diagnostics 
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::WallDuration(1.0).sleep();

  addCollisionObjects(planning_scene_interface);

  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();

  pick(arm_group, &nh);

  ros::WallDuration(1.0).sleep();

  place(arm_group);

  ros::waitForShutdown();
  return 0;
}