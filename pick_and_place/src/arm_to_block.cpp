#include <ros/ros.h>
#include <ros/console.h>
// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
// Gazebo
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include "std_msgs/Int8.h"

#include "pick_and_place/PickSrv.h"
#include "pick_and_place/PlaceSrv.h"
#include "pick_and_place/RecoverSrv.h"

moveit::planning_interface::PlanningSceneInterface *planning_scene_interface; // Testing 2 methods for global variable access
boost::scoped_ptr<moveit::planning_interface::MoveGroupInterface> group;      // Both seem valid - evaluate later
std::string held_object = "";
bool use_grasp_generator = false;
ros::Publisher state_pub;

int trina_state = 0;
int stack_size = 0;

void state_callback(const std_msgs::Int8::ConstPtr &msg)
{
    trina_state = msg->data;
}

void update_coll_object(std::string obj)
{
    moveit_msgs::CollisionObject object;
    object.id = obj;
    object.header.frame_id = "trina2_1/base_link";

    gazebo_msgs::GetModelState block_state;
    block_state.request.model_name = obj;
    block_state.request.relative_entity_name = "trina2_1/base_link";

    if (ros::service::call("/gazebo/get_model_state", block_state))
        //ROS_INFO("Got model state");
        std::cout << " "; // debug hack
    else
    {
        ROS_WARN("service call to get_model_state failed!");
        return;
    }

    /* Define the pose of the box. */
    object.primitive_poses.resize(1);
    object.primitive_poses[0] = block_state.response.pose;
    object.operation = object.MOVE;
    planning_scene_interface->applyCollisionObject(object);
}
void add_coll_object(std::string obj, bool to_add = true)
{
    moveit_msgs::CollisionObject object;

    // Add the bounding box for obj.

    object.id = obj;
    object.header.frame_id = "trina2_1/base_link";

    /* Define the primitive and its dimensions. Update with dimensions from Gazebo when available*/
    object.primitives.resize(1);
    object.primitives[0].type = object.primitives[0].BOX;
    object.primitives[0].dimensions.resize(3);
    if (obj == "unit_box")
    { //table
        object.primitives[0].dimensions[0] = 2.0;
        object.primitives[0].dimensions[1] = 1.0;
        object.primitives[0].dimensions[2] = 0.5;
    }
    else
    { //block
        object.primitives[0].dimensions[0] = 0.05;
        object.primitives[0].dimensions[1] = 0.05;
        object.primitives[0].dimensions[2] = 0.1;
    }

    gazebo_msgs::GetModelState block_state;
    block_state.request.model_name = obj;
    block_state.request.relative_entity_name = "trina2_1/base_link";

    if (ros::service::call("/gazebo/get_model_state", block_state))
        //ROS_INFO("Got model state");
        std::cout << " "; // debug hack
    else
    {
        ROS_WARN("service call to get_model_state failed!");
        return;
    }

    /* Define the pose of the box. */
    object.primitive_poses.resize(1);
    object.primitive_poses[0] = block_state.response.pose;
    object.operation = object.ADD;
    if (!to_add)
        object.operation = object.REMOVE;
    planning_scene_interface->applyCollisionObject(object);
}

void add_attached_coll_object(std::string obj, bool to_add = true)
{
    moveit_msgs::AttachedCollisionObject att_object;

    // Add the bounding box for obj.

    att_object.link_name = "trina2_1/left_arm_left_outer_finger";
    att_object.object.id = obj;
    att_object.object.header.frame_id = "trina2_1/base_link";

    /* Define the primitive and its dimensions. Update with dimensions from Gazebo when available*/
    /* A default pose */
    gazebo_msgs::GetModelState block_state;
    block_state.request.model_name = obj;
    block_state.request.relative_entity_name = "trina2_1/base_link";
    if (ros::service::call("/gazebo/get_model_state", block_state))
        //ROS_INFO("Got model state");
        std::cout << " "; // debug hack
    else
    {
        ROS_WARN("service call to get_model_state failed!");
        return;
    }

    /* Define a box to be attached */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.05;
    primitive.dimensions[1] = 0.05;
    primitive.dimensions[2] = 0.1;

    att_object.object.primitives.push_back(primitive);
    att_object.object.primitive_poses.push_back(block_state.response.pose);

    att_object.touch_links = std::vector<std::string>{
        "trina2_1/left_arm_left_inner_finger",
        "trina2_1/left_arm_left_inner_finger_pad",
        "trina2_1/left_arm_left_inner_knuckle",
        "trina2_1/left_arm_left_outer_finger",
        "trina2_1/left_arm_left_outer_knuckle",
        "trina2_1/left_arm_right_outer_knuckle",
        "trina2_1/left_arm_right_inner_finger_pad",
        "trina2_1/left_arm_right_inner_finger",
        "trina2_1/left_arm_right_inner_knuckle",
    };
    att_object.object.operation = to_add ? att_object.object.ADD : att_object.object.REMOVE;
    planning_scene_interface->applyAttachedCollisionObject(att_object);
}

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
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory &posture)
{
    // Add all joints of the left gripper move group - make these generic later
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
    posture.points[0].time_from_start = ros::Duration(0.5);
}

moveit::planning_interface::MoveItErrorCode pick_block(std::string pick_obj)
{
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);
    moveit::planning_interface::MoveItErrorCode result = moveit::planning_interface::MoveItErrorCode::FAILURE;
    // Setting grasp pose
    // ++++++++++++++++++++++
    gazebo_msgs::GetModelState block_state;
    block_state.request.relative_entity_name = "trina2_1/base_link";
    block_state.request.model_name = pick_obj;
    if (ros::service::call("/gazebo/get_model_state", block_state))
        ROS_INFO("Done");
    else
    {
        ROS_WARN("service call to get_model_state failed!");
        return result;
    }
    if (!use_grasp_generator)
    {
        geometry_msgs::Pose target_pose1 = block_state.response.pose;
        grasps[0].grasp_pose.header.frame_id = "trina2_1/base_link";
        tf2::Quaternion orientation;
        orientation.setRPY(-M_PI / 2, -M_PI, -M_PI / 2);
        grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
        grasps[0].grasp_pose.pose.position = block_state.response.pose.position;
        grasps[0].grasp_pose.pose.position.x -= 0.14;
        grasps[0].grasp_pose.pose.position.z += 0.045;
        grasps[0].max_contact_force = 20;
        // Setting pre-grasp approach
        // ++++++++++++++++++++++++++
        /* Defined with respect to frame_id */
        grasps[0].pre_grasp_approach.direction.header.frame_id = "trina2_1/base_link";
        /* Direction is set as positive x axis */
        grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
        grasps[0].pre_grasp_approach.min_distance = 0.095;
        grasps[0].pre_grasp_approach.desired_distance = 0.115;

        // Setting post-grasp retreat
        // ++++++++++++++++++++++++++
        /* Defined with respect to frame_id */
        grasps[0].post_grasp_retreat.direction.header.frame_id = "trina2_1/base_link";
        /* Direction is set as positive z axis */
        grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
        grasps[0].post_grasp_retreat.min_distance = 0.1;
        grasps[0].post_grasp_retreat.desired_distance = 0.25;

        // Setting posture of eef before grasp
        // +++++++++++++++++++++++++++++++++++
        openGripper(grasps[0].pre_grasp_posture);

        // Setting posture of eef during grasp
        // +++++++++++++++++++++++++++++++++++
        closedGripper(grasps[0].grasp_posture);
    }
    else
    {
        //grasps = create_grasps()
        std::cout << "If the grasp generator was working, we'd generate grasps now" << std::endl;
    }
    // Set support surface as table1.
    group->setSupportSurfaceName("unit_box");
    // Call pick to pick up the object using the grasps given
    group->setGoalTolerance(0.05);
    group->setStartStateToCurrentState();
    std_msgs::Int8 new_state;
    new_state.data = 2; // PICKING
    state_pub.publish(new_state);
    result = group->pick(pick_obj, grasps);
    held_object = pick_obj;
    return result;
}

void retreat()
{

    geometry_msgs::PoseStamped eef_pose = group->getCurrentPose();

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose target_pose = eef_pose.pose;
    for (int i = 0; i < 4; i++)
    {
        target_pose.position.z += 0.05;
        waypoints.push_back(target_pose);
    }
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.005;
    bool avoid = false; // avoid collisions - may need to make this false if the support surface causes problems
    double fraction = group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, avoid);
    if (fraction > 0)
        group->execute(trajectory);
    else
        std::cout << "retreat failed" << std::endl;
    group->clearPathConstraints();
}

bool trina_pick(pick_and_place::PickSrv::Request &req, pick_and_place::PickSrv::Response &res)
{
    if (trina_state != 0)
    {
        std::cout << "Not in WAITING state, can't execute Pick service" << std::endl;
        return false;
    }
    std_msgs::Int8 new_state;
    new_state.data = 1; // COMMANDED
    state_pub.publish(new_state);
    std::string pick_obj = req.pick_obj;
    ROS_INFO("Pick requested");
    // To update location of pick_obj, in case needed
    for (int i = 0; i < 3; ++i)
        add_coll_object("unit_box_" + std::to_string(i), true);

    res.success = (pick_block(pick_obj) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!res.success)
    {
        new_state.data = 4; // FAILED
        //add_coll_object(pick_obj, false);
        //add_attached_coll_object(pick_obj, true);
        //retreat();
    }
    else
    {
        new_state.data = 0; // WAITING
    }
    state_pub.publish(new_state);
    //ros::Duration(1).sleep(); // Make sure there is time for the state to update before we try again
    return res.success;
}

// service for recovery (pick failure) - if "continue == true", attach object and retreat; if not, open gripper and plan to home

moveit::planning_interface::MoveItErrorCode place_block(double x, double y)
{
    // Create a vector of placings to be attempted, currently only creating single place location.
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(3);
    moveit::planning_interface::MoveItErrorCode result = moveit::planning_interface::MoveItErrorCode::FAILURE;
    // Setting place location pose
    // +++++++++++++++++++++++++++
    place_location[0].place_pose.header.frame_id = "trina2_1/base_link";
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, M_PI / 2);
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

    // Desired location of the center of the object
    place_location[0].place_pose.pose.position.x = x;
    place_location[0].place_pose.pose.position.y = y;
    place_location[0].place_pose.pose.position.z = 0.6; // Not sure why this much separation from the table is needed - work to go
    if (stack_size > 1)
    {
        place_location[0].place_pose.pose.position.z += 0.05 * (stack_size - 1);
    }

    // Setting pre-place approach
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    place_location[0].pre_place_approach.direction.header.frame_id = "trina2_1/base_link";
    /* Direction is set as negative z axis */
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.115;

    // Setting post-grasp retreat
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    place_location[0].post_place_retreat.direction.header.frame_id = "trina2_1/base_link";
    /* Direction is set as negative y axis */
    place_location[0].post_place_retreat.direction.vector.x = -1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;

    // Setting posture of eef after placing object
    // +++++++++++++++++++++++++++++++++++++++++++
    /* Similar to the pick case */
    openGripper(place_location[0].post_place_posture);

    place_location[1] = place_location[0];
    orientation.setRPY(M_PI / 2, 0, M_PI / 2);
    place_location[1].place_pose.pose.orientation = tf2::toMsg(orientation);
    openGripper(place_location[1].post_place_posture);

    place_location[2] = place_location[0];
    orientation.setRPY(0, M_PI / 2, M_PI / 2);
    place_location[2].place_pose.pose.orientation = tf2::toMsg(orientation);
    openGripper(place_location[2].post_place_posture);

    group->setSupportSurfaceName("unit_box"); // Future work - support surface and associated xyz parameters should be set by button click

    // Call place to place the object using the place locations given.
    group->setGoalTolerance(0.5);
    group->setStartStateToCurrentState();
    group->setNumPlanningAttempts(10);
    result = group->place(held_object, place_location);
    return result;
}
bool trina_place(pick_and_place::PlaceSrv::Request &req, pick_and_place::PlaceSrv::Response &res)
{
    ROS_INFO("Place requested");
    // Update block locations in planning scene
    for (int i = 0; i < 3; ++i)
        update_coll_object("unit_box_" + std::to_string(i));
    res.success = (place_block(req.x, req.y) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    stack_size++;
    return res.success;
}
bool trina_recover(pick_and_place::RecoverSrv::Request &req, pick_and_place::RecoverSrv::Response &res)
{
    if (trina_state != 4)
    {
        std::cout << "Not in state FAILED, cannot execute recovery" << std::endl;
        res.success = false;
        return res.success;
    }
    bool cont = req.cont; // continue?
    ROS_INFO("Recovery requested");
    // To update location of pick_obj, in case needed
    for (int i = 0; i < 3; ++i)
        add_coll_object("unit_box_" + std::to_string(i), true);

    // assuming here that we've had the persistent gripper trajectory issue, hard-coding the object attachment
    // Future: check to be sure there is a block in the gripper.
    if (cont)
    {
        add_coll_object(held_object, false);
        add_attached_coll_object(held_object, true);
        retreat();
    }
    else
    {
        // Open the gripper - using MoveIt because we may later want to change this to a different trajectory plan
        moveit::planning_interface::MoveGroupInterface grip_group("left_gripper");
        moveit::core::RobotStatePtr current_state = grip_group.getCurrentState();
        std::vector<double> joint_group_positions;
        const moveit::core::JointModelGroup *joint_model_group = grip_group.getCurrentState()->getJointModelGroup("left_gripper");
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        joint_group_positions[0] = 0;
        grip_group.setJointValueTarget(joint_group_positions);
        // get rid of collision object, to eliminate collision issues
        add_coll_object(held_object, false);
        bool success = (grip_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO("Opening gripper %s", success ? "" : "FAILED");
        if (success)
        {
            success = (grip_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            retreat();
        }
        add_coll_object(held_object, true);
    }
    std_msgs::Int8 new_state;
    new_state.data = 0; // WAITING
    state_pub.publish(new_state);
    res.success = true;
    return res.success;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_to_block");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::WallDuration(1.0).sleep();
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    group.reset(new moveit::planning_interface::MoveGroupInterface("left_arm"));
    group->setPlanningTime(45.0);
    planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    state_pub = nh.advertise<std_msgs::Int8>("buttons", 1);
    // Make sure we have a clean planning scene before starting the node - for some reason, we were never able
    // to get the planning scene cleared, which would have better allowed for test resets after failures without
    // having to reset the simulation.  Code is retained for future troubleshooting, when possible.

    // auto names = planning_scene_interface->getKnownObjectNames();
    // for (auto name : names) {
    //     std::cout << "known object " << name << std::endl;
    //     //planning_scene_interface->removeCollisionObject(name);
    // }
    // //planning_scene_interface->removeCollisionObjects(names);
    // names = planning_scene_interface->getKnownObjectNames();
    // std::cout << "Known objects after removal: " << std::endl;
    // for (auto name : names)
    //     std::cout << "known object " << name << std::endl;

    // Alerts users if unexpected objects exist in the planning scene on node initiation.
    auto names = planning_scene_interface->getKnownObjectNames();
    for (auto name : names)
    {
        std::cout << "known object " << name << std::endl;
        add_coll_object(name, false);
    }
    // Initialize planning scene
    add_coll_object("unit_box", true);
    for (int i = 0; i < 3; ++i)
        add_coll_object("unit_box_" + std::to_string(i), true);

    ros::Subscriber state_sub = nh.subscribe("trina_state", 1000, state_callback);

    ros::ServiceServer pick_service = nh.advertiseService("trina_pick", trina_pick);
    ros::ServiceServer place_service = nh.advertiseService("trina_place", trina_place);
    ros::ServiceServer recover_service = nh.advertiseService("trina_recover", trina_recover);
    ros::waitForShutdown();
    return 0;
}