/* This code borrows from https://github.com/zengzhen/zhen_baxter_moveit/blob/master/src/robot_block_simulator.cpp
*/


#include <ros/ros.h>
#include <gazebo_msgs/GetLinkState.h>
#include <tf/transform_broadcaster.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/collision_detection/collision_matrix.h>