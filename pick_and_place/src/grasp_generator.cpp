#include <ros/ros.h>
#include <ros/console.h>

#include <gazebo_msgs/GetModelState.h>
#include "std_msgs/Int8.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "pick_and_place/GraspSrv.h"

enum class Approaches
{
    FORWARD,
    LEFT,
    RIGHT,
    DOWN
};

/* This class defines a grasp generator that works with the TRINA2 simulation developed by the HIRO Lab.  
 * It stores the most recently recommended grasp approach direction, and accepts change requests on topic "grasp_dir".
 * It also provides the "trina_collab_grasp" ROS service, which returns a moveit_msgs::Grasp object that combines 
 * knowledge about the robot, the object to be grasped, and the recommended grasp direction to produce a feasible pick action.
 */
class TrinaGraspGen
{
private:
    Approaches approach_direction; // Stores the recommendation from the user
    ros::Subscriber grasp_sub;
    ros::ServiceServer grasp_srv;

public:
    TrinaGraspGen(ros::NodeHandle *nh)
    {
        approach_direction = Approaches::FORWARD;
        grasp_sub = nh->subscribe("grasp_dir", 1000, &TrinaGraspGen::collab_grasp_callback, this);
        grasp_srv = nh->advertiseService("trina_collab_grasp", &TrinaGraspGen::trina_grasp, this);
    }
    bool trina_grasp(pick_and_place::GraspSrv::Request &req, pick_and_place::GraspSrv::Response &res)
    {
        geometry_msgs::Pose target_pose1;
        // If we are running just the RViz demo, we set the known object position, since there is no physical object to find.
        // Note that for the demo object, only the FORWARD and DOWN grasps are reachable in TRINA's workspace.
        if (req.object == "demo_object")
        {
            target_pose1.position.x = 0.5;
            target_pose1.position.y = 0;
            target_pose1.position.z = 0.5;
        }
        // This code converts the grasp generator to work with Gazebo objects.
        // Gets the actual position of a block.
        else
        {
            gazebo_msgs::GetModelState block_state;
            block_state.request.relative_entity_name = "trina2_1/base_link";
            block_state.request.model_name = req.object;
            if (ros::service::call("/gazebo/get_model_state", block_state))
                ROS_INFO("Done");
            else
            {
                ROS_WARN("service call to get_model_state failed!");
                return false;
            }
            target_pose1 = block_state.response.pose;
        }
        moveit_msgs::Grasp collab_grasp;

        // Set grasp location based on block position and end effector geometry
        collab_grasp.grasp_pose.header.frame_id = "trina2_1/base_link";
        collab_grasp.grasp_pose.pose.position = target_pose1.position;
        collab_grasp.max_contact_force = 20;

        // Set pre-grasp approach common params
        collab_grasp.pre_grasp_approach.direction.header.frame_id = "trina2_1/base_link";
        collab_grasp.pre_grasp_approach.min_distance = 0.095;
        collab_grasp.pre_grasp_approach.desired_distance = 0.115;

        // Set post-grasp retreat common params
        collab_grasp.post_grasp_retreat.direction.header.frame_id = "trina2_1/base_link";
        /* Direction is set as positive z axis */
        collab_grasp.post_grasp_retreat.direction.vector.z = 1.0;
        collab_grasp.post_grasp_retreat.min_distance = 0.1;
        collab_grasp.post_grasp_retreat.desired_distance = 0.25;

        tf2::Quaternion orientation;
        switch (approach_direction)
        {
        case Approaches::FORWARD:
            // Parameters for positive x-direction approach and grasp
            orientation.setRPY(-M_PI / 2, -M_PI, -M_PI / 2);
            collab_grasp.grasp_pose.pose.orientation = tf2::toMsg(orientation);
            collab_grasp.pre_grasp_approach.direction.vector.x = 1.0;
            collab_grasp.grasp_pose.pose.position.x -= 0.085;
            break;
        case Approaches::LEFT:
            // Parameters for negative y-direction approach and grasp
            orientation.setRPY(-M_PI / 2, -M_PI, -M_PI);
            collab_grasp.grasp_pose.pose.orientation = tf2::toMsg(orientation);
            collab_grasp.pre_grasp_approach.direction.vector.y = -1.0;
            collab_grasp.grasp_pose.pose.position.y += 0.085;
            break;
        case Approaches::RIGHT:
            // Parameters for positive y-direction approach and grasp
            orientation.setRPY(-M_PI / 2, -M_PI, 0);
            collab_grasp.grasp_pose.pose.orientation = tf2::toMsg(orientation);
            collab_grasp.pre_grasp_approach.direction.vector.y = 1.0;
            collab_grasp.grasp_pose.pose.position.y -= 0.085;
            break;
        case Approaches::DOWN:
            // Parameters for negative z-direction approach and grasp
            orientation.setRPY(0, -M_PI, -M_PI / 2);
            collab_grasp.grasp_pose.pose.orientation = tf2::toMsg(orientation);
            collab_grasp.pre_grasp_approach.direction.vector.z = -1.0;
            collab_grasp.grasp_pose.pose.position.z += 0.2;
            break;
        }
        res.grasp = collab_grasp;
        return true;
    }

    // Callback for "grasp_dir" topic.  Given an integer input, do a basic check for validity and set the desired grasping direction.
    void collab_grasp_callback(const std_msgs::Int8::ConstPtr &msg)
    {
        int temp = msg->data;
        if ((temp >= 0) && (temp < 4))
        {
            approach_direction = static_cast<Approaches>(temp);
            ROS_INFO("Recommended approach direction: %i", temp);
        }
        else
        {
            ROS_ERROR("Not a valid approach direction");
        }
    }
};

// Initialize the ROS node, create an instance of the grasp generator, and spin...
int main(int argc, char **argv)
{
    ros::init(argc, argv, "trina_grasp");
    ros::NodeHandle nh;
    TrinaGraspGen trina_1_grasp = TrinaGraspGen(&nh);
    ros::spin();
    return 0;
}