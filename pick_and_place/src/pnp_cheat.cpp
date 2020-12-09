
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <gazebo_msgs/ModelState.h>


#define UPDATE_HZ 5

gazebo_msgs::ModelState block_state;

void gripper_callback(const geometry_msgs::PointStamped::ConstPtr &gripper_loc)
{
    block_state.pose.position = gripper_loc->point;
    block_state.pose.position.z += 5;
}

int main(int argc, char *argv[])
{

    //Setup the block to be attached - first argument should be "unit_box_N"
 
    block_state.model_name = std::string block_name(argv[1]);;
    block_state.reference_frame = "trina2_1/base_link";

    ros::init(argc, argv, "capricorn_odom_vis");
    ros::NodeHandle nh;
    ros::Rate update_rate(UPDATE_HZ);

    //Subscribe to TRINA's joint states to get the end effector pose

    ros::Subscriber joint_states_sub = node_handle.subscribe("/panda/joint_states", 1, jointStatesCallback);

    // Create publisher to send the updated location for the block
    ros::Publisher state_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1, true);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    while(ros::ok())
    {
        bool haveTransform = false;
        while(!haveTransform)
        {
            try
            {
                odom_to_map = tfBuffer.lookupTransform("map", robot_name + "_odom", ros::Time(0));
                haveTransform = true;
            }
            catch (tf2::LookupException &ex)
            {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
        }
       

        state_pub.publish(block_state);

        ros::spinOnce();
        update_rate.sleep();
    }

    std::cout << "Shouldn't reach here" << std::endl;
}