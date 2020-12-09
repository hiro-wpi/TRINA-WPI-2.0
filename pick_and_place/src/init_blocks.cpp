#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <gazebo_msgs/SetModelState.h>


gazebo_msgs::SetModelState block_state;



int main(int argc, char *argv[])
{
    //Setup ROS
    ros::init(argc, argv, "block_initializer");

    ros::NodeHandle nh;
    ros::Rate pause(25);
    ros::ServiceClient set_block_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, M_PI / 2);
    block_state.request.model_state.pose.orientation = tf2::toMsg(orientation);
    block_state.request.model_state.reference_frame = "trina2_1/base_link";

    // Set up the table
    block_state.request.model_state.model_name = "unit_box";
    block_state.request.model_state.pose.position.x = 1.1;
    block_state.request.model_state.pose.position.y = 0;
    block_state.request.model_state.pose.position.z = 0.25;
    set_block_client.call(block_state);
    //make sure service call was successful
    bool result = block_state.response.success;
    if (!result)
        ROS_WARN("service call to set_model_state failed!");
    else
        ROS_INFO("Done");
    ros::spinOnce();    
    pause.sleep();    

    //Set up the block instances
    for (int i=0; i<3;++i) {
        block_state.request.model_state.model_name = "unit_box_" + std::to_string(i);
        block_state.request.model_state.pose.position.x = 0.8;
        block_state.request.model_state.pose.position.y = 0.5 - i*0.2; // 0.5 - i*0.2 for left arm, -0.5 + i*0.2 for right
        block_state.request.model_state.pose.position.z = 0.54;
        set_block_client.call(block_state);
        //make sure service call was successful
        bool result = block_state.response.success;
        if (!result)
            ROS_WARN("service call to set_model_state failed!");
        else
            ROS_INFO("Done");
        ros::spinOnce();    
        pause.sleep();
    }

    std::cout << "Completed" << std::endl;
}