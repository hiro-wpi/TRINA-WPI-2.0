#include <ros/ros.h>

#include "pick_and_place/PickSrv.h"
#include "pick_and_place/PlaceSrv.h"
#include "pick_and_place/RecoverSrv.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pick_and_place_demo");

    ros::NodeHandle nh;
    ros::ServiceClient pick_client = nh.serviceClient<pick_and_place::PickSrv>("/trina2_1/trina_pick");
    ros::ServiceClient place_client = nh.serviceClient<pick_and_place::PlaceSrv>("/trina2_1/trina_place");
    ros::ServiceClient recover_client = nh.serviceClient<pick_and_place::RecoverSrv>("/trina2_1/trina_recover");

    pick_and_place::PickSrv pick_srv;
    pick_and_place::PlaceSrv place_srv;
    pick_and_place::RecoverSrv recover_srv;
    std::string object;
    place_srv.request.x = 0.7;
    place_srv.request.y = 0.0;
    recover_srv.request.cont = true;

    for (int i = 0; i < 3; i++)
    {
        object = "unit_box_" + std::to_string(i);
        pick_srv.request.pick_obj = object;

        //If pick service can be fixed, use this version with error checking : 
        if (pick_client.call(pick_srv))
        {
            ROS_INFO("Picked up block %i", i);
            if (place_client.call(place_srv))
            {
                ROS_INFO("Placed block");
            }
            else
            {
                ROS_ERROR("Failed to place, exiting");
                return -1;
            }
        }
        else
        {
            ROS_INFO("Failed to pick up block %i, executing CONTINUE option", i);
                ros::Duration(1).sleep(); // Make sure there is time for the state to update before we try again

            if (recover_client.call(recover_srv))
            {
                ROS_INFO("Recovery complete");
                if (place_client.call(place_srv))
                {
                    ROS_INFO("Placed block");
                }
                else
                {
                    ROS_ERROR("Failed to place, exiting");
                    return -1;
                }
            }
            else {
                ROS_INFO("Failed to recover, exiting");
                return -1;
            }
        }
        // pick_client.call(pick_srv);
        // place_client.call(place_srv);
    }
    return 0;
}