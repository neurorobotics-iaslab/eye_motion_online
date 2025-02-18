#include <ros/ros.h>

#include "eye_motion_online/Offline_cf.hpp"


int main(int argc, char** argv) {

    // ros initialization
    ros::init(argc, argv, "offline_cf_node");

    eye::Offline_cf offline_cf;

    if(offline_cf.configure() == false){
        ROS_ERROR("online eye node configuration failed");
        ros::shutdown();
        return 0;
    }
    offline_cf.run();

    ros::shutdown();

    return 0;
    
    
}
