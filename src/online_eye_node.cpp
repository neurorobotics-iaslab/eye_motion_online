#include <ros/ros.h>

#include "eye_motion_online/Online_eye.hpp"


int main(int argc, char** argv) {

    // ros initialization
    ros::init(argc, argv, "online_eye_node");

    eye::Online_eye online_eye;

    if(online_eye.configure() == false){
        ROS_ERROR("online eye node configuration failed");
        ros::shutdown();
        return 0;
    }
    online_eye.run();

    ros::shutdown();

    return 0;
    
    
}
