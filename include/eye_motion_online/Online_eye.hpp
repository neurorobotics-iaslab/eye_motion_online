#ifndef EYE_MOTION_ONLINE_ONLINE_EYE_HPP
#define EYE_MOTION_ONLINE_ONLINE_EYE_HPP

#include <ros/ros.h>
#include <geometry_msgs/Point.h>   

#include "rosneuro_msgs/NeuroEvent.h"
#include "eye_decoder/Eye.h"
#include "feedback_cvsa/Repeat_trial.h"
#include "feedback_cvsa/TrainingCVSA.h"

#include <cmath>



namespace eye{

class Online_eye{
    public:
        Online_eye(void);
        ~Online_eye(void);

        void eventsCallback(const rosneuro_msgs::NeuroEvent::ConstPtr& msg);
        void eyeCallback(const eye_decoder::Eye::ConstPtr& msg);
        bool configure();

        void run(void);

        void publish();

        geometry_msgs::Point mean(std::vector<geometry_msgs::Point> points);
        geometry_msgs::Point std(std::vector<geometry_msgs::Point> points, geometry_msgs::Point mean);

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle p_nh_;
        ros::Subscriber sub_events_;
        ros::Subscriber sub_cvsa_;
        ros::ServiceClient srv_;

        bool in_online_cf_;
        bool in_calibration_;

        std::vector<int> classes_;
        int th_frame_;
        int cont_frame_;
        bool just_repeated_;

        bool calib_center_ = false;
        std::vector<geometry_msgs::Point> l_center_;
        std::vector<geometry_msgs::Point> r_center_;
        int c_class_;

        geometry_msgs::Point mean_l_center_;
        geometry_msgs::Point mean_r_center_;
        geometry_msgs::Point std_l_center_;
        geometry_msgs::Point std_r_center_;

};
}

#endif