#ifndef OFFLINE_CF_HPP
#define OFFLINE_CF_HPP

#include <ros/ros.h>
#include <geometry_msgs/Point.h>   
#include <sensor_msgs/Image.h>

#include "rosneuro_msgs/NeuroEvent.h"
#include "eye_decoder/Eye.h"
#include "feedback_cvsa/TrainingCVSA.h"

#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace eye{

class Offline_cf{
    public:
        Offline_cf(void);
        ~Offline_cf(void);

        void eventsCallback(const rosneuro_msgs::NeuroEvent::ConstPtr& msg);
        void eyeCallback(const eye_decoder::Eye::ConstPtr& msg);
        bool configure();

        void run(void);

        geometry_msgs::Point mean(std::vector<geometry_msgs::Point> points);
        geometry_msgs::Point std(std::vector<geometry_msgs::Point> points, geometry_msgs::Point mean);

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle p_nh_;
        ros::Subscriber sub_events_;
        ros::Subscriber sub_cvsa_;
        sensor_msgs::Image face_image_;

        bool in_trial_;
        int idx_trial_;

        std::vector<geometry_msgs::Point> l_center_;
        std::vector<geometry_msgs::Point> r_center_;
};
}
#endif