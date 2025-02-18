#include "eye_motion_online/Offline_cf.hpp"

namespace eye{

Offline_cf::Offline_cf() : p_nh_("~"){
    this->sub_events_ = this->nh_.subscribe("events/bus", 1, &Offline_cf::eventsCallback, this);
    this->sub_cvsa_   = this->nh_.subscribe("cvsa/eye", 1, &Offline_cf::eyeCallback, this);
}

bool Offline_cf::configure(){
    this->in_trial_ = false;
    this->idx_trial_ = 0;

    return true;
}

Offline_cf::~Offline_cf(void) {
    this->sub_events_.shutdown();
    this->sub_cvsa_.shutdown();
}

geometry_msgs::Point Offline_cf::mean(std::vector<geometry_msgs::Point> points) {
    geometry_msgs::Point mean;
    mean.x = 0.0;
    mean.y = 0.0;
    mean.z = 0.0;

    for(auto& p : points) {
        mean.x += p.x;
        mean.y += p.y;
        mean.z += p.z;
    }

    mean.x = std::ceil(mean.x / points.size());
    mean.y = std::ceil(mean.y / points.size());
    mean.z = std::ceil(mean.z / points.size());
    
    return mean;
}

geometry_msgs::Point Offline_cf::std(std::vector<geometry_msgs::Point> points, geometry_msgs::Point mean) {
    geometry_msgs::Point std;
    std.x = 0.0;
    std.y = 0.0;
    std.z = 0.0;

    for(auto& p : points) {
        std.x += (p.x - mean.x) * (p.x - mean.x);
        std.y += (p.y - mean.y) * (p.y - mean.y);
        std.z += (p.z - mean.z) * (p.z - mean.z);
    }

    std.x = std::ceil(std.x / points.size());
    std.y = std::ceil(std.y / points.size());
    std.z = std::ceil(std.z / points.size());
    
    return std;
}

void Offline_cf::eventsCallback(const rosneuro_msgs::NeuroEvent::ConstPtr& msg) {
    // check if in cf
    if(msg->event == feedback::Events::Fixation){
        this->in_trial_ = true;
    }else if (msg->event == feedback::Events::CFeedback + feedback::Events::Off){
        this->in_trial_ = false;
        this->idx_trial_ += 1;
        ROS_INFO("Trial %d", this->idx_trial_);

        geometry_msgs::Point mean_l = this->mean(this->l_center_);
        geometry_msgs::Point mean_r = this->mean(this->r_center_);

        std::vector<geometry_msgs::Point> diff_l_mean;
        std::vector<geometry_msgs::Point> diff_r_mean;
        for(auto& p : this->l_center_){
            geometry_msgs::Point diff;
            diff.x = p.x - mean_l.x;
            diff.y = p.y - mean_l.y;
            diff.z = p.z - mean_l.z;
            diff_l_mean.push_back(diff);
        }
        for(auto& p : this->r_center_){
            geometry_msgs::Point diff;
            diff.x = p.x - mean_r.x;
            diff.y = p.y - mean_r.y;
            diff.z = p.z - mean_r.z;
            diff_r_mean.push_back(diff);
        }

        double max_r_x = 0.0;
        double max_r_y = 0.0;
        double max_l_x = 0.0;
        double max_l_y = 0.0;

        for(auto& p : diff_l_mean){
            if(std::abs(p.x) > max_l_x){
                max_l_x = std::abs(p.x);
            }
            if(std::abs(p.y) > max_l_y){
                max_l_y = std::abs(p.y);
            }
        }

        for(auto& p : diff_r_mean){
            if(std::abs(p.x) > max_r_x){
                max_r_x = std::abs(p.x);
            }
            if(std::abs(p.y) > max_r_y){
                max_r_y = std::abs(p.y);
            }
        }

        ROS_INFO("Max l_x: %f, Max l_y: %f", max_l_x, max_l_y);
        ROS_INFO("Max r_x: %f, Max r_y: %f", max_r_x, max_r_y);

        this->l_center_.clear();
        this->r_center_.clear();
    }
}

void Offline_cf::eyeCallback(const eye_decoder::Eye::ConstPtr& msg) {

    if(this->in_trial_){
        geometry_msgs::Point p_l = msg->left_pupil;
        geometry_msgs::Point p_r = msg->right_pupil;

        cv::Mat image = cv_bridge::toCvCopy(msg->face_image, "bgr8")->image;
        cv::circle(image, cv::Point(p_l.x, p_l.y), 2, cv::Scalar(0, 255, 0), 2);
        cv::circle(image, cv::Point(p_r.x, p_r.y), 2, cv::Scalar(0, 255, 0), 2);
        cv::imshow("Camera Image", image);
        cv::waitKey(1);

        this->l_center_.push_back(p_l);
        this->r_center_.push_back(p_r);
    }
}

void Offline_cf::run(void) {
    ros::spin();
}

}