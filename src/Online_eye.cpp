#include "eye_motion_online/Online_eye.hpp"

namespace eye{

Online_eye::Online_eye(void) : p_nh_("~"){
    this->sub_events_ = this->nh_.subscribe("events/bus", 1, &Online_eye::eventsCallback, this);
    this->sub_cvsa_   = this->nh_.subscribe("cvsa/eye", 1, &Online_eye::eyeCallback, this);
    this->srv_        = this->nh_.serviceClient<feedback_cvsa::Repeat_trial>("/cvsa/repeat_trial");

    
}

bool Online_eye::configure(){

    if(this->p_nh_.getParam("th_frame", this->th_frame_) == false) {
        ROS_ERROR("Parameter 'th_frame' is mandatory");
        return false;
    }

    if(this->p_nh_.getParam("classes", this->classes_) == false) {
        ROS_ERROR("Parameter 'classes' is mandatory");
        return false;
    }  

    this->in_online_cf_ = false;
    this->in_calibration_ = false;
    this->cont_frame_ = 0;
    this->just_repeated_ = false;

    return true;
}

Online_eye::~Online_eye(void) {
    this->sub_events_.shutdown();
    this->sub_cvsa_.shutdown();
    this->srv_.shutdown();
}

geometry_msgs::Point Online_eye::mean(std::vector<geometry_msgs::Point> points) {
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

geometry_msgs::Point Online_eye::std(std::vector<geometry_msgs::Point> points, geometry_msgs::Point mean) {
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

void Online_eye::eventsCallback(const rosneuro_msgs::NeuroEvent::ConstPtr& msg) {

    if(msg->event == feedback::Events::StartCalibEye) {
        this->in_calibration_ = true;
    }else if(msg->event == feedback::Events::StartCalibEye + feedback::Events::Off) {
        this->in_calibration_ = false;
    }

    // check for calibration with eyes looking to the center of the screen
    if(this->in_calibration_ && msg->event == eye::Center_event){
        this->calib_center_ = true;
    }else if(this->calib_center_ && msg->event ==  eye::Center_event + feedback::Events::Off){
        this->calib_center_ = false;
        this->mean_l_center_ = this->mean(this->l_center_);
        this->mean_r_center_ = this->mean(this->r_center_);
        this->std_l_center_ = this->std(this->l_center_, this->mean_l_center_);
        this->std_r_center_ = this->std(this->r_center_, this->mean_r_center_);

        //std::cout << "m_l_x: " << this->mean_l_center_.x << " std_x: " << this->std_l_center_.x << std::endl;
        //std::cout << "m_r_x: " << this->mean_r_center_.x << " std_x: " << this->std_r_center_.x << std::endl;
    }

    if(!this->in_calibration_) {
        auto find = std::find(this->classes_.begin(), this->classes_.end(), msg->event);
        if(find != this->classes_.end()){
            this->c_class_ = msg->event;
            this->cont_frame_ = 0;
        }else if(msg->event == feedback::Events::CFeedback) {
            this->in_online_cf_ = true;
        }else if(msg->event == feedback::Events::CFeedback + feedback::Events::Off || msg->event == feedback::Events::Miss || msg->event == feedback::Events::Hit) {
            this->in_online_cf_ = false;
            this->just_repeated_ = false;
        }
    }
    
}

void Online_eye::eyeCallback(const eye_decoder::Eye::ConstPtr& msg) {

    if(this->in_calibration_ && this->calib_center_){
        this->l_center_.push_back(msg->left_pupil);
        this->r_center_.push_back(msg->right_pupil);
    }

    if(this->in_online_cf_){
        geometry_msgs::Point p_l = msg->left_pupil;
        geometry_msgs::Point p_r = msg->right_pupil;

        bool l = p_l.x > this->mean_l_center_.x + this->std_l_center_.x || p_l.x < this->mean_l_center_.x - this->std_l_center_.x;
        bool r = p_r.x > this->mean_r_center_.x + this->std_r_center_.x || p_r.x < this->mean_r_center_.x - this->std_r_center_.x;

        if(l && r && this->cont_frame_ < this->th_frame_){
            this->cont_frame_ ++;
            std::cout << "l_x:" << p_l.x - mean_l_center_.x << " r_x: " << p_r.x - mean_r_center_.x << std::endl;
        }
        
        if(this->cont_frame_ >= this->th_frame_ && !this->just_repeated_){

            feedback_cvsa::Repeat_trial srv;
            srv.request.class2repeat = this->c_class_;
            if(this->srv_.call(srv)) {
                std::cout << "Repeat trial of class " << this->c_class_ << std::endl;
            }else{
                ROS_ERROR("Failed to call service");
            }
            this->cont_frame_= 0;
            this->just_repeated_ = true;

        }

    }
    
}

void Online_eye::run(void) {
    ros::spin();
}

}
