#include "localization.h"




namespace Localization{
    Localization::Localization(){};
    Localization::Localization(ros::NodeHandle& nh)
    : x_(6770.755371),y_(5443.435059), yaw_(-89.5*M_PI/180){
        gnss_sub_ = nh.subscribe<sensor_msgs::NavSatFix>("/carla/hero/GPS",10,&Localization::gnss_callback, this);
        pose_pub_ = nh.advertise<custom_msgs::vehicle_state>("/carla/hero/pose",10);
        pose_.x = x_;
        pose_.y = y_;

    };

    void Localization::projection(double lat, double lon){
        x_ = lon*M_PI*EARTH_RADIUS_EQUA/180;
        y_ = - EARTH_RADIUS_EQUA*std::log(tan((90+lat)*M_PI/360.0));
    }
    void Localization::gnss_callback(const sensor_msgs::NavSatFix::ConstPtr& msg){
        projection(static_cast<double>(msg -> latitude), static_cast<double>(msg -> longitude));
        ROS_INFO("gnss call back x,y (%f, %f)", pose_.x, pose_.y);
    }
    void Localization::publish(){
        pose_.x = x_;
        pose_.y = y_;
        pose_.heading = yaw_;
        pose_pub_.publish(pose_);
    }
    
}