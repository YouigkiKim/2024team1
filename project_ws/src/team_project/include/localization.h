#pragma once

#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include "ros/ros.h"
#include <sensor_msgs/NavSatFix.h>
#include <custom_msgs/vehicle_state.h>
#include <cmath>

#define EARTH_RADIUS_EQUA 6378137.0   

namespace Localization{
    class Localization{

        public:
        Localization();
        Localization(ros::NodeHandle& nh);
        void projection(double lat, double lon);
        ros::Publisher          pose_pub_;
        void publish();

        private:
        void gnss_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
        //pub
        double x_;
        double y_;
        double yaw_;
        //sub
        ros::Subscriber                gnss_sub_;
        //msg
        custom_msgs::vehicle_state     pose_;
        sensor_msgs::NavSatFix         gnss_;

    };
}

#endif //LOCALIZATION_H