#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <proj_api.h>
#include <Eigen/Dense>

struct EgoState {
    float x;
    float y;
    float heading;
    float velocity;
};

class Localization {
public:
    Localization();
    ~Localization();
    EgoState getEgoState() const;

private:
    ros::Subscriber gps_sub;
    ros::Subscriber vehicle_status_sub;
    projPJ pj_latlong, pj_utm;
    EgoState ego;

    void cb_gps(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void cb_vehicle_status(const carla_msgs::CarlaEgoVehicleStatus::ConstPtr& msg);
};

#endif // LOCALIZATION_H