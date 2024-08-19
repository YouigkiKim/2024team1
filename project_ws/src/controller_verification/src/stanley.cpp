#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <chrono>
#include <vector>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <carla_msgs/CarlaSpeedometer.h>
#include <sensor_msgs/NavSatFix.h>  // GNSS 메시지 타입
#include <sensor_msgs/Imu.h>        // IMU 메시지 타입
#include <geometry_msgs/TwistStamped.h> // 속도계 메시지 타입
#include <fstream>
#include <string>
#include <sstream>



class StanleyController {
public:
    StanleyController() {
        nh_.param("k", k_, 0.8);
        nh_.param("k_p", k_p_, 0.8);
        nh_.param("h", h_, 20.0);
        nh_.param("max_steering_angle", max_steering_angle_, 1.2);

        odometry_sub_ = nh_.subscribe("/carla/hero/odometry",10, &StanleyController::odomCallback,this);
        speedometer_sub_ = nh_.subscribe("/carla/hero/Speed", 10, &StanleyController::speedometerCallback, this);
        waypoint_sub_ = nh_.subscribe("/ref/ref_point",10, &StanleyController::waypointCallback,this);
        
        control_pub_ = nh_.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/hero/vehicle_control_cmd", 100);
        control_cmd.header.frame_id = "vehicle_control_msg";
    }



    // odometry 데이터 콜백함수
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
        current_position_ = msg->pose.pose.position;
        
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        current_yaw_ = -yaw;  // current_yaw_ 설정
    }

    // Speedometer 데이터 콜백 함수
    void speedometerCallback(const carla_msgs::CarlaSpeedometer::ConstPtr& msg) {
        current_velocity_ = static_cast<double>(msg->speed); 
        ROS_INFO("Speed Callback");
    }

    // Waypoint 콜백함수
    void waypointCallback(const geometry_msgs::Point::ConstPtr& msg){
        target_waypoint_ =* msg;
        ROS_INFO("Waypoint Callback: Target waypoint received");
    }

    void calculateControl() {
        if (waypoints_.empty()) {
            ROS_WARN("No waypoints received.");
            return;
        }

        double min_dist = std::numeric_limits<double>::max();
        int closest_waypoint_index = -1;
        double search_radius = 10.0;

        for (int i = 0; i < waypoints_.size(); ++i) {
            double dx = current_x_ - waypoints_[i].x;
            double dy = current_y_ - waypoints_[i].y;
            double dist = sqrt(dx * dx + dy * dy);

            if (dist < search_radius && dist < min_dist) {
                min_dist = dist;
                closest_waypoint_index = i;
            }
        }

        if (closest_waypoint_index == -1) {
            ROS_WARN("No waypoints found within search radius.");
            return;
        }

        int target_waypoint_index = closest_waypoint_index + 1;

        if (target_waypoint_index >= waypoints_.size()) {
            target_waypoint_index = waypoints_.size() - 1;
        }

        geometry_msgs::Point target_waypoint = waypoints_[target_waypoint_index];

        double dx = target_waypoint_.x - current_x_;
        double dy = target_waypoint_.y - current_y_;
        double path_yaw = atan2(dy, dx);
        double cross_track_error = sqrt (dx * dx + dy * dy) ;
        double heading_error1 = path_yaw - current_yaw_;
        double heading_error = atan2(sin(heading_error1), cos(heading_error1));
        // double cross_track_error = min_dist;
     
        double delta = k_p_ * heading_error + atan2(k_ * cross_track_error, h_ + current_velocity_);

    
        if (delta > max_steering_angle_) {
            delta = max_steering_angle_;
        } else if (delta < -max_steering_angle_) {
            delta = -max_steering_angle_;
        }

        double delta_for_cmd = delta / 1.2;

        control_cmd.brake = 0.0;
        control_cmd.steer = delta_for_cmd;
        control_cmd.reverse = false;
        control_cmd.manual_gear_shift = false;
        control_cmd.throttle = 0.37;
        control_cmd.header.stamp = ros::Time::now();
        ROS_INFO("Publishing control command.");
        control_pub_.publish(control_cmd);
    }

private:
    ros::NodeHandle nh_;

    ros::Subscriber speedometer_sub_; 
    ros::Subscriber waypoint_sub_;
    ros::Subscriber odometry_sub_;
    ros::Publisher control_pub_;
    
    double current_x_;
    double current_y_;
    double current_yaw_;
    double current_velocity_; // 속도계 데이터를 받을 변수
    
    carla_msgs::CarlaEgoVehicleControl control_cmd;
    geometry_msgs::Point target_waypoint_;

    
    double k_;
    double k_p_;
    double h_;
    double max_steering_angle_;

   
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "stanley_test_node");


    // 경로 점 추종 StanleyControl
    StanleyController controller;

    ros::Rate rate(100);  // 100Hz

    while (ros::ok()) {
        controller.calculateControl();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
