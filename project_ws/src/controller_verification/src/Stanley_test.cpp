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

// // CSV에서 좌표 데이터를 읽어서 벡터에 저장
// std::vector<geometry_msgs::Point> loadWaypointsFromCSV(const std::string& csv_file) {
//     std::vector<geometry_msgs::Point> waypoints;
//     std::ifstream file(csv_file);

//     if (!file.is_open()) {
//         ROS_ERROR("Cannot open the file: %s", csv_file.c_str());
//         return waypoints;
//     }

//     std::string line;
//     while (std::getline(file, line)) {
//         std::istringstream ss(line);
//         std::string x_str, y_str;

//         if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',')) {
//             geometry_msgs::Point point;
//             point.x = std::stod(x_str);
//             point.y = std::stod(y_str);
//             waypoints.push_back(point);
//         }
//     }
//     ROS_INFO("Waypoints loaded from CSV.");
//     return waypoints;
// }

class StanleyController {
public:
    StanleyController() {
        nh_.param("k", k_, 0.6);
        nh_.param("k_p", k_p_, 0.7);
        nh_.param("h", h_, 15.0);
        nh_.param("max_steering_angle", max_steering_angle_, 1.2);

        gnss_sub_ = nh_.subscribe("/carla/hero/GPS", 10, &StanleyController::gnssCallback, this);
        imu_sub_ = nh_.subscribe("/carla/hero/IMU", 10, &StanleyController::imuCallback, this);

        // odometry_sub_ = nh_.subscribe("/carla/hero/odometry",10, &StanleyController::odomCallback,this);
        speedometer_sub_ = nh_.subscribe("/carla/hero/Speed", 10, &StanleyController::speedometerCallback, this);
        waypoint_sub_ = nh_.subscribe("/ref/ref_point",10, &StanleyController::waypointCallback,this);
        
        control_pub_ = nh_.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/hero/vehicle_control_cmd", 100);
        control_cmd.header.frame_id = "vehicle_control_msg";
    }

    // GNSS 데이터 콜백 함수
    void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        auto gps_location = _gps_to_location({{"lat", msg->latitude}, {"lon", msg->longitude}});
        ROS_INFO("gnssCallBack");
        current_x_ = gps_location["x"];
        current_y_ = gps_location["y"];
    }

    // IMU 데이터 콜백 함수
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg->orientation, quat);
        ROS_INFO("IMU CallBack");
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        current_yaw_ = -yaw;
    }

    // // odometry 데이터 콜백함수
    // void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    //     current_position_ = msg->pose.pose.position;
        
    //     tf::Quaternion quat;
    //     tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    //     double roll, pitch, yaw;
    //     tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    //     current_yaw_ = yaw;  // current_yaw_ 설정
    // }

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
        // if (waypoints_.empty()) {
        //     ROS_WARN("No waypoints received.");
        //     return;
        // }

        // double min_dist = std::numeric_limits<double>::max();
        // int closest_waypoint_index = -1;
        // double search_radius = 10.0;

        // for (int i = 0; i < waypoints_.size(); ++i) {
        //     double dx = current_x_ - waypoints_[i].x;
        //     double dy = current_y_ - waypoints_[i].y;
        //     double dist = sqrt(dx * dx + dy * dy);

        //     if (dist < search_radius && dist < min_dist) {
        //         min_dist = dist;
        //         closest_waypoint_index = i;
        //     }
        // }

        // if (closest_waypoint_index == -1) {
        //     ROS_WARN("No waypoints found within search radius.");
        //     return;
        // }

        // int target_waypoint_index = closest_waypoint_index + 1;

        // if (target_waypoint_index >= waypoints_.size()) {
        //     target_waypoint_index = waypoints_.size() - 1;
        // }

        // geometry_msgs::Point target_waypoint = waypoints_[target_waypoint_index];

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
    ros::Subscriber gnss_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber speedometer_sub_; 
    ros::Subscriber waypoint_sub_;
    // ros::Subscriber odometry_sub_;
    ros::Publisher control_pub_;
    
    double current_x_;
    double current_y_;
    double current_yaw_;
    double current_velocity_; // 속도계 데이터를 받을 변수
    
    carla_msgs::CarlaEgoVehicleControl control_cmd;
    geometry_msgs::Point target_waypoint_;
    // geometry_msgs::Point current_position_;  // 현재 위치를 위한 변수
    
    double k_;
    double k_p_;
    double h_;
    double max_steering_angle_;

    // gnss 좌표에서 carla 좌표계
    std::map<std::string, double> _gps_to_location(std::map<std::string, double> gps) {
        double EARTH_RADIUS_EQUA = 6378137.0;
        double mx = gps["lon"] * M_PI * EARTH_RADIUS_EQUA / 180.0;
        double my = std::log(tan((90.0 + gps["lat"]) * M_PI / 360.0)) * -EARTH_RADIUS_EQUA ;
        return {{"x", mx}, {"y", my}};
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "stanley_test_node");

    // CSV 파일에서 경로 점 로드
    // std::vector<geometry_msgs::Point> waypoints = loadWaypointsFromCSV("/home/ailab/carla-ros-bridge/project_ws/src/controller_verification/src/waypoints.csv");

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
