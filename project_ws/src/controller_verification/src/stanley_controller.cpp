#include "waypoint_loader.h"
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <carla_msgs/CarlaSpeedometer.h>

class StanleyController {
public:
    StanleyController(const std::vector<std::pair<double, double>>& waypoints) 
        : waypoints_(waypoints) {
        nh_.param("k", k_, 1.9);
        nh_.param("k_p", k_p_, 1.0);
        nh_.param("h", h_,0.0);
        nh_.param("max_steering_angle", max_steering_angle_, 0.61);

        // odometry_sub_ = nh_.subscribe("/carla/hero/odometry", 100, &StanleyController::odomCallback, this);
        gnss_sub_ = nh_.subscribe("/carla/hero/GPS", 10, &StanleyController::gnssCallback, this);
        imu_sub_ = nh_.subscribe("/carla/hero/IMU", 10, &StanleyController::imuCallback, this);

        speedometer_sub_ = nh_.subscribe("/carla/hero/Speed", 100, &StanleyController::speedometerCallback, this);
        
        control_pub_ = nh_.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/hero/vehicle_control_cmd", 100);
        control_cmd.header.frame_id = "vehicle_control_msg";
    }

    // // odometry 데이터 콜백함수
    // void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    //     current_position_ = {msg->pose.pose.position.x, -(msg->pose.pose.position.y)};
    //     tf::Quaternion quat;
    //     tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    //     double roll, pitch, yaw;
    //     tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    //     current_yaw_ = -yaw;  // current_yaw_ 설정

    //     ROS_INFO("Odometry received: Position (%f, %f), Yaw %f", current_position_.first, current_position_.second, current_yaw_);
    // }

    // Speedometer 데이터 콜백 함수
    void speedometerCallback(const carla_msgs::CarlaSpeedometer::ConstPtr& msg) {
        current_velocity_ = static_cast<double>(msg->speed); 
        ROS_INFO("Speedometer received: Speed %f", current_velocity_);
    }

    // GNSS 데이터 콜백 함수
    void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        auto gps_location = _gps_to_location({{"lat", msg->latitude}, {"lon", msg->longitude}});
        ROS_INFO("gnssCallBack");
        current_position_.first = gps_location["x"];
        current_position_.second = -gps_location["y"];
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
    



    // CTE 계산 함수
    double calculateCTE(const std::pair<double, double>& prev_waypoint, const std::pair<double, double>& target_waypoint, const std::pair<double, double>& current_position) {
        double a = target_waypoint.second - prev_waypoint.second;
        double b = prev_waypoint.first - target_waypoint.first;
        double c = target_waypoint.first * prev_waypoint.second - prev_waypoint.first * target_waypoint.second;

        double cte = std::abs(a * current_position.first + b * current_position.second + c) / sqrt(a * a + b * b);
        double dx = target_waypoint.first - prev_waypoint.first;
        double dy = target_waypoint.second - prev_waypoint.second;
        double error_direction = (dx * (current_position.second - prev_waypoint.second)) - (dy * (current_position.first - prev_waypoint.first));
        if (error_direction > 0) {
            cte = -cte;
        }

        ROS_INFO("CTE calculated: %f", cte);
        return cte;
    }

    void calculateControl() {
        if (waypoints_.empty()) {
            ROS_WARN("No waypoints available.");
            return;
        }

        double min_dist = std::numeric_limits<double>::max();
        int closest_waypoint_index = -1;
        double search_radius = 5.0;

        for (int i = 0; i < waypoints_.size(); ++i) {
            double dx = waypoints_[i].first - current_position_.first;
            double dy = waypoints_[i].second - current_position_.second;
            double dist = sqrt(dx * dx + dy * dy);

            if (dist < search_radius && dist < min_dist) {
                min_dist = dist;
                closest_waypoint_index = i;
            }
        }

        if (closest_waypoint_index == -1) {
            ROS_WARN("No waypoints found in search radius.");
            return;
        }

        int target_waypoint_index = closest_waypoint_index + 1;
        if (target_waypoint_index >= waypoints_.size()) {
            target_waypoint_index = waypoints_.size() - 1;
        }

        std::pair<double, double> prev_waypoint = waypoints_[closest_waypoint_index];
        std::pair<double, double> target_waypoint = waypoints_[target_waypoint_index];

        double cte = calculateCTE(prev_waypoint, target_waypoint, current_position_);

        double dx = target_waypoint.first - prev_waypoint.first;
        double dy = target_waypoint.second - prev_waypoint.second;
        double path_yaw = atan2(dy, dx);
        double heading_error = path_yaw - current_yaw_;
        double delta = k_p_ * heading_error + atan2(k_ * cte, h_ + current_velocity_);

        delta = std::max(-max_steering_angle_, std::min(delta, max_steering_angle_));

        control_cmd.steer = delta / max_steering_angle_;
        control_cmd.brake = 0.0;
        control_cmd.throttle = 0.38;
        control_cmd.gear = 1;
        control_cmd.manual_gear_shift = false;
        control_cmd.reverse = false;
        control_cmd.header.stamp = ros::Time::now();

        ROS_INFO("Control command: Steer = %f, Throttle = %f", control_cmd.steer, control_cmd.throttle);

        control_pub_.publish(control_cmd);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber gnss_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber speedometer_sub_; 
    // ros::Subscriber odometry_sub_;
    ros::Publisher control_pub_;
    
    std::pair<double, double> current_position_;
    double current_yaw_;
    double current_velocity_;

    carla_msgs::CarlaEgoVehicleControl control_cmd;

    std::vector<std::pair<double, double>> waypoints_;
    
    double k_;
    double k_p_;
    double h_;
    double max_steering_angle_;
     // gnss 좌표에서 carla 좌표계
    std::map<std::string, double> _gps_to_location(std::map<std::string, double> gps) {
        double EARTH_RADIUS_EQUA = 6378137.0;
        double mx = gps["lon"] * M_PI * EARTH_RADIUS_EQUA / 180.0;
        double my = std::log(tan((90.0 + gps["lat"]) * M_PI / 360.0)) * EARTH_RADIUS_EQUA ;
        return {{"x", mx}, {"y", my}};
    }
};

int main(int argc, char** argv) {
    // Waypoints 로드
    std::string waypoint_file = "/home/ailab/carla-ros-bridge/project_ws/src/my_stanley_controller/src/waypoints.txt";
    std::vector<std::pair<double, double>> waypoints = WaypointLoader::loadWaypointsFromFile(waypoint_file);

    // Waypoints가 잘 로드되었는지 확인
    if (waypoints.empty()) {
        std::cerr << "Failed to load waypoints. Exiting." << std::endl;
        return -1;
    }

    // ROS 노드 초기화
    ros::init(argc, argv, "stanley_controller_node");

    // StanleyController 
    StanleyController controller(waypoints);

    ros::Rate rate(40);  // 100Hz

    while (ros::ok()) {
        controller.calculateControl();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
