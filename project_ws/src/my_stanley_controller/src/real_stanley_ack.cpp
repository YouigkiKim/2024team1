#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <carla_msgs/CarlaSpeedometer.h>
#include <custom_msgs/ref_control.h> 
#include <custom_msgs/vehicle_state.h> 

class StanleyController {
public:
    StanleyController() : target_speed_(7.0) {  // 목표 속도:7m/s
        nh_.param("k", k_, 7.0);
        nh_.param("k_p", k_p_, 1.0);
        nh_.param("h", h_, 0.0);
        nh_.param("max_steering_angle", max_steering_angle_, 0.61);

        vehicle_pose_sub_ = nh_.subscribe("/carla/hero/pose",10, &StanleyController::vehiclestateCallback, this);
        speedometer_sub_ = nh_.subscribe("/carla/hero/Speed", 10, &StanleyController::speedometerCallback, this);
        ref_control_sub_ = nh_.subscribe("/ref/control", 10, &StanleyController::refControlCallback, this);

        control_pub_ = nh_.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/hero/vehicle_control_cmd", 10);
        speed_pub_ = nh_.advertise<ackermann_msgs::AckermannDrive>("/carla/hero/ackermann_cmd", 10);
    }

    void vehiclestateCallback(const custom_msgs::vehicle_state::ConstPtr& msg){
        current_position_ ={msg->x, msg->y};
        current_yaw_ = msg->heading;
        ROS_INFO("Vehicle state received: x: %f ,y:%f, yaw:%f",msg->x, msg->y , msg->heading);
    }

    void speedometerCallback(const carla_msgs::CarlaSpeedometer::ConstPtr& msg) {
        current_velocity_ = static_cast<double>(msg->speed); 
        ROS_INFO("Speedometer received: Speed %f", current_velocity_);
    }

    void refControlCallback(const custom_msgs::ref_control::ConstPtr& msg) {
        target_point_ = {msg->x, msg->y};
        target_velocity_ = msg->velocity;
        target_yaw_ = msg->yaw;
        ROS_INFO("Received ref_control: x=%f, y=%f, velocity=%f, yaw=%f", msg->x, msg->y, msg->velocity, msg->yaw);
    }

    double calculateCTE(const std::pair<double, double>& current_position, const std::pair<double, double>& target_point, double target_yaw) {
        double dx = target_point.first - current_position.first;
        double dy = target_point.second - current_position.second;

        double a = std::tan(target_yaw);
        double b = -1.0;
        double c = target_point.second - a * target_point.first;

        double cte = std::abs(a * current_position.first + b * current_position.second + c) / sqrt(a * a + b * b);

        double cross_product = dx * std::sin(target_yaw) - dy * std::cos(target_yaw);
        if (cross_product < 0) {
            cte = -cte;
        }

        ROS_INFO("CTE calculated: %f", cte);
        return cte;
    }

    void calculateControl() {
        double cte = calculateCTE(current_position_, target_point_, target_yaw_);

        double dx = target_point_.first - current_position_.first;
        double dy = target_point_.second - current_position_.second;
        double path_yaw = atan2(dy, dx);
        double heading_error = path_yaw - current_yaw_;
        double delta = k_p_ * heading_error + atan2(k_ * cte, h_ + current_velocity_);

        if (delta > max_steering_angle_) {
            delta = max_steering_angle_;
        } else if (delta < -max_steering_angle_) {
            delta = -max_steering_angle_;
        }

        // 조향 명령 유지
        carla_msgs::CarlaEgoVehicleControl control_cmd;
        control_cmd.steer = delta / max_steering_angle_;
        control_cmd.brake = 0.0;
        control_cmd.throttle = 0.37;  
        control_cmd.gear = 1;
        control_cmd.manual_gear_shift = false;
        control_cmd.reverse = false;
        control_cmd.header.stamp = ros::Time::now();
        control_pub_.publish(control_cmd);

        // 속도 제어 명령 보내기
        ackermann_msgs::AckermannDrive ackermann_cmd;
        ackermann_cmd.speed = target_speed_;  // 목표 속도 5m/s로 설정
        speed_pub_.publish(ackermann_cmd);

        ROS_INFO("Control command published: Steer = %f, Speed = %f", control_cmd.steer, ackermann_cmd.speed);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber speedometer_sub_; 
    ros::Subscriber vehicle_pose_sub_;
    ros::Subscriber ref_control_sub_;
    ros::Publisher control_pub_;
    ros::Publisher speed_pub_;

    std::pair<double, double> current_position_;
    double current_yaw_;
    double current_velocity_;
    double target_speed_;  // 목표 속도 설정 (5 m/s)
    std::pair<double, double> target_point_;
    double target_velocity_;
    double target_yaw_;
    
    double k_;
    double k_p_;
    double h_;
    double max_steering_angle_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "stanley_controller_node");

    StanleyController controller;

    ros::Rate rate(100);  // 50Hz

    while (ros::ok()) {
        controller.calculateControl();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
