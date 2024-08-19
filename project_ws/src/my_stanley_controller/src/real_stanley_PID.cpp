#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <nav_msgs/Odometry.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <carla_msgs/CarlaSpeedometer.h>
#include <custom_msgs/ref_control.h>
#include <custom_msgs/vehicle_state.h>

// PID 제어기 클래스
class PIDController {
public:
    PIDController(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd), integral_(0.0), prev_error_(0.0) {}

    double compute(double setpoint, double pv, double dt) {
        double error = setpoint - pv;
        integral_ += error * dt;
        double derivative = (error - prev_error_) / dt;
        prev_error_ = error;

        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

        ROS_INFO("PID Computation -> Setpoint: %f, PV: %f, Error: %f, Integral: %f, Derivative: %f, Output: %f", 
                 setpoint, pv, error, integral_, derivative, output);

        return output;
    }

private:
    double kp_, ki_, kd_;
    double integral_, prev_error_;
};

// 통합된 제어기 클래스
class Controller {
public:
    Controller() : pid_(1.20431, 0.031026405, 0.02805) {
        nh_.param("k", k_, 2.81);
        nh_.param("k_p", k_p_, 1.0);
        nh_.param("h", h_, 10.0);
        nh_.param("max_steering_angle", max_steering_angle_, 0.61);  // rad

        // Subscribing
        vehicle_pose_sub_ = nh_.subscribe("/carla/hero/pose", 10, &Controller::vehiclestateCallback, this);
        speedometer_sub_ = nh_.subscribe("/carla/hero/Speed", 10, &Controller::speedometerCallback, this);
        ref_control_sub_ = nh_.subscribe("/ref/control", 10, &Controller::refControlCallback, this);

        // Publishing
        control_pub_ = nh_.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/hero/vehicle_control_cmd", 100);
        control_cmd_.header.frame_id = "vehicle_control_msg";
    }
    // 콜백 함수들
    void vehiclestateCallback(const custom_msgs::vehicle_state::ConstPtr& msg) {
        current_yaw_ = msg->heading;
        current_position_ = {msg->x , msg->y};
        ROS_INFO("Vehicle state received: x: %f ,y:%f, yaw:%f", msg->x, msg->y, msg->heading);
    }

    void speedometerCallback(const carla_msgs::CarlaSpeedometer::ConstPtr& msg) {
        current_velocity_ = static_cast<double>(msg->speed);
        ROS_INFO("Speedometer received: Speed %f", current_velocity_);
    }

    void refControlCallback(const custom_msgs::ref_control::ConstPtr& msg) {
        target_point_ = {msg->x , msg->y};
        target_velocity_ = msg->velocity;
        target_yaw_ = msg->yaw;
        aeb_flag_ = msg->aeb_flag;
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
        
        if (cross_product > 0) {
            cte = -cte;
        }

        ROS_INFO("CTE calculated: %f", cte);
        return cte;
    }

    void calculateControl(double dt) {
        // control_cmd_.throttle = 0.0;
        // control_cmd_.brake = 0.0;
        // control_cmd_.steer = 0.0;
        control_cmd_.gear = 1;
        control_cmd_.hand_brake = false;
        control_cmd_.manual_gear_shift = false;
        if (!aeb_flag_) {  // aeb_flag_가 0일 때 정상 주행
            // 횡방향 제어 (조향각)
            double cte = calculateCTE(current_position_, target_point_, target_yaw_);
         
            double heading_error = target_yaw_ - current_yaw_;

            while (heading_error > M_PI) {
            heading_error -= 2 * M_PI;
            }

            while (heading_error < -M_PI) {
            heading_error += 2 * M_PI;
            } 

            double delta = k_p_ * heading_error + atan2(k_ * cte, h_ + current_velocity_);

            
            delta = std::max(-max_steering_angle_, std::min(delta, max_steering_angle_));

            control_cmd_.steer = delta / max_steering_angle_;

            // 종방향 제어 (속도 제어)
            double control_effort = pid_.compute(target_velocity_, current_velocity_, dt);
            control_effort = std::tanh(control_effort);

            if (control_effort > 0.0) {
                control_cmd_.throttle = control_effort;
                control_cmd_.brake = 0.0;
            } else {
                control_cmd_.throttle = 0.0;
                control_cmd_.brake = -control_effort;
            }

            if (target_velocity_ == 0.0) {
                control_cmd_.throttle = 0.0;
                control_cmd_.brake = 0.0;
            }
        } 
        else {  // aeb_flag_가 1(true)일 때는 풀브레이크
            control_cmd_.throttle = 0.0;
            control_cmd_.brake = 1.0;  // 최대 제동
            ROS_INFO("AEB flag activated. Applying full brake.");
        }

        control_cmd_.gear = 1;
        control_cmd_.header.stamp = ros::Time::now();

        control_pub_.publish(control_cmd_);
        ROS_INFO("Control command published: Steer = %f, Throttle = %f, Brake = %f", control_cmd_.steer, control_cmd_.throttle, control_cmd_.brake);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber speedometer_sub_;
    ros::Subscriber vehicle_pose_sub_;
    ros::Subscriber ref_control_sub_;

    ros::Publisher control_pub_;

    std::pair<double, double> current_position_;
    double current_yaw_;
    double current_velocity_;
    
    carla_msgs::CarlaEgoVehicleControl control_cmd_;

    std::pair<double, double> target_point_;

    double target_velocity_;
    double target_yaw_;

    double k_;
    double k_p_;
    double h_;
    double max_steering_angle_;

    PIDController pid_;
    bool aeb_flag_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "Controller_node"); //  Node name : Controller_node

    Controller controller;

    ros::Rate rate(40);  // 40Hz

    ros::Time prev_time = ros::Time::now();
    const double fixed_dt = 1.0 / 40.0; // 고정된 dt 값 (0.025초)
    // while (ros::ok()) {
    //     ros::spinOnce();  // 콜백 함수 실행

    //     // 루프 시작 시간
    //     ros::Time loop_start_time = ros::Time::now();

    //     // dt 계산
    //     ros::Duration duration = loop_start_time - prev_time;
    //     double dt = duration.toSec();
    //     prev_time = loop_start_time;

    //     ROS_INFO("Controller dt: %f", dt);

    //     // 제어 연산 수행
    //     controller.calculateControl(dt);
        
    //     // 주기 유지
    //     rate.sleep();
    // }
    while (ros::ok()) {
        ros::spinOnce();  // 콜백 함수 실행

        // dt 계산을 루프 시작 지점에서 수행
        ros::Time loop_start_time = ros::Time::now();
        ros::Duration duration = loop_start_time - prev_time;
        double dt = duration.toSec();

        // 제어 연산 수행
        controller.calculateControl(fixed_dt);

        rate.sleep();
        prev_time = ros::Time::now(); 

        ROS_INFO("Controller dt: %f", dt);
    }
    return 0;
}