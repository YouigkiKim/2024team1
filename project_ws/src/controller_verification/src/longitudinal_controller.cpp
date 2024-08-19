// 종방향 속도제어기 완성. 08 16 14 26
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <carla_msgs/CarlaSpeedometer.h>

// PID 제어기 클래스
class PIDController {
public:
    PIDController(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd), integral_(0.0), prev_error_(0.0) {}

    double compute(double setpoint, double pv, double dt) {     //pv: process variable , setpoint은 reference
        double error = setpoint - pv;
        integral_ += error * dt;                              // 수치적분
        double derivative = (error - prev_error_) / dt;       // 수치미분
        prev_error_ = error;
            // 제어기 출력 계산
        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

        // 디버깅을 위한 로그 출력 (추가된 부분)
        ROS_INFO("PID Computation -> Setpoint: %f, PV: %f, Error: %f, Integral: %f, Derivative: %f, Output: %f", 
                 setpoint, pv, error, integral_, derivative, output);

        return output; // 제어기 출력 반환
        
    }

private:
    double kp_, ki_, kd_;
    double integral_, prev_error_;
};

class LongitudinalController {
public:
    LongitudinalController() : pid_(1.20431,0.031026405, 0.02805) {  // PID 제어기 초기화 (튜닝)
        // ROS 노드 핸들러 초기화
        ros::NodeHandle nh;

        // 현재 속도 구독자 초기화
        speedometer_sub_ = nh.subscribe("/carla/hero/speedometer", 10, &LongitudinalController::speedometerCallback, this);

        // 제어 명령 퍼블리셔 초기화
        control_pub_ = nh.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/hero/vehicle_control_cmd", 100);
        
        // 기준 속도, 현재 속도 변수값 초기화
        target_speed_ = 2.0; // 초기 값 설정
        current_speed_ = 0.0;
    }

    // 현재 속도 콜백 함수
    void speedometerCallback(const carla_msgs::CarlaSpeedometer::ConstPtr& msg) {
        current_speed_ = static_cast<double>(msg->speed); 
        ROS_INFO("Speedometer received: Speed %f", current_speed_);
    }

    // 기준 속도 갱신 함수
    void updateTargetSpeed(const ros::Time& start_time, const ros::Time& current_time) {
        ros::Duration elapsed_time = current_time - start_time;

        if (elapsed_time.toSec() < 8.0) {
            target_speed_ = 5.0; // 5초 동안 
        } else if (elapsed_time.toSec() < 13.0) {
            target_speed_ = 11.0; // 다음 5초 동안 
        } else if (elapsed_time.toSec() < 18.0) {
            target_speed_ = 5.0; // 다음 5초 동안 6        
        } else {
            target_speed_ = 0.0; // 이후에는 0 m/s

        }
    }

    // 제어 명령 계산 함수
    void computeControl(double dt) {
        double control_effort = pid_.compute(target_speed_, current_speed_, dt);

        // control_effort 값 제한
        control_effort = std::tanh(control_effort);

        // 제어 출력 확인
        ROS_INFO("Control Effort: %f", control_effort);
        ROS_INFO("Target Speed: %f, Current Speed: %f", target_speed_, current_speed_);

        carla_msgs::CarlaEgoVehicleControl control_cmd;

        // 메시지에 타임스탬프 추가
        control_cmd.header.stamp = ros::Time::now();

        // control_effort: PID 제어기의 출력값, 양수이면 가속, 음수이면 감속
        if (control_effort > 0.0) {  
            control_cmd.throttle = control_effort; 
            control_cmd.brake = 0.0;
        } else {  // 음수일 때는 제동
            control_cmd.throttle = 0.0;
            control_cmd.brake = -control_effort;
        }

        if (target_speed_== 0.0){
            control_cmd.throttle = 0.0;
        }
        control_cmd.hand_brake = false;
        control_cmd.reverse = false;
        control_cmd.gear = 1;
        control_cmd.manual_gear_shift = false;
        
        control_pub_.publish(control_cmd);
    }

private:
    ros::Subscriber speedometer_sub_;
    ros::Publisher control_pub_;
    double target_speed_;
    double current_speed_;
    PIDController pid_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "longitudinal_controller");

    LongitudinalController controller;

    // 제어 주기 설정
    ros::Rate rate(40);  // Hz

    ros::Time start_time = ros::Time::now();  // 시작 시간
    ros::Time prev_time = ros::Time::now();

    while (ros::ok()) {
        ros::spinOnce();

        ros::Time current_time = ros::Time::now();
        ros::Duration duration = current_time - prev_time;
        double dt = duration.toSec();
        prev_time = current_time;

        // 타겟 속도 갱신
        controller.updateTargetSpeed(start_time, current_time);

        // 제어 명령 계산
        controller.computeControl(dt);

        rate.sleep();
    }

    return 0;
}
