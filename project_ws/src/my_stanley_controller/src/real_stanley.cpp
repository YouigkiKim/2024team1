#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <carla_msgs/CarlaSpeedometer.h>
#include <custom_msgs/ref_control.h> 
#include <custom_msgs/vehicle_state.h> 

class StanleyController {
public:
    StanleyController() {
        nh_.param("k", k_, 5.0);     // stanley constant 
        nh_.param("k_p", k_p_, 1.0); // proportional constant
        nh_.param("h", h_, 60.0);    // speed constant
        nh_.param("max_steering_angle", max_steering_angle_, 0.61);
        

        // Subscribing
        // odometry_sub_ = nh_.subscribe("/carla/hero/odometry", 100, &StanleyController::odomCallback, this);
        vehicle_pose_sub_ = nh_.subscribe("/carla/hero/pose",10, &StanleyController::vehiclestateCallback, this);
        speedometer_sub_ = nh_.subscribe("/carla/hero/Speed", 10, &StanleyController::speedometerCallback, this);
        ref_control_sub_ = nh_.subscribe("/ref/control", 10, &StanleyController::refControlCallback, this);
        
        // Publishing
        control_pub_ = nh_.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/hero/vehicle_control_cmd", 100);
        control_cmd.header.frame_id = "vehicle_control_msg";
    }

    // // odometry 데이터 콜백함수
    // void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    //     current_position_ = {(msg->pose.pose.position.x), -(msg->pose.pose.position.y)};
    //     tf::Quaternion quat;
    //     tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    //     double roll, pitch, yaw;
    //     tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    //     current_yaw_ = -yaw;  // current_yaw_ 설정
    //     ROS_INFO("Odometry received: Position (%f, %f), Yaw %f", current_position_.first, current_position_.second, current_yaw_);
    // }
 
    // vehicle x,y,yaw 데이터 콜백함수
    void vehiclestateCallback(const custom_msgs::vehicle_state::ConstPtr& msg){
        current_position_ ={msg->x, msg->y};
        current_yaw_ = msg->heading;
        ROS_INFO("Vehicle state received: x: %f ,y:%f, yaw:%f",msg->x, msg->y , msg->heading);
    }


    // Speedometer 데이터 콜백 함수
    void speedometerCallback(const carla_msgs::CarlaSpeedometer::ConstPtr& msg) {
        current_velocity_ = static_cast<double>(msg->speed); 
        ROS_INFO("Speedometer received: Speed %f", current_velocity_);
    }

    // ref_control 메시지 콜백 함수
    void refControlCallback(const custom_msgs::ref_control::ConstPtr& msg) {
        target_point_ = {msg->x, msg->y};
        target_velocity_ = msg->velocity;
        target_yaw_ = msg->yaw;
        ROS_INFO("Received ref_control: x=%f, y=%f, velocity=%f, yaw=%f", msg->x, msg->y, msg->velocity, msg->yaw);
    }

    // CTE 계산 함수 (현재 위치, 목표 지점, 목표 yaw를 사용하여 계산)
    double calculateCTE(const std::pair<double, double>& current_position, const std::pair<double, double>& target_point, double target_yaw) {
        // 목표 지점과 현재 위치 간의 벡터 계산
        double dx = target_point.first - current_position.first;
        double dy = target_point.second - current_position.second;

        // 목표 yaw 값으로 기울기 계산
        double a = std::tan(target_yaw); // 기울기
        double b = -1.0;
        double c = target_point.second - a * target_point.first; // 직선 방정식의 절편

        // CTE 계산 ,  점 직선 사이의 거리
        double cte = std::abs(a * current_position.first + b * current_position.second + c) / sqrt(a * a + b * b);

        // 외적으로 좌우 판단
        double cross_product = dx * std::sin(target_yaw) - dy * std::cos(target_yaw);
        if (cross_product < 0) {
            cte = -cte; // 왼쪽에 있으면 양수, 오른쪽에 있으면 음수
        }

        ROS_INFO("CTE calculated: %f", cte);
        return cte;
    }

    void calculateControl() {
        // if (target_point_ == std::pair<double, double>{0.0, 0.0}) {
        //     ROS_INFO("Target point is not set.");
        //     return;
        // }

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

        control_cmd.steer = delta / max_steering_angle_;  
        control_cmd.brake = 0.0;
        control_cmd.throttle = 0.37;  
        control_cmd.gear = 1;
        control_cmd.manual_gear_shift = false;
        control_cmd.reverse = false;
        control_cmd.header.stamp = ros::Time::now();
        control_pub_.publish(control_cmd);

        ROS_INFO("Control command published: Steer = %f", control_cmd.steer);
    }

private:
    ros::NodeHandle nh_;
    // ros::Subscriber odometry_sub_;
    ros::Subscriber speedometer_sub_; 
    ros::Subscriber vehicle_pose_sub_;
    ros::Subscriber ref_control_sub_;

    ros::Publisher control_pub_;

    std::pair<double, double> current_position_;
    double current_yaw_;
    double current_velocity_;
    carla_msgs::CarlaEgoVehicleControl control_cmd;
    std::pair<double, double> target_point_;  // 목표 지점 저장
    double target_velocity_;  // 목표 속도 저장
    double target_yaw_;  // 목표 yaw 저장
    
    double k_;
    double k_p_;
    double h_;
    double max_steering_angle_;
};

int main(int argc, char** argv) {
    // ROS 노드 초기화
    ros::init(argc, argv, "stanley_controller_node");

    // StanleyController 
    StanleyController controller;

    ros::Rate rate(40);  // 100Hz

    while (ros::ok()) {
        controller.calculateControl();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

