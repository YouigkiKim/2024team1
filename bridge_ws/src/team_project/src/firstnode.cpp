#include <iostream>
#include <ros/ros.h>

#include <std_msgs/Time.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <std_msgs/Header.h> 

#include <chrono>
#include <ctime>

int main(int argc, char **argv){
    ros::init(argc, argv, "first_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/hero/vehicle_control_cmd", 10);

    carla_msgs::CarlaEgoVehicleControl control_msg;
    ros::Rate rate(100.0);
    int count = 0;
    control_msg.manual_gear_shift = true;
    control_msg.gear = 1;
    control_msg.header.stamp = ros::Time::now();	
    control_msg.header.frame_id = "vehicle_control_msg";
    while(ros::ok()){
        if(1 ) {
            control_msg.throttle = 1.0; // 0~1까지
        }
        else{
            control_msg.throttle = 0.0;
        }
        count++;
        control_msg.header.stamp = ros::Time::now();
        pub.publish(control_msg);
        ROS_INFO("ROS TIME : %f",ros::Time::now().toSec());
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
