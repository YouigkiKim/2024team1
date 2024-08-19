#include "trajectorygenerator.h"
#include "collisionChecker.h"
#include <custom_msgs/vehicle_state.h>
#include <geometry_msgs/Pose.h>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

// 의존성 추가해야되는 메세지들
#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Quaternion.h>

double x,y,yaw;
void callback_pose(const custom_msgs::vehicle_state::ConstPtr& msg){
    x = msg -> x;
    y = msg -> y;
    yaw = msg -> heading;

    // By Custom Message
    //x_ = msg -> x;
    //y_ = msg -> y;
    //yaw_ = msg -> heading;

    // ROS_INFO("by pose           %f %f ", x_,y_);
}
int main(int argc, char** argv){

    ros::init(argc, argv, "trajectory_node");
    ros::NodeHandle nh;

    double v0 = 10;
    bool initialize = false;
    nav_msgs::Path path_msg;
    ros::Subscriber pose_sub = nh.subscribe<custom_msgs::vehicle_state>("carla/hero/pose",10,&callback_pose);
    Plan::CollisionChecker generator;
    ros::Rate rate(10);  // 100Hz
    std::vector<ros::Publisher> publishers;
    std::vector<nav_msgs::Path> msgs;
    while (ros::ok()) {
        generator.update_generator_state(x,y,yaw);
        generator.get_trajectories(0,v0);
        

        if(!initialize){
            for(int i=0; i<generator.trajectories_.size();i++){
                std::string topic_name = "/plan/path" + std::to_string(i); // 고유한 토픽 이름
                ros::Publisher trajectory_pub = nh.advertise<nav_msgs::Path>(topic_name,10);
                publishers.push_back(trajectory_pub);
                nav_msgs::Path path_msg;
                path_msg.header.frame_id = "map"; // 필요한 경우 프레임 설정
                msgs.push_back(path_msg);
            }
            initialize = true;
        }
        int i=0;
        geometry_msgs::PoseStamped temp;

        tf2::Quaternion q;
        while(!generator.trajectories_.empty()){
            auto& path=generator.trajectories_.front();
            msgs[i].poses.clear();
            for(auto state: path){
                temp.pose.position.x = state.x_;
                temp.pose.position.y = state.y_;
                q.setRPY(0,0,state.yaw_);
                temp.pose.orientation.x = q.x();
                temp.pose.orientation.y = q.y();
                temp.pose.orientation.z = q.z();
                temp.pose.orientation.w = q.w();
                msgs[i].poses.push_back(temp);
            }
            generator.trajectories_.pop();
            i++;
        }
        for(int i=0;i< msgs.size();i++ ){
            publishers[i].publish(msgs[i]);
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}