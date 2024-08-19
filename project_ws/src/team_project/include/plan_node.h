#pragma once

#ifndef PLAN_NODE_H
#define PLAN_NODE_H


#include "ros/ros.h"
#include <cmath>
//message header
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
#include <derived_object_msgs/ObjectArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Polygon.h>
#include <custom_msgs/ref_control.h>
#include <custom_msgs/vehicle_state.h>
#include <nav_msgs/Path.h>
#include <carla_msgs/CarlaSpeedometer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/MarkerArray.h>

//visualization header
#include <rviz_visual_tools/rviz_visual_tools.h>

//project header
#include "whole.h"
#include "planner.h"
#include "trajectorygenerator.h"
#include "collisionChecker.h"

namespace Plan{

    class PlanNode : public BehaviorPlan {
        public:
        PlanNode();
        PlanNode(ros::NodeHandle& nh);

        void publish();
        void visualization();
        //ros pub, sub
        ros::Publisher                          way_pub_ ;
        ros::Publisher                          vel_pub_ ;
        ros::Publisher                          ref_pub_ ;
        std::vector<ros::Publisher>             trajectory_pubs_;
        ros::Publisher                          collision_pub_;

        // std::vector<ros::Publisher>             trajectory_pub_;

        ros::Subscriber                         objects_sub_ ;
        ros::Subscriber                         pose_sub_ ;
        ros::Subscriber                         waypoint_sub_;
        ros::Subscriber                         speed_sub_;

        //ros msgs
        custom_msgs::ref_control               ref_msg_;
        std::vector<nav_msgs::Path>            path_msgs_;
        visualization_msgs::MarkerArray             collision_msg_;


        //visualization tools
        rviz_visual_tools::RvizVisualToolsPtr visual_tools_;


        //member
        Objects objects_;
        int path_num_;
        Pos ref_way_;
        //call back funstion
        void callback_objects(const derived_object_msgs::ObjectArray::ConstPtr& msg);
        // void regist_property_points(Object& object, const geometry_msgs::Polygon& msg);
        void callback_pose(const custom_msgs::vehicle_state::ConstPtr& msg);
        void callback_speed(const carla_msgs::CarlaSpeedometer::ConstPtr& msg);

        //publish_function  > 하나로 합칠까
        void publish_way();
        void publish_velocity();
        // void publish_trajectory();
        void publish_ref();
        void publish_collision();
    };
}

#endif // PLAN_NODE_H