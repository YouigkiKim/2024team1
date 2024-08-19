#pragma once

#ifndef WHOLE_H
#define WHOLE_H


#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <chrono>
#include <vector>
#include <queue>
#include <string>
#include <algorithm>
#include <memory>
#include "planner.h"
#include "collisionChecker.h"

#define LEFT_CORNER 1
#define RIGHT_CORNER 2
#define STRAIGHT 3


namespace Plan{



    class Planner{

        public:
        Planner(){};
        Planner(double x, double y, double yaw);
        void placement_object(Objects& objects);
        void const print_objects();
        void check_collision();

        protected:
        double          x_;
        double          y_;
        double          yaw_;
        double          vel_;
        double          w_;
        Objects         objects_;
        Objects         localized_objects_;
        void rotation(Object& object);
    };

    class VelocityControl{
        private:
        //정속 속도
        double                  ref_low_, ref_high_;
        double                  ref_current_;
        // 감속 비율
        double                  control_loss_ratio_;
        double                  human_flag_ratio_;
        double                  traffic_sign_ratio_;
        double                  collision_time_ratio_;
        double                  left_corner_ratio_, right_corner_ratio_;
        //급정거 threshold
        double                  hard_break_time_;
        //traffic sign 높이
        int                     traffic_sign_goal_fixel_;
        int                     traffic_sign_current_fixel_;
        
        public:
        VelocityControl();
        double human();
        double control_loss();
        double traffic_sign();
        double corner_left();
        double corner_right();
        double normal(const Drive& drive);
    };


    class WayPoint{
        public:
        struct Point : Pos{
            int opt;
        };

        WayPoint();
        int get_opt();
        Pos get_waypoint(const double& x, const double& y);
        void get_direction(std::queue<std::shared_ptr<Point>>& path);
        void print_waypoints();
        ~WayPoint(){};

        private:
        void plan_local();
        std::queue< std::shared_ptr<Point> > global_path_;
        std::queue< std::shared_ptr<Point> > local_path_;
        void load_global(std::queue< std::shared_ptr<Point> >& global_path);
        void remove_passed_way(std::queue<std::shared_ptr<Point>>& path, const double& x, const double& y);
        double dot_product_way(const Pos& pos, std::queue<std::shared_ptr<Point>>& path );
        Point prev_point_;
        double way_direction_;
    };

    /*
    Behavior plan
    */
    class BehaviorPlan :public VelocityControl, public Planner, public WayPoint, public CollisionChecker{ 
        protected:
        //Behavior planning classes
        Drive                   drive_;

        // objects flags
        bool                    human_flag_;
        bool                    control_loss_flag_;
        bool                    traffic_sign_flag_;
        bool                    traffic_sign_; // Red true, green, yellow false
        bool                    open_door_flag_;
        bool                    construction_flag_;
        bool                    corner_left_flag_;
        bool                    corner_right_flag_;

        //BehaviorPlan private function
        void                    update_flag(const Objects& Objects);
        void                    all_flag_false();
        double                  decide_vel();
        Pos                     decide_way();
        //control value
        double                  ref_vel;
        Pos                     ref_way;
        //ref value
        State                   ref_;

        public:
        BehaviorPlan();
        void                    plan_global();

    };
 

    

} // namespace plan end





#endif // WHOLE_H