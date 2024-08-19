#pragma once

#ifndef PLANNER_H
#define PLANNER_H


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

#define LEFT_CORNER 1
#define RIGHT_CORNER 2
#define STRAIGHT 3


namespace Plan{
    struct Pos{
        double x_,y_,yaw_;
        Pos(){};
        Pos(double x, double y, double yaw)
        : x_(x), y_(y), yaw_(yaw){};
    };
    struct State : Pos{
        double vel_;
        double w_;
    };
    enum class Drive{
        HIGH,
        LOW,
    };
    struct Object{
        double          x_;
        double          y_;
        std::string     flag_;
        enum class      Flag{
            HUMAN,
            CONTROL_LOSS,
            TRAFFIC_SIGN,
            OPEN_DOOR,
            CUT_IN,
            CONSTRUCTION_SITE
        };
        Flag Flag_;
        Object();
        Object(double x,double y, std::string flag);
    };

    struct Objects{
        int             object_num_;
        std::vector<Object> objects_;
        Objects() : object_num_(0){};
        //test method >> ros msg형태에 맞춰 바꿔줘야함 아마 콜백함수가 될 예정
        void create_objects(){
            clear();
            objects_.push_back(Object(0,15,"HUMAN"));
            objects_.push_back(Object(-8,-10,"HUMAN"));
            objects_.push_back(Object(-6,20,"HUMAN"));
            objects_.push_back(Object(2,30,"HUMAN"));
            objects_.push_back(Object(5,-30,"HUMAN"));
            objects_.push_back(Object(10,-10,"HUMAN"));
            objects_.push_back(Object(-10,30,"HUMAN"));
            object_num_ = 7;
        }
        void clear(){
            object_num_ = 0;
            objects_.clear();
        }
        ~Objects(){}
    };



    class CollisionChecker{
        private:
        double          w_;
        double          dt_;
        double          vel_;
        double          predict_time_;
        double          collision_time_;
        double          circle_radius_;
        int             queue_size;
        Pos             current_pos_;
        std::vector<Object> circles_pos;
        std::queue<Pos> predicted_ego;

        public:
        CollisionChecker();
        CollisionChecker(const double pt, const double dt, const double r);
        void predict_ego();
        double collision_check(const Objects& Objects); // global 좌표 기준
        //callback에서 정보 받을 때 사용
        void update_current_state(const double x, const double y,const double yaw, const double v, const double w);
        void update_current_state(const Pos pos,const double v, const double w);
        void update_current_state(const Object object,const double yaw, const double v, const double w);
    };


    class Planner{

        public:
        Planner(){};
        Planner(double x, double y, double yaw);
        void placement_object(Objects& objects);
        void const print_objects();
        CollisionChecker collision_; 
        void check_collision();

        protected:
        double          x_;
        double          y_;
        double          yaw_;
        double          vel_;
        double          w_;

        private:
        double          collision_time_;
        Objects         objects_;
        Objects         localized_objects_;
        void rotation(Object& object);

    };

    /*
    Behavior plan
    */
    class BehaviorPlan : public Planner{ 
        private:
        //Behavior planning classes
        Drive                   drive_;
        WayPoint                waypoint_;
        VelocityControl         velcon_;

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
        void                    update_flag(const Objects& Objects);\
        void                    all_flag_false();
        double                  decide_vel();
        Pos                     decide_way();
        //control value
        double                  ref_vel;
        Pos                     ref_way;


        public:
        BehaviorPlan();
        void                    plan_global();

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
        struct Point : Pos{
            int opt;
        };

        void plan_local();
        std::queue<std::shared_ptr<Point>> global_path_;
        std::queue<std::shared_ptr<Point>> local_path_;
        void load_global(std::queue<std::shared_ptr<Point>>& global_path);
        void remove_passed_way(std::queue<std::shared_ptr<Point>> path);
        double dot_product_way(const Pos& pos, const std::shared_ptr<Point>& point);

        public:
        WayPoint();
        int get_opt();
        Pos get_waypoint();
        void print_waypoints();
        ~WayPoint(){};
    };


    

} // namespace plan end





#endif // PLANNER_H