#ifndef PLANNER_H
#define PLANNER_H


#include <iostream>
#include <cmath>
#include <chrono>
#include <vector>
#include <queue>
#include <string>
#include <decision.h>
using Decision::VelocityControl ;
using Decision::WayPoint ;

#define LEFT_CORNER 2
#define RIGHT_CORNER 3

namespace Plan{
    struct Object{
        double          x_;
        double          y_;
        std::string     flag_;
        enum class      Flag{
            EGO,
            HUMAN,
            CONTROL_LOSS,
            TRAFFIC_SIGN,
            OPEN_DOOR,
            CUT_IN,
            CONSTRUCTION_SITE
        };
        Flag Flag_;
        Object(double x,double y, std::string flag);
    };
    struct Pos{
        double x_,y_,yaw_;
        Pos(double x, double y, double yaw)
        : x_(x), y_(y), yaw_(yaw){};
    };
    enum class Drive{
        HIGH,
        LOW,
    };

    struct Objects{
        int             object_num_;
        std::vector<Object> objects_;
        Objects() : object_num_(0){};

        //test method
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
        CollisionChecker(const double pt, const double dt, const double r);
        void predict_ego();
        double collision_check(const Objects& Objects); // global 좌표 기준
        void update_current_state(const double x, const double y,const double yaw, const double v, const double w);
        void update_current_state(const Pos pos,const double v, const double w);
        void update_current_state(const Object object,const double yaw, const double v, const double w);

    };

    /*
    Behavior plan
    */
    class BehaviorPlan{
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
        void                    decide();

    };





    class Planner{

        public:
        Planner(double x, double y, double yaw);
        void placement_object(Objects& objects);
        void const print_objects();
        CollisionChecker collision_; 
        void check_collision();


        private:
        double          x_;
        double          y_;
        double          yaw_;
        double          vel_;
        double          w_;
        double          collision_time_;
        Objects         objects_;
        Objects         localized_objects_;
        void rotation(Object& object);

    };



} // namespace plan end

#endif // PLANNER_H