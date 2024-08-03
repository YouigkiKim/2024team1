#ifndef PLANNER_H
#define PLANNER_H


#include <iostream>
#include <cmath>
#include <chrono>
#include <vector>
#include <queue>


namespace Plan{
    struct Object{
        double          x_;
        double          y_;
        Object(double x,double y):x_(x), y_(y){};
    };
    struct Ego{
        double          x_;
        double          y_;
        double          w_;
        double          vel_;
    };

    struct Objects{
        int             object_num_;
        std::vector<Object> objects_;
        Objects() : object_num_(0){};

        //test method
        void create_objects(){
            clear();
            objects_.push_back(Object(0,15));
            objects_.push_back(Object(-8,-10));
            objects_.push_back(Object(-6,20));
            objects_.push_back(Object(2,30));
            objects_.push_back(Object(5,-30));
            objects_.push_back(Object(10,-10));
            objects_.push_back(Object(-10,30));
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
        struct Pos{
            double x_;
            double y_;
            double yaw_;
        };
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
        //상황 별 flag
        enum class VelMode{
            LOW,
            HIGH,
            ACC
        };
        bool                    human_flag_;
        bool                    control_loss_flag_;
        bool                    traffic_sign_flag_;
        bool                    traffic_sign_; // Red true, green, yellow false
        //정속 속도
        double                  low_constant_speed_;
        double                  high_constant_speed_;
        // 감속 비율
        double                  control_loss_ratio_;
        double                  human_flag_ratio_;
        double                  traffic_sign_ratio_;
        double                  collision_time_ratio_;
        //급정거 threshold
        double                  hard_break_time_;
        //목표속도
        double                  ref_vel_;

        public:
        BehaviorPlan();
        
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