#ifndef DECISION_H
#define DECISION_H


#include <planner.h>
using Plan::Pos;
using Plan::Drive;
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <chrono>
#include <vector>
#include <queue>
#include <string>
#include <decision.h>
#include <algorithm>
#include <memory>


namespace Decision{

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
        double normal(Drive drive);

    };


    class WayPoint{
        struct Point : Pos{
            int opt;
        };

        void plan_local();
        std::queue<std::unique_ptr<Point>> global_path_;
        std::queue<std::unique_ptr<Point>> local_path_;
        void load_global(std::queue<std::unique_ptr<Point>>& global_path);
        void remove_passed_way(const Pos& pos);
        double dot_product_way(const Pos& pos, const std::unique_ptr<Point>& point);
        

        public:
        WayPoint();
        int get_opt();
        Pos get_waypoint(const Pos& pos);
        ~WayPoint();
    };


    
}


#endif // DECISION_H