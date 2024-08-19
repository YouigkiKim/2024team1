
#pragma once
#ifndef COLLISIONCHECKER_H
#define COLLISIONCHECKER_H
#include "planner.h"
#include "trajectorygenerator.h"
#include <rviz_visual_tools/rviz_visual_tools.h>

namespace Plan{
    class CollisionChecker : public TrajectoryGenerator{
        private:

        double                                      dt_;
        double                                      predict_time_;
        double                                      collision_time_;
        double                                      circle_radius_;
        double                                      x_offset_;
        int                                         predict_step_; //predict step
        Pos                                         current_pos_;
        Object                                      ego_;
        std::vector<Object>                         circles_pos;
        Circle                                      ego_start_circle_;
        Circle                                      ego_circle_;
        std::vector<Circle>                         circles_;
        std::vector<Pos>                            predicted_ego;
        std::vector<std::vector<Pos>>               predicted_egos;

        void draw_circles(const Objects& objects, const double& ego_yaw);
        void draw_circle(Circle& circle, const Object& object, const double& ego_yaw);
        void draw_ego_circle(const Object& ego);
        void predict_ego(const Pos& start_pose, const double& velocity, const double& ang_vel); 
        void predict_egos(std::queue<std::vector<State>>& trajectories, Pos& ego_pos) ;
        void collision_check(const std::vector<Circle>& circles); // global 좌표 기준
        //callback에서 정보 받을 때 사용
        void update_current_state(const double x, const double y,const double yaw, const double v, const double w);
        void update_current_state(const Pos pos,const double v, const double w);
        void update_current_state(const Object object,const double yaw, const double v, const double w);
        void update_collision_state(const double& x,const double& y ,const double&,const double& yaw);
        public:
        CollisionChecker();
        CollisionChecker(const double pt, const double dt, const double r);
        void check_collision(const Objects& objects,const double& x,const double& y,const double& yaw,  const double& yaw_error, const double& velocity);
        void visualization_circles(const rviz_visual_tools::RvizVisualToolsPtr& visual_tools);
        void visualization_collision(const rviz_visual_tools::RvizVisualToolsPtr& visual_tools);
        std::vector<Result>                 result_;
        std::vector<std::vector<State>>     after_trajectories_;
    };
}



#endif //COLLISIONCHECKER_H