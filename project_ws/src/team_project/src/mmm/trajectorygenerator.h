#pragma once

#ifndef TRAJECTORYGENERATOR_H
#define TRAJECTORYGENERATOR_H


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
#include <Eigen/Dense>



namespace Plan{
    // trajectory 후보들을 생성해 collision checker로 넘겨주는 클래스
    class TrajectoryGenerator{
        // protected:
        public:
        TrajectoryGenerator();
        void produce_trajectories(const double& x,const double& y,const double& yaw,const double& psi,const double& v0) ;
        int path_num_;
        std::queue<std::vector<State>> trajectories_;

        // private:
        double max_kappa_;
        std::vector<double> y_offset_vector_;
        std::vector<double> route_kappa_;
        std::vector<State> trajectory_;
        Eigen::Matrix3d transform_mat_;
        double sf_;
        double max_lateral_acc_;
        double max_long_acc_;
        double length_ratio_;
        double road_width_;
        double yaw_e_;
        double road_yaw_;
        double ego_yaw_;
        double get_C1( const double& psi);
        double get_C2(const double& c1,const double& sf,const double& y_offset);
        double get_C3(const double& c1, const double& c2,const double&  sf);
        double get_route(const double& y);
        double get_velocity(const double &y);
        void get_trajectory(std::vector<State>& trajectory,std::vector<double>& kappas, const double& c1, const double&c2, const double& c3, double sf, const double& v0);
        double get_dlds(const double& c1,const double& c2, const double& c3, const double& s);
        double get_ddlds(const double& c2, const double& c3, const double& s);
        double get_kappa(const double& dlds, const double& ddlds);
                    //각 스텝마다 필요한 인자들 : kappa -> dlds, ddlds^2,  lateral_offset -> c1,c2,c3,현재스텝의 s
        double get_l(const double & c1,const double & c2,const double & c3,const double & s);
        void update_generator_state(const double& x,const double&y,const double&yaw);
        void get_trajectories(const double& psi, const double& v0 );
        void extend_trajectory(std::vector<State>& trajectory,const double& v0,  double& sf);
    };



} //namespace Plan end





#endif // TRAJECTORYGENERATOR_H