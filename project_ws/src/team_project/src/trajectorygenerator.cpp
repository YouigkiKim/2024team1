#include "trajectorygenerator.h"


namespace Plan{

    /*
    Trajectory Generator
    */
    TrajectoryGenerator::TrajectoryGenerator()
    : sf_(5.5), max_lateral_acc_(2.94), road_width_(2.75), length_ratio_(1.05), max_kappa_(0.3),path_num_(7){
        for(int i=1; i< 4;i++){
            std::cout << road_width_/3*i<<std::endl;
            y_offset_vector_.push_back(road_width_/3*i);
            y_offset_vector_.push_back(-road_width_/3*i);
        }
        y_offset_vector_.push_back(0.0);
    }
    //psi >> 도로와 차량의 오차 >> 도로의 theta?
    double TrajectoryGenerator::get_C1( const double& psi) { 
        yaw_e_ =psi; 
        return tan(psi); 
        }
    double TrajectoryGenerator::get_C2(const double& c1, const double& sf, const double& y_offset){
        return -2*c1/sf+3*y_offset/(sf*sf);
    }
    double TrajectoryGenerator::get_C3(const double& c1, const double& c2,const double&  sf){
        return -(c1+2*c2*sf)/(3*sf*sf);
    }
    //여기서의 yaw -> 차량기준
    void TrajectoryGenerator::update_generator_state(const double& x,const double&y,const double&yaw){
        ego_yaw_ = yaw;
        transform_mat_ <<   cos(-yaw),sin(-yaw) ,x,          
                            -sin(-yaw),cos(-yaw) ,y,
                            0, 0 ,1 ;
    }


    //트래젝토리 "들" 생성 -> x,y,vel만 남기면 됨 -> 곡선 계수들 남기는게 더 낫나? s,y,vel남기는게
    void TrajectoryGenerator::get_trajectories(const double& psi, const double& v0 ){
        // 트레젝토리 초기화
        while(!trajectories_.empty()){
            trajectories_.front().clear();
            trajectories_.pop();
        }
        // 트레젝토리 생성
        for (double y_offset : y_offset_vector_){
            // std::cout  << "y offset :  "<< y_offset<<std::endl;
            double sf = 5.5;
            std::vector<State> trajectory;
            std::vector<double> kappas;
            while(true){
                double c1 = get_C1(psi);
                double c2 = get_C2(c1,sf,y_offset);
                double c3 = get_C3(c1,c2,sf);
                kappas.clear();
                trajectory.clear();
                get_trajectory(trajectory,kappas,c1,c2,c3,sf,v0);
                if(std::max(abs(*std::max_element(kappas.begin(), kappas.end())), 
                            abs(*std::min_element(kappas.begin(), kappas.end()))) <= max_kappa_){
                    // std::cout <<"sf " << sf<< "kappas "<<kappas[0]<<" "<<kappas[*std::max_element(kappas.begin(), kappas.end())]<<std::endl;
                    // for(auto state : trajectory){
                    //     std::cout <<state.x_ << " "<<state.y_ << std::endl;
                    // }
                    if(sf <= v0*4){
                        extend_trajectory(trajectory,v0,sf);
                    }
                    trajectories_.push(trajectory);
                    break;
                }
                sf *= length_ratio_;
            }
        }
    }
    //트래젝토리 생성 >> 최대곡률 조건을 만족하는 트래젝토리를 반환
    void TrajectoryGenerator::get_trajectory(
            std::vector<State>& trajectory,std::vector<double>& kappas, const double& c1, const double&c2, const double& c3, double sf, const double& v0){
        std::vector<double> s;
        double s_gen = 0.0; 
        while(s_gen < sf){
            s.push_back(s_gen);
            // std::cout <<s_gen<<", ";
            s_gen+=0.3;
        }
        s.push_back(sf);
        // std::cout << "s spec "<< s[0] << " "<< *s.end()<<" "<<s.size()<<std::endl;
        // std::cout << std::endl;
        double x = 0;
        double s_prev = 0;
        double dx = 0;
        double l_prev = 0;
        double vel_prev = 0;
        for(double s_current : s ){
            double dlds = get_dlds(c1,c2,c3,s_current);
            double theta = atan(dlds);
            double ddlds = get_ddlds(c2,c3,s_current);
            double kappa = get_kappa(dlds,ddlds);
            kappas.push_back(kappa);
            double l = get_l(c1,c2,c3,s_current);
            dx = sqrt((s_current-s_prev)*(s_current-s_prev)-(l-l_prev)*(l-l_prev));
            l_prev = l;
            s_prev = s_current;
            State state;
            //transform to global
            Eigen::Vector3d vec;
            x +=dx;
            // x -= 1.45;
            vec << x,
                    l,
                    1; 
            vec = transform_mat_*vec;
            state.x_ = vec[0];
            state.y_ = vec[1];
            state.k_ = kappa;
            state.yaw_ = ego_yaw_+theta;
            // std::cout << "x y "<<state.x_ <<" "<<state.y_<<std::endl
            if(abs(kappa) < 0.003 ){
                state.vel_ = sqrt(abs(max_lateral_acc_ / kappa));
                if(abs(vel_prev - state.vel_) > max_long_acc_ )
                    state.vel_ = vel_prev + state.vel_/abs(state.vel_)*max_long_acc_;
            }else{
                state.vel_ = v0;
                if(abs(vel_prev - state.vel_) > max_long_acc_ ) 
                    state.vel_ = vel_prev + state.vel_/abs(state.vel_)*max_long_acc_;
                state.vel_ = 10/3.6;
            }
            vel_prev = state.vel_;
            trajectory.push_back(state);
            s_prev = s_current;
        }
    }
    void TrajectoryGenerator::extend_trajectory(std::vector<State>& trajectory,const double& v0,  double& sf){
        State state;
        state.vel_ = 5/3.6;
        while(sf < v0*3){
            
            Eigen::Vector3d vec;
            vec<< 0.3,
                    0,
                    0;
            vec = transform_mat_*vec;
            // std::cout << "transformed vector" << vec[0] << " "<< vec[1]<<std::endl;
            auto endpoint = trajectory.back();
            state.x_ = endpoint.x_ + vec[0];
            state.y_ = endpoint.y_ + vec[1];
            state.vel_ = std::min(state.vel_+1, 10/3.6);
            state.k_ = 0.0;
            sf += 0.3;
            trajectory.push_back(state);
        }
    }
    void TrajectoryGenerator::produce_trajectories(const double& x,const double& y,const double& yaw,const double& psi,const double& v0){
        update_generator_state(x,y,yaw);
        get_trajectories(psi,v0);
    }
    double TrajectoryGenerator::get_dlds(const double& c1,const double& c2, const double& c3, const double& s) {return c1+2*c2*s+3*c3*s*s;}
    double TrajectoryGenerator::get_ddlds(const double& c2, const double& c3, const double& s){return 2*c2+6*c3*s;}
    double TrajectoryGenerator::get_kappa(const double& dlds, const double& ddlds){
        double Q2 = dlds*dlds+1;
        double Q = sqrt(Q2);
        // std::cout << ddlds/Q2/Q<<std::endl;
        return ddlds/Q2/Q;
    }
    double TrajectoryGenerator::get_l(const double& c1,const double& c2,const double& c3,const double& s){
        return c1*s+c2*s*s+c3*s*s*s;
    }
}  // namespace Plan end

