#include "trajectorygenerator.h"


namespace Plan{

    /*
    Trajectory Generator
    */
    TrajectoryGenerator::TrajectoryGenerator()
    : sf_(5.5), max_lateral_acc_(2.94), road_width_(2.75), length_ratio_(1.05), max_kappa_(0.3){
        for(int i=0; i< 6;i++){
            y_offset_vector_.push_back(road_width_/3*i);
            y_offset_vector_.push_back(-road_width_/3*i);
        }
    }
    double TrajectoryGenerator::get_C1( const double& psi) { return tan(psi); }
    double TrajectoryGenerator::get_C2(const double& c1, const double& sf, const double& y_offset){
        return -2*c1/sf+3*y_offset/(sf*sf);
    }
    double TrajectoryGenerator::get_C3(const double& c1, const double& c2,const double&  sf){
        return -(c1+2*c2*sf)/(3*sf*sf);
    }
    double TrajectoryGenerator::rotation_to_global(){}
    void TrajectoryGenerator::update_generator_state(const double& x,const double&y,const double&yaw){
        transform_mat_ <<   cos(yaw),sin(yaw) ,x, 
                            -sin(yaw),cost(yaw) ,y,
                            0,0 ,1 ;
    }


    //트래젝토리 "들" 생성 -> x,y,vel만 남기면 됨
    void TrajectoryGenerator::get_trajectories(const double& psi, const double& v0 ){
        // 트레젝토리 초기화
        while(trajectories_.empty()){
            trajectories_.pop();
        }
        // 트레젝토리 생성
        for (int i=0; i< y_offset_vector_.size();i++){
            double sf = 5.5;

            double c1 = get_C1(psi);
            double c2 = get_C2(c1,sf,y_offset_vector_[i]);
            double c3 = get_C3(c1,c2,sf);
            //각 횡방향 오프셋에 알맞은 트레젝토리 생성 후 push_back
            get_trajectory(c1,c2,c3,sf,v0);
            // while(true){
                
                
            //     if( *std::max_element(route_kappa_.begin(), route_kappa_.end()) < max_kappa_){
            //         trajectories_.push(trajectory);
            //         break;
            //     }
            //     sf*length_ratio_;
            // }
        }
    }
    //트래젝토리 생성 >> 최대곡률 조건을 만족하는 트래젝토리를 반환
    void TrajectoryGenerator::get_trajectory(
            const double& c1, const double&c2, const double& c3, const double& sf, const double& v0, const double& yaw){
        std::vector<State> trajectory;
        std::vector<double> kappas;        
        // 0.5곡선길이단위로 s 생성
        double s_gen;
        std::vector<double> s;
        while(s_gen < sf){
            s.push_back(s_gen);
            s_gen+0.5;
        }
        s.push_back(sf);

        //while문 조건은 true로 설정하고 if문을 사용해 break
        while(true){
            //각 s마다 횡방향오프셋, kappa, velocity  구하는 루프
            //각 스텝마다 필요한 인자들 : kappa -> dlds, ddlds^2,  lateral_offset -> c1,c2,c3,현재스텝의 s
            for(double s_current : s ){
                double dlds = get_dlds(c1,c2,c3,s_current);
                double theta = atan(dlds);
                double ddlds = get_ddlds(c2,c3,s_current);
                double kappa = get_kappa(dlds,ddlds);

                State state;
                //transform to global
                Eigen::Vector3d vec;
                vec << s_current*cos(-theta),
                       s_current*sin(-theta),
                       0; 
                vec = transform_mat_*vec;
                state.x_ = vec[0];
                state.y_ = vec[1];
                if(kappa != 0 ){
                    state.vel_ = sqrt(max_lateral_acc_ / kappa);
                    if(state.vel_ > 20) state.vel_ = 20;
                }else{
                    state.vel_ = 20;
                }
                kappas.push_back(kappa);
                trajectory.push_back(state);
            }
            //트래젝토리에 대한 곡률조건 검사
            //조건에 맞으면 while문 중단
            if(kappas[*std::max_element(kappas.begin(), kappas.end())] < max_lateral_acc_){
                trajectories_.push(trajectory);
                break;
            }
        }
    }

    double TrajectoryGenerator::get_dlds(const double& c1,const double& c2, const double& c3, const double& s) {return c1+2*c2*s+3*c3*s*s;}
    double TrajectoryGenerator::get_ddlds(const double& c2, const double& c3, const double& s){return 2*c2+6*c3*s;}
    double TrajectoryGenerator::get_kappa(const double& dlds, const double& ddlds){
        double Q2 = dlds*dlds+1;
        double Q = sqrt(Q2);
        return ddlds/Q2/Q;
    }
    void TrajectoryGenerator::rotation_to_global(double&x, double& y){

    }







}  // namespace Plan end

