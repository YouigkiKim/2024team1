#include "whole.h"


namespace Plan{
    // Pos rotation(Pos& Ego, Pos& object){
    //     Pos temp;
    //     temp.x_ = cos(Ego.yaw_)*(Ego.x_ - object.x_)
    //     return temp
    // }


    /*
    class planner
    */
    Planner::Planner(double x, double y, double yaw)
    : x_(x), y_(y), yaw_(yaw), vel_(5){
        // std::string flag_("EGO"); f
        // Object::Flag Flag_ = Object::Flag::EGO;
    };

    // objects 객체를 직접 localization하는 코드
    void Planner::placement_object (Objects& objects){
        objects_.clear();
        for(int i=0; i< objects.object_num_;i++){
            //global object
            objects_.objects_.push_back(objects.objects_[i]);
            objects_.object_num_ ++;

            //localization
            objects.objects_[i].x_ -= x_;
            objects.objects_[i].y_-=y_;
            rotation(objects.objects_[i]);
            localized_objects_.objects_.push_back( objects.objects_[i]);
            localized_objects_.object_num_++;
        }
    }

    void Planner::rotation(Object& object){
        object.x_ = object.x_*sin(yaw_) - object.y_*cos(yaw_);
        object.y_ = object.x_*cos(yaw_) + object.y_*sin(yaw_);
    }
    
    void const Planner::print_objects(){
        std::cout << "num of objects :          "<<objects_.object_num_<<std::endl;
        std::cout << "current car position :    ("<<x_<<", "<<y_<<", "<<yaw_<<")"<<std::endl;
        for(int i=0;i<objects_.object_num_;i++){
            std::cout << "global (x,y) "<< "("<<objects_.objects_[i].x_<<", "
            <<objects_.objects_[i].y_<<")"<<std::endl;
        }
    }




    /*
    behavior plan
    */
    BehaviorPlan::BehaviorPlan()
        {all_flag_false();}

    void BehaviorPlan::all_flag_false(){
        human_flag_ = false;
        control_loss_flag_ = false;
        traffic_sign_flag_ = false;
        traffic_sign_ = false;
        open_door_flag_ = false;
        construction_flag_ = false;
    }
    void BehaviorPlan::plan_global(){};

    void BehaviorPlan::update_flag(const Objects& Objects){
        all_flag_false();
        for(int i=0; i< Objects.object_num_;i++){
            switch(Objects.objects_[i].Flag_){
                case Object::Flag::HUMAN:
                    human_flag_ = true;
                    break;
                case Object::Flag::CONTROL_LOSS:
                    control_loss_flag_ = true;
                    break;
                case Object::Flag::CONSTRUCTION_SITE:
                    construction_flag_ = true;
                    break;
                case Object::Flag::TRAFFIC_SIGN:
                    traffic_sign_ = true;
                    break;
                case Object::Flag::OPEN_DOOR:
                    break;
                case Object::Flag::CUT_IN:
                    break;
            }
        }
        if(get_opt() == LEFT_CORNER){
            corner_left_flag_ = true;
        }else if(get_opt() == RIGHT_CORNER){
            corner_right_flag_ = true;
        }
    }

    // double BehaviorPlan::decide_vel(){
    //     if(human_flag_){
    //        return human();
    //     }else if(control_loss_flag_){
    //         return control_loss();
    //     }else if(traffic_sign_){
    //         return traffic_sign();
    //     }else if(corner_left_flag_){
    //         return corner_left();
    //     }else if(corner_right_flag_){
    //         return corner_right();
    //     }else{
    //         return normal(drive_);
    //     }
    //     return -1;
    // }

    /*
    VelocityControl
    */
   /*
    고려해야되는 상황
    object  많을 때 > 고속
            적을 때 > 저속
    acc  -> 센터 트래잭토리의 ttc로 계산 -> 일정 수준의 ttc 유지하게 제어?
    신호등 -> threshold픽셀에서 멈추기 -> 제동거리 생각해서 ref와 선형 or exponential로 잇기 -> exponential이 좋을 것 같음
    aeb  -> 사람플래그 + 가운데 3개 트레젝토리 중 충돌판단 일어났을 때
            센터 트레젝토리 ttc 1초 미만 -> ref vel = 0으로 전달
   */
    VelocityControl::VelocityControl()
        : ref_low_(20/3.6), ref_high_(40/3.6), 
        control_loss_ratio_(0.7),human_flag_ratio_(0.5),traffic_sign_ratio_(1),
        hard_break_time_(1.0), left_corner_ratio_(0.6), right_corner_ratio_(0.75),
        acc_time_(2.5), safe_distance_(3){
            ref_current_ = ref_low_;
    }
    double VelocityControl::human()         {   return ref_current_*human_flag_ratio_; }
    double VelocityControl::control_loss()  {   return ref_current_*control_loss_ratio_;}
    double VelocityControl::traffic_sign(){
        int dpixel = traffic_sign_goal_fixel_ -  traffic_sign_current_fixel_;
        return std::max( std::min(ref_current_, static_cast<double>(dpixel)/50-1), 0.0 );
    }
    double VelocityControl::corner_left(){
        if(ref_current_ == ref_high_)   return ref_current_ * left_corner_ratio_;
        else                            return ref_current_;
    }
    double VelocityControl::corner_right()  {return ref_low_*right_corner_ratio_;}

    double VelocityControl::normal(const Drive& drive){
        switch(drive){
            case(Drive::HIGH):
                ref_current_ = ref_high_;
                return ref_current_;
            case(Drive::LOW):
                ref_current_ = ref_low_;
                return ref_current_;
            default:
                return ref_current_;
        }
    }

    double VelocityControl::decide_vel(const std::vector<std::vector<State>>& trajectories, 
        const std::vector<Result>& collision_result, const double& current_vel, const double& predict_time){

        double velocity = ref_current_;

        if(collision_result[6].collision_time != predict_time) velocity = std::min(velocity, get_acc_velocity(collision_result, current_vel));

        if(aebflag(trajectories, collision_result, predict_time )) velocity = 0.0;
        return velocity;
    }
    bool VelocityControl::aebflag(const std::vector<std::vector<State>>& trajectories, 
        const std::vector<Result>& collision_result, const double& predict_time){
        if( object_flag_.human && (collision_result[6].collision_time != predict_time
             || collision_result[1].collision_time != predict_time
             || collision_result[2].collision_time != predict_time) )return true;
        // if(조건들 추가하기)
        return false;
    }
    double VelocityControl::get_acc_velocity(const std::vector<Result>& collision_result, const double& current_vel){
        double acc_velocity = current_vel;
        double distance = collision_result.back().collision_time * current_vel;
        acc_velocity = (distance-safe_distance_) / acc_time_;
        if(acc_velocity < 0.0) acc_velocity = 0.0;
        return acc_velocity;
    }
    /*
    WayPoints method
    */
    WayPoint::WayPoint(){
        local_path_.empty();
        global_path_.empty();
        load_global(global_path_);
        std::cout <<"waypoint class initialized"<<std::endl;
    }

    int WayPoint::get_opt(){
        return global_path_.front() -> opt;
    }

    void WayPoint::load_global(std::queue<std::shared_ptr<Point>>& global_path){
        std::queue<std::shared_ptr<Point>> waypoints;
        std::ifstream file("/home/ailab/carla-ros-bridge/project_ws/src/team_project/src/waypoint_lla_opt_output.txt");

        if( !file.is_open()){ std::cout << "ERORR :  Load Waypoints Fail   ======"<<std::endl;}
        std::string line;
        while(std::getline(file,line)){
            std::istringstream iss(line);
            auto waypoint = std::make_shared<Point>();
            if(iss >> waypoint -> x_ >>  waypoint -> y_  >> waypoint -> yaw_ >> waypoint -> opt){
                // std::cout << waypoint -> x_ << " " << waypoint -> y_ <<" "<< waypoint -> yaw_<<" " <<waypoint -> opt<<std::endl;
                global_path.push(waypoint);
            }else std::cout << "parsing fail" << std::endl;
        }
        std::cout << "Global Path Load"<<std::endl;
        // std::cout << global_path_.front() -> x_ << " "<<global_path_.front() -> y_ << " "<<global_path_.front() -> yaw_ << std::endl;
    }
    void WayPoint::remove_passed_way(std::queue<std::shared_ptr<Point>>& path,const double& x, const double& y){
        if(path.empty()) return; 
        // std::cout << dot_product_way(Pos(x,y,0), path) << " dot Product"<<std::endl;
        while( dot_product_way(Pos(x,y,0), path) < 0 ){
            prev_point_.x_ = path.front() -> x_ ;
            prev_point_.y_ = path.front() -> y_ ;
            // //debug code
            // std::cout << prev_point_.x_ << " "<< prev_point_.y_ << " popped";
            path.pop();
        }
    }

    double WayPoint::dot_product_way(const Pos& pos, std::queue<std::shared_ptr<Point>>& path){
        return cos((path.front() -> yaw_)*M_PI/180 ) * ( path.front() -> x_ - pos.x_ ) + 
            sin((path.front() -> yaw_)*M_PI/180) * ( path.front() -> y_ - pos.y_ );
    }

    Pos WayPoint::get_waypoint(const double& x, const double& y){
        if(! local_path_.empty()){
            remove_passed_way(local_path_, x,y);
            if(! local_path_.empty()){
                Pos temp(local_path_.front() -> x_, local_path_.front() -> y_, way_direction_);
                return temp;
                }
            else {
                Pos temp(global_path_.front() ->x_,global_path_.front() ->y_,  way_direction_);
                return temp;
            }
            return Pos(0,0,0);
        }

        remove_passed_way(global_path_,x,y);

        Pos temp(global_path_.front() ->x_,global_path_.front() ->y_,  way_direction_ );
        get_direction(global_path_);
        // std::cout << "get dy dx , create temp"<< std::endl;

        // double dx = temp.x_ - x;
        // double dy = temp.y_ - y;
        // double length = sqrt(dx*dx+dy*dy);
        // if(length >1.0){
        //     temp.x_ = x+dx/length;
        //     temp.y_ = y+dy/length;
        // }
        // std::cout << "return temp "<< std::endl;
        return temp;
    }
    //디버깅 후 temp지우기
    void WayPoint::get_direction(std::queue<std::shared_ptr<Point>>& path){
        way_direction_ =  std::atan2( path.front()->y_ - prev_point_.y_, path.front()->x_ - prev_point_.x_);
    }
}