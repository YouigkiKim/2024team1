# include "decision.h"
#include <algorithm>

namespace Decision{

    VelocityControl::VelocityControl()
        : ref_low_(15), ref_high_(40), 
        control_loss_ratio_(0.7),human_flag_ratio_(0.5),traffic_sign_ratio_(1),
        hard_break_time_(2.0), left_corner_ratio_(0.6), right_corner_ratio_(0.75){
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

    double VelocityControl::normal(Drive drive){
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


    /*
    WayPoints method
    */
    WayPoint::WayPoint(){
        load_global(global_path_);
    }

    int WayPoint::get_opt(){return global_path_.front() -> opt;}

    void WayPoint::load_global(std::queue<std::unique_ptr<Point>>& global_path){
        std::queue<std::unique_ptr<Point>> waypoints;
        std::ifstream file("waypoint_xy_yaw.txt");

        if( !file.is_open()){ std::cout << "ERORR :  Load Waypoints Fail   ======"<<std::endl;}
        std::string line;
        while(std::getline(file,line)){
            std::istringstream iss(line);
            auto waypoint = std::make_unique<Point>();
            if(iss >> waypoint -> x_ >> waypoint -> y_  >> waypoint -> yaw_ >> waypoint -> opt){
                global_path.push(waypoint);
            }else std::cout << "parsing fail" << std::endl;
        }
    }
    void WayPoint::remove_passed_way(const Pos& pos){
        while(dot_product_way(pos, global_path_.front()) < 0){
            global_path_.pop();
            if(global_path_.empty()) break; 
        }
    }
    double WayPoint::dot_product_way(const  Pos& pos,const std::unique_ptr<Point>& point){
         return cos(point ->yaw_)*pos.x_+sin(point -> yaw_) * pos.y_ ;
    }
    Pos WayPoint::get_waypoint(const Pos& pos){
        if(! local_path_.empty()){
            while(dot_product_way(pos, local_path_.front()) < 0.0){
                local_path_.pop();
                if(local_path_.empty()) break; 
            }
            if(! local_path_.empty()) {
                Pos temp(local_path_.front() -> x_, local_path_.front() -> y_, local_path_.front() -> yaw_);
                return temp;
                }
            else {
                Pos temp(global_path_.front() ->x_,global_path_.front() ->y_,global_path_.front() ->yaw_);
                return temp;
            }

        }
    }
}  // namespace Decision end


