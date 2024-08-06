#include "planner.h"


namespace Plan{
    Object::Object(double x,double y, std::string flag):x_(x), y_(y),flag_(flag){
            if(flag == "HUMAN" || flag == "BYCICLE") Flag Flag_ = Flag::HUMAN;
            else if(flag == "CONTROL_LOSS") Flag Flag_ = Flag::CONTROL_LOSS;
            else if(flag == "TRAFFIC_SIGN") Flag Flag_ = Flag::TRAFFIC_SIGN;
            else if(flag == "OPEN_DOOR") Flag Flag_ = Flag::OPEN_DOOR;
            else if(flag == "CUT_IN") Flag Flag_ = Flag::CUT_IN;
            else if(flag == "CONSTRUCTION_SITE") Flag Flag_ = Flag::CONSTRUCTION_SITE;
        };

    /*
    class planner
    */
    Planner::Planner(double x, double y, double yaw)
    : x_(x), y_(y), yaw_(yaw), vel_(5), collision_(3.0, 0.5, 1.0){
        std::string flag_("EGO");
        Object::Flag Flag_ = Object::Flag::EGO;
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
    void Planner::check_collision(){
        collision_.update_current_state(x_, y_, yaw_, vel_, w_);
        collision_.predict_ego();
        collision_time_ = collision_.collision_check(objects_);
        if(collision_time_ == -1.0){
            std::cout << "박치기 안일어남"<<std::endl;
        }else{
            std::cout << "박치기 " << collision_time_<<"초 후에 일어남"<< std::endl;
        }
    }

    /*
    collisionchecker
    */
    CollisionChecker::CollisionChecker(const double pt, const double dt, const double r)
        : predict_time_(pt), dt_(dt), circle_radius_(r){
            circles_pos.push_back(Object(1.0,0,"HUMAN"));
            // circles_pos.push_back(Object(-1.0,0));
            queue_size = predict_time_/dt_;
        };
    void CollisionChecker::update_current_state(double x, double y, double yaw, double v, double w){
        current_pos_.x_ = x;
        current_pos_.y_ = y;
        current_pos_.yaw_ = yaw;
        vel_ = v;
        w_ = w;
    }
    void CollisionChecker::update_current_state(Pos pos, double v, double w){
        current_pos_ = pos;
        vel_ = v;
        w_ = w;
    };
    void CollisionChecker::update_current_state(Object object,double yaw, double v, double w){
        current_pos_.x_ = object.x_;
        current_pos_.y_ = object.y_;
        current_pos_.yaw_ = yaw;
        vel_ = v;
        w_ = w;
    };


    void CollisionChecker::predict_ego(){
        while(!predicted_ego.empty()){
            predicted_ego.pop();
        }
        Pos pre_pos = current_pos_;
        Pos temp;
        for (int i=0;i<queue_size;i++){
            temp.yaw_ = pre_pos.yaw_+dt_*w_;
            temp.x_ = pre_pos.x_ + cos(temp.yaw_)*dt_*vel_;
            temp.y_ = pre_pos.y_ + sin(temp.yaw_)*dt_*vel_;
            predicted_ego.push(temp);
            pre_pos = temp;
            std::cout << "predicted after "<< dt_*(i+1)<<"  "<<temp.x_<<", "<<temp.y_<<std::endl;
        }
    }
    double CollisionChecker::collision_check(const Objects& objects){
        Pos current_step_pos;
        int step = 1;
        double d_x1;
        double d_y1;
        double d_x2;
        double d_y2;
        double range;
        double object_radius;
        while(!predicted_ego.empty()){
            current_step_pos = predicted_ego.front();
            
            // 이것도 이중 for문에 해당하나?
            for(const Object& object : objects.objects_){
                
                // 추가사항 - 객체별 radius설정 추가 필요
                if(true)object_radius = 1;
                else object_radius = 2;

                d_x1 = object.x_ - current_step_pos.x_ - circles_pos[0].x_;
                d_y1 = object.y_ - current_step_pos.y_ - circles_pos[0].y_;
                range = sqrt(d_x1*d_x1+d_y1*d_y1);


                //추가사항 - 리턴을 무엇을 할 것인지 결정 필요
                if(range < circle_radius_+object_radius){
                    std::cout << "박치기 일어남 (x,y)" << object.x_<<","<<object.y_<<std::endl;
                    return step*dt_;
                    break;
                }
            }
            step++;
            predicted_ego.pop();
        }
        return -1.0;
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
        if(waypoint_.get_opt() == LEFT_CORNER){
            corner_left_flag_ = true;
        }else if(waypoint_.get_opt() == RIGHT_CORNER){
            corner_right_flag_ = true;
        }
    }

    double BehaviorPlan::decide_vel(){
        if(human_flag_){
           return velcon_.human();
        }else if(control_loss_flag_){
            return velcon_.control_loss();
        }else if(traffic_sign_){
            return velcon_.traffic_sign();
        }else if(corner_left_flag_){
            return velcon_.corner_left();
        }else if(corner_right_flag_){
            return velcon_.corner_right();
        }else{
            return velcon_.normal(drive_);
        }
    }

    Pos decide_way(){

    }
    void BehaviorPlan::decide(){
        ref_vel = decide_vel();
        ref_way = decide_way();
    }
}