#include "collisionChecker.h"


namespace Plan{
    /*
    collisionchecker
    */
    CollisionChecker::CollisionChecker()
    : predict_time_(3.0), dt_(0.5){
        predict_step_ = predict_time_/dt_;
        ego_.width_ = 1.896;
        ego_.length_ = 4.93;
    };
    CollisionChecker::CollisionChecker(const double pt, const double dt, const double r)
        : predict_time_(pt), dt_(dt), circle_radius_(r){
            // circles_pos.push_back(Object(-1.0,0));
            predict_step_ = predict_time_/dt_;
    };
    void CollisionChecker::update_current_state(double x, double y, double yaw, double v, double w){
        current_pos_.x_ = x;
        current_pos_.y_ = y;
        current_pos_.yaw_ = yaw;
    }
    void CollisionChecker::update_current_state(Pos pos, double v, double w){
        current_pos_ = pos;

    }
    void CollisionChecker::update_current_state(Object object,double yaw, double v, double w){
        current_pos_.x_ = object.x_;
        current_pos_.y_ = object.y_;
        current_pos_.yaw_ = yaw;
    }

    void CollisionChecker::draw_circles(const Objects& objects, const double& ego_yaw){
        circles_.clear();
        for( auto object: objects.objects_){
            Circle circle;
            circle.x_= object.x_;
            circle.y_= object.y_;
            draw_circle(circle, object,ego_yaw);
            circles_.push_back(circle);
        }
    }
    //object기준으로 circle그리기 trajectory가 들어오고 -> predict_ego하고 -> collision check하는 순서 -> 충돌시간 충돌위치 반환,
    void CollisionChecker::draw_circle(Circle& circle, const Object& object, const double& ego_yaw){
        if(object.length_ > 0.8 || object.width_ > 0.8 ){
            // two circle 
            double x_offset = object.length_/3;
            circle.radius = sqrt(0.25*object.width_*object.width_+ x_offset*x_offset*0.25);
            double dx = x_offset*cos(-ego_yaw);
            double dy = x_offset*sin(-ego_yaw);
            Position temp1, temp2;
            temp1.x_ =  object.x_ + dx;
            temp1.y_ = object.y_ + dy;
            temp2.x_ = object.x_-dx;
            temp2.y_ = object.y_-dy;
            circle.centerpoints_.push_back(temp1);
            circle.centerpoints_.push_back(temp2);
            std::cout <<"circle1 position x,y : "<< temp1.x_ <<" "<<temp1.y_<<std::endl;
            std::cout <<"circle2 position x,y : "<< temp2.x_ <<" "<<temp2.y_<<std::endl;
        }else{
            Position temp;
            temp.x_ = object.x_;
            temp.y_ = object.y_;
            if(object.width_ > object.length_){
                circle.radius = object.width_*0.5;
            }else{
                circle.radius = object.length_*0.5;
            }
            circle.centerpoints_.push_back(temp);
        }
    }
    void CollisionChecker::predict_egos(std::queue<std::vector<State>>& trajectories, Pos& ego_pos){
        for(auto temp : predicted_egos){
            temp.clear();
        }
        predicted_egos.clear();
        int trajectory_num = 0;
        while( !trajectories.empty()){
            int idx = 0;
            std::vector<State>& current_trajectory = trajectories.front();
            predicted_ego.clear();
            std::cout <<  current_trajectory[0].x_<< " "<<current_trajectory[0].y_<< std::endl;
            after_trajectories_.push_back(trajectories.front());
            for(int i=0;i<predict_step_;i++){
                double ds = current_trajectory[idx+1].vel_*dt_;
                int trajectory_step = static_cast<int>(std::round(ds/0.3));
                idx +=trajectory_step;
                if(idx > current_trajectory.size() || idx < 0) idx = current_trajectory.size() -1;
                std::cout <<"ds " <<ds<<" velocity "<<current_trajectory[idx+1].vel_<<" trajectory step "<<trajectory_step<<std::endl;

                State predict_point = trajectories.front()[idx];
                predicted_ego.push_back(predict_point);
                std::cout << "trajectory : "<<trajectory_num << " predict x,y : "<<  predict_point.x_<<" "<<predict_point.y_<< "idx " << idx
                <<" trajectory size " << current_trajectory.size()<<std::endl;
                std::cout << std::endl;
            }
            predicted_egos.push_back(predicted_ego);
            trajectories.pop();
            trajectory_num ++;
        }
    };
    void CollisionChecker::predict_ego(const Pos& start_pose, const double& velocity,const double& ang_vel){
        predicted_ego.clear();
        Pos prev_pose = start_pose;
        Pos current_pos;
        for (int i=0;i<predict_step_;i++){
            current_pos.yaw_ = prev_pose.yaw_+dt_*ang_vel;
            current_pos.x_ = prev_pose.x_ + cos(current_pos.yaw_)*dt_*velocity;
            current_pos.y_ = prev_pose.y_ + sin(current_pos.yaw_)*dt_*velocity;
            predicted_ego.push_back(current_pos);
            prev_pose = current_pos;
            std::cout << "predicted after "<< dt_*(i+1)<<"  "<<current_pos.x_<<", "<<current_pos.y_<<std::endl;
        }
    }

    void CollisionChecker::collision_check(const std::vector<std::vector<Pos>>& predicted_egos,const std::vector<Circle>& circles){
        result_.clear();
        bool break_flag = false;
        for(auto ego_poses : predicted_egos){
            int i=0;
            break_flag = false;
            for(auto ego_pose : ego_poses){ 
                ego_.x_ = ego_pose.x_;
                ego_.y_ = ego_pose.y_;
                ego_.yaw_ = ego_pose.yaw_;
                draw_ego_circle(ego_);
                for(auto circle : circles){
                    Pos ego_circle1;
                    ego_circle1.x_ = ego_circle_.centerpoints_[0].x_;
                    ego_circle1.y_ = ego_circle_.centerpoints_[0].y_;
                    Pos ego_circle2;
                    ego_circle2.x_ = ego_circle_.centerpoints_[1].x_;
                    ego_circle2.y_ = ego_circle_.centerpoints_[1].y_;

                    if(circle.centerpoints_.size() ==2 ){
                        Pos temp1 = Pos(circle.centerpoints_[0].x_, circle.centerpoints_[0].y_, 0);
                        Pos temp2 = Pos(circle.centerpoints_[1].x_, circle.centerpoints_[1].y_, 0);
                        double distance_square1 = distance_square(ego_circle1,temp1 );
                        double distance_square2 = distance_square(ego_circle2,temp1 );
                        double distance_square3 = distance_square(ego_circle1,temp2 );
                        double distance_square4 = distance_square(ego_circle2,temp2 );
                        double threshold = circle.radius+ego_circle_.radius;
                        if(distance_square1 <threshold*threshold ||distance_square2 <threshold*threshold||
                            distance_square3 < threshold*threshold || distance_square4 < threshold*threshold ){
                            Result temp;
                            temp.collision_time = i*0.5;
                            temp.collision_spot = ego_pose;
                            result_.push_back(temp);
                            break_flag = true;
                            break;
                        }
                    }else{
                        Pos temp1 = Pos(circle.centerpoints_[0].x_, circle.centerpoints_[0].y_, 0);
                        double distance_square1 = distance_square(ego_circle1,temp1 );
                        double distance_square2 = distance_square(ego_circle2,temp1 );
                        double threshold = circle.radius+ego_circle_.radius;
                        if(distance_square1 <threshold*threshold ||distance_square2 <threshold*threshold ){
                            Result temp;
                            temp.collision_time = i*0.5;
                            temp.collision_spot = ego_pose;
                            result_.push_back(temp);
                            break_flag = true;
                            break;
                        }
                    }
                }
                if(break_flag){
                    break;
                }
                i++;
            }

            if(!break_flag){
               Result temp;
                temp.collision_time = 3;
                result_.push_back(temp);
            }
        }
    } 
    
    
    // global 좌표 기준
    void CollisionChecker::draw_ego_circle(const Object& ego){
        ego_circle_.centerpoints_.clear();
        double x_offset = ego.length_/3;
        ego_circle_.radius = sqrt(0.25*ego.width_*ego.width_+ x_offset*x_offset*0.25);
        double dx = x_offset*cos(-ego.yaw_);
        double dy = x_offset*sin(-ego.yaw_);
        Position temp1, temp2;
        temp1.x_ =  ego.x_ + dx;
        temp1.y_ = ego.y_ + dy;
        temp2.x_ = ego.x_-dx;
        temp2.y_ = ego.y_-dy;
        ego_circle_.centerpoints_.push_back(temp1);
        ego_circle_.centerpoints_.push_back(temp2);
        std::cout <<"ego circle1 position x,y : "<< temp1.x_ <<" "<<temp1.y_<<std::endl;
        std::cout <<"ego circle2 position x,y : "<< temp2.x_ <<" "<<temp2.y_<<std::endl;
    }
} //namespace Plan end

int main(){
    Plan::CollisionChecker collision;
    Plan::Objects objects;
    Plan::Pos start_pos = Plan::Pos(6700,5400,0);
    objects.objects_.push_back( Plan::Object(6703, 5405,4,2) );
    collision.update_current_state(6700,5400,0, 10/3.6,0);
    // collision.predict_ego(collision.current_pos_, 5,0 );
    collision.update_generator_state(start_pos.x_,start_pos.y_,start_pos.yaw_);
    collision.get_trajectories(0,5);
    collision.predict_egos(collision.trajectories_, start_pos );
    collision.draw_circles(objects, 0);
    collision.collision_check(collision.predicted_egos,collision.circles_);
    for(auto result : collision.result_){
        std::cout << result.collision_time<<std::endl;
        std::cout << result.collision_spot.x_<<" "<<result.collision_spot.y_<<std::endl;
    }
    std::cout << "end"<<std::endl;
}