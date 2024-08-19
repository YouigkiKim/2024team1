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
        x_offset_ = ego_.length_/3;
        ego_circle_.radius = sqrt(0.25*ego_.width_*ego_.width_+ x_offset_*x_offset_*0.25);
    };
    CollisionChecker::CollisionChecker(const double pt, const double dt, const double r)
        : predict_time_(pt), dt_(dt), circle_radius_(r){
            // circles_pos.push_back(Object(-1.0,0));
            predict_step_ = predict_time_/dt_;
    };
    void CollisionChecker::update_current_state(double x, double y, double yaw, double v, double w){
        current_pos_.x_ = x -1.45*cos(-yaw);
        current_pos_.y_ = y-1.45*sin(-yaw);
        current_pos_.yaw_ = yaw;
    }
    void CollisionChecker::update_collision_state(const double& x,const double& y ,const double&,const double& yaw){
        current_pos_.x_ = x-1.45*cos(-yaw);
        current_pos_.y_ = y-1.45*sin(-yaw);
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
        if(object.length_/object.width_ > 1.4 || object.length_/object.width_ < 1/1.4){
            // two circle 
            double x_offset = std::max(object.length_,object.width_)/3;
            // circle.radius = sqrt(0.25*object.width_*object.width_+ x_offset*x_offset*0.25);
            circle.radius = std::min(object.width_, object.length_)/2;
            double dx, dy;
            dx = x_offset*cos(-ego_yaw);
            dy = x_offset*sin(-ego_yaw);
            Position temp1, temp2;
            temp1.x_ =  object.x_ + dx;
            temp1.y_ = object.y_ + dy;
            temp2.x_ = object.x_-dx;
            temp2.y_ = object.y_-dy;
            circle.centerpoints_.push_back(temp1);
            circle.centerpoints_.push_back(temp2);
            // std::cout <<"circle1 position x,y : "<< temp1.x_ <<" "<<temp1.y_<<std::endl;
            // std::cout <<"circle2 position x,y : "<< temp2.x_ <<" "<<temp2.y_<<std::endl;
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
        after_trajectories_.clear();
        int trajectory_num = 0;
        while( !trajectories.empty()){
            after_trajectories_.push_back(trajectories.front());
            int idx = 0;
            std::vector<State>& current_trajectory = trajectories.front();
            predicted_ego.clear();
            // std::cout <<  current_trajectory[0].x_<< " "<<current_trajectory[0].y_<< std::endl;
            for(int i=0;i<predict_step_;i++){
                double ds = current_trajectory[idx+1].vel_*dt_;
                int trajectory_step = static_cast<int>(std::round(ds/0.3));
                idx +=trajectory_step;
                if(idx > current_trajectory.size() || idx < 0) idx = current_trajectory.size() -1;
                // std::cout <<"ds " <<ds<<" velocity "<<current_trajectory[idx+1].vel_<<" trajectory step "<<trajectory_step<<std::endl;

                State predict_point = trajectories.front()[idx];
                predicted_ego.push_back(predict_point);
                // std::cout << "trajectory : "<<trajectory_num << " predict x,y : "<<  predict_point.x_<<" "<<predict_point.y_<< "idx " << idx
                // <<" trajectory size " << current_trajectory.size()<<std::edl;
                // std::cout << std::endl;
            }
            predicted_egos.push_back(predicted_ego);
            trajectories.pop();
            trajectory_num ++;
        }
    }
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
            // std::cout << "predicted after "<< dt_*(i+1)<<"  "<<current_pos.x_<<", "<<current_pos.y_<<std::endl;
        }
    }

    void CollisionChecker::collision_check(const std::vector<Circle>& circles){
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
        std::cout << "collision check result ======="<<std::endl;
        for(auto result : result_){
            std::cout <<"  collision spot "<< result.collision_spot.x_<< ", "<<result.collision_spot.y_ <<"  collisiont time : "<<result.collision_time<<std::endl;
        }
    } 
    
    
    // global 좌표 기준
    void CollisionChecker::draw_ego_circle(const Object& ego){
        ego_circle_.centerpoints_.clear();
        double& x_offset = x_offset_;
        double dx = x_offset*cos(-ego.yaw_);
        double dy = x_offset*sin(-ego.yaw_);
        Position temp1, temp2;
        temp1.x_ =  ego.x_ + dx;
        temp1.y_ = ego.y_ + dy;
        temp2.x_ = ego.x_-dx;
        temp2.y_ = ego.y_-dy;
        ego_circle_.centerpoints_.push_back(temp1);
        ego_circle_.centerpoints_.push_back(temp2);
        // std::cout << "=======ego circle    "<<temp1.x_ << " "<<temp1.y_ << ",    "<<temp2.x_ <<"  "<<temp2.y_ << std::endl; 
        // std::cout <<"ego circle1 position x,y : "<< temp1.x_ <<" "<<temp1.y_<<std::endl;
        // std::cout <<"ego circle2 position x,y : "<< temp2.x_ <<" "<<temp2.y_<<std::endl;
    }
    void CollisionChecker::check_collision(const Objects& objects,const double& x,const double& y,const double& yaw,  const double& yaw_error, const double& velocity){
        update_generator_state(x,y,yaw);
        get_trajectories(yaw_error, velocity);
        predict_egos(trajectories_,current_pos_);
        draw_circles(objects, yaw);
        collision_check(circles_);
    }

    void CollisionChecker::visualization_circles(const rviz_visual_tools::RvizVisualToolsPtr& visual_tools){
        for(auto circle : circles_){
            double radius = circle.radius;
            for(auto center : circle.centerpoints_){
                // std::cout << "==== circle center "<<center.x_ << " "<<center.y_ <<"  ==="<<std::endl;
                Eigen::Vector3d start;
                Eigen::Vector3d end;
                start << center.x_,center.y_,0;
                end << center.x_,center.y_,2;
                if(! visual_tools -> publishCylinder(start, end,rviz_visual_tools::colors::BLUE, static_cast<double>(radius))){
                    ROS_INFO(" circles visualization fault");
                };
            }
        }
        Eigen::Vector3d ego_start1;
        Eigen::Vector3d ego_end1;
        Eigen::Vector3d ego_start2;
        Eigen::Vector3d ego_end2;

        ego_start1 << x_offset_,0,1;
        ego_end1 << x_offset_,0,1;

        ego_start2<< -x_offset_,0,1;
        ego_end2<< -x_offset_,0,1;

        ego_start1 = transform_mat_*ego_start1;
        ego_start2 = transform_mat_*ego_start2;
        ego_end1 = transform_mat_*ego_end1;
        ego_end2 = transform_mat_*ego_end2;
        
        ego_start1[2] = 0;
        ego_start2[2] = 0;
        ego_end1[2] =2;
        ego_end2[2]= 2;
        // ego_start1 << ego_circle_.centerpoints_[0].x_ , ego_circle_.centerpoints_[0].y_, 0.0;
        // ego_start1 << ego_circle_.centerpoints_[0].x_ , ego_circle_.centerpoints_[0].y_, 0.0;
        // ego_end1 << ego_circle_.centerpoints_[0].x_ , ego_circle_.centerpoints_[0].y_, 2.0;
        // ego_start2 << ego_circle_.centerpoints_[1].x_ , ego_circle_.centerpoints_[1].y_, 0.0;
        // ego_end2 << ego_circle_.centerpoints_[1].x_ , ego_circle_.centerpoints_[1].y_,2.0;

        if(! visual_tools -> publishCylinder(ego_start1, ego_end1, rviz_visual_tools::colors::PINK,  1)){
            ROS_INFO(" collision result visualization fault");

        };
        if(! visual_tools -> publishCylinder(ego_start2, ego_end2, rviz_visual_tools::colors::PINK,  1)){
            ROS_INFO(" collision result visualization fault");

        };
    }
    void CollisionChecker::visualization_collision(const rviz_visual_tools::RvizVisualToolsPtr& visual_tools){
        
        for(auto& result : result_){
            auto& spot = result.collision_spot;
            if(spot.x_ != 0.0 && spot.y_ != 0.0){
                Eigen::Vector3d start;
                Eigen::Vector3d end;
                start << spot.x_, spot.y_, 0;
                end << spot.x_, spot.y_, 2;
                if(!visual_tools -> publishCylinder(start, end, rviz_visual_tools::colors::RED,  1)){
                    ROS_INFO(" collision result visualization fault");
                }
            }
        }
    }
} //namespace Plan end

// int main(){
//     Plan::CollisionChecker collision;
//     Plan::Objects objects;
//     Plan::Pos start_pos = Plan::Pos(6700,5400,0);
//     objects.objects_.push_back( Plan::Object(6703, 5405,4,2) );
//     collision.update_current_state(6700,5400,0, 10/3.6,0);
//     // collision.predict_ego(collision.current_pos_, 5,0 );
//     collision.update_generator_state(start_pos.x_,start_pos.y_,start_pos.yaw_);
//     collision.get_trajectories(0,5);
//     collision.predict_egos(collision.trajectories_, start_pos );
//     collision.draw_circles(objects, 0);
//     collision.collision_check(collision.predicted_egos,collision.circles_);
//     for(auto result : collision.result_){
//         std::cout << result.collision_time<<std::endl;
//         std::cout << result.collision_spot.x_<<" "<<result.collision_spot.y_<<std::endl;
//     }
//     std::cout << "end"<<std::endl;
// }