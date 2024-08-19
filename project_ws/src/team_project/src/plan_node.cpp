#include "plan_node.h"


namespace Plan{
    
    PlanNode::PlanNode(){}
    PlanNode::PlanNode(ros::NodeHandle& nh): path_num_(7){

        //ros pub
        way_pub_ = nh.advertise<geometry_msgs::Point>("/ref/ref_point",10);
        vel_pub_ = nh.advertise<std_msgs::Float64>("/ref/ref_vel",10);
        ref_pub_ = nh.advertise<custom_msgs::ref_control>("/ref/control",10);
        collision_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/plan/collision",10);
        for(int i=0; i<path_num_;i++){
            std::string topic_name = "/plan/path" + std::to_string(i); 
            ros::Publisher trajectory_pub = nh.advertise<nav_msgs::Path>(topic_name,10);
            trajectory_pubs_.push_back(trajectory_pub);
            nav_msgs::Path path_msg;
            path_msg.header.frame_id = "map"; // 필요한 경우 프레임 설정
            path_msgs_.push_back(path_msg);
        }

        //ros sub
        objects_sub_ = nh.subscribe<derived_object_msgs::ObjectArray>("/object_data_array",10, &PlanNode::callback_objects, this);
        pose_sub_ = nh.subscribe<custom_msgs::vehicle_state>("/carla/hero/pose",10, &PlanNode::callback_pose, this);
        speed_sub_ = nh.subscribe<carla_msgs::CarlaSpeedometer>("/carla/hero/Speed", 10, &PlanNode::callback_speed, this);
        
        x_ = 6770.755371 ;
        y_ =  5443.435059;
        yaw_ = -89.5; //degree

        //visualization
        visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("map","/rviz_visual_markers"));

    }

    void PlanNode::callback_pose(const custom_msgs::vehicle_state::ConstPtr& msg){
        yaw_ = msg -> heading;
        x_ = msg -> x ;
        y_ = msg -> y ;

        // By Custom Message
        //x_ = msg -> x;
        //y_ = msg -> y;
        //yaw_ = msg -> heading;

        // ROS_INFO("by pose           %f %f ", x_,y_);
    }
    // void PlanNode::regist_property_points(Object& object, const geometry_msgs::Polygon& msg){
    //     object.property_points_.clear();
    //     // //Debug Code
    //     // object.property_points_.push_back(Position(1, 1));
    //     // std::cout <<"polygon register"<< object.property_points_[0].x_ <<object.property_points_[0].y_<<std::endl;
    //     if(msg.points.size() != 0){
    //         for(const auto& position : msg.points){
    //             object.property_points_.push_back( Position(position.x, position.y) );
    //         }
    //     }
    // }
    //call back function
    void PlanNode::callback_objects(const derived_object_msgs::ObjectArray::ConstPtr& msg ){
        objects_.object_num_ = 0;
        objects_.clear();
        for (const auto& object : msg -> objects ){
            objects_.objects_.push_back(Object());
            objects_.objects_[objects_.object_num_].x_          =   object.pose.position.x;
            objects_.objects_[objects_.object_num_].y_          =   object.pose.position.y;
            objects_.objects_[objects_.object_num_].int_flag_   =   object.classification;
            objects_.objects_[objects_.object_num_].width_      =   abs(object.shape.dimensions[shape_msgs::SolidPrimitive::BOX_Y]);
            objects_.objects_[objects_.object_num_].length_     =   abs(object.shape.dimensions[shape_msgs::SolidPrimitive::BOX_X]);
            // std::cout <<"object size ===========    " << object.shape.BOX_Y << "  "<< object.shape.BOX_Y << "      ======"<<std::endl;
            // regist_property_points(objects_.objects_[objects_.object_num_], object.polygon);
            // std::cout << objects_.object_num_<<"th object detected "<<objects_.objects_[objects_.object_num_].x_<<", "<<objects_.objects_[objects_.object_num_].y_<<std::endl;

            objects_.object_num_ ++;
        }
        // print_objects();
        // ROS_INFO("object callback object num : %d", objects_.object_num_);
    }
    
    void PlanNode::callback_speed(const carla_msgs::CarlaSpeedometer::ConstPtr& msg){
        PlanNode::vel_ = msg -> speed;
    }

    void PlanNode::publish_ref(){
        Pos waypoint = get_waypoint(x_,y_);
        ref_msg_.x = waypoint.x_;
        ref_msg_.y = waypoint.y_;
        ref_msg_.yaw = waypoint.yaw_;
        // ref_msg_.velocity = 15/3.6;
        double predicttime = 3.0;
        ref_msg_.velocity = decide_vel(after_trajectories_, result_, vel_, predicttime);
        ref_msg_.aeb_flag = false;
        // std::cout << "pose "<<x_<<" "<<y_<<std::endl; 
        // std::cout << "next "<<waypoint.x_<<" "<<waypoint.y_<<std::endl; 
        ref_pub_.publish(ref_msg_);
    }
    // void PlanNode::publish_trajectory(){
    //     auto road_pose = get_waypoint(x_,y_);
    //     produce_trajectories(x_,y_,yaw_,yaw_- road_pose.yaw_,vel_);

    //     geometry_msgs::PoseStamped temp;
    //     tf2::Quaternion q;
    //     int i=0;
    //     for(auto trajectory : after_trajectories_){
    //         // std::cout << after_trajectories_.size()<<std::endl;
    //         path_msgs_[i].poses.clear();
    //         for(auto state: trajectory){
    //             temp.pose.position.x = state.x_;
    //             temp.pose.position.y = state.y_;
    //             q.setRPY(0,0,state.yaw_);
    //             temp.pose.orientation.x = q.x();
    //             temp.pose.orientation.y = q.y();
    //             temp.pose.orientation.z = q.z();
    //             temp.pose.orientation.w = q.w();
    //             // std::cout << state.x_ << state.y_ << "   ";
    //             path_msgs_[i].poses.push_back(temp);
    //         }
    //         std::cout << std::endl;
    //         i++;
    //     }
    //     for(int i=0;i< path_msgs_.size();i++ ){
    //         trajectory_pubs_[i].publish(path_msgs_[i]);
    //     }
    // }
    void PlanNode::publish_collision(){
        auto road_pose = get_waypoint(x_,y_);
        // produce_trajectories(x_,y_,yaw_, road_pose.yaw_ - yaw_ ,20);
        CollisionChecker::check_collision(objects_, x_,y_,yaw_,  road_pose.yaw_ - yaw_,vel_);
    }
    void PlanNode::publish(){
        update_flag(objects_);
        ref_way_ = get_waypoint(x_,y_);
        publish_collision();
        publish_ref();
        visualization();
        // ROS_INFO("Successfully publish");
    }
    void PlanNode::visualization(){
        //collision check list, path, object 
        //collision check visualizationt
        auto& road_pose = ref_way;
        auto& waypoint = ref_way;

        geometry_msgs::PoseStamped temp;
        tf2::Quaternion q;
        int i=0;
        for(auto trajectory : after_trajectories_){
            // std::cout << after_trajectories_.size()<<std::endl;
            path_msgs_[i].poses.clear();
            for(auto state: trajectory){
                temp.pose.position.x = state.x_;
                temp.pose.position.y = state.y_;
                q.setRPY(0,0,state.yaw_);
                temp.pose.orientation.x = q.x();
                temp.pose.orientation.y = q.y();
                temp.pose.orientation.z = q.z();
                temp.pose.orientation.w = q.w();
                // std::cout << state.x_ << state.y_ << "   ";
                path_msgs_[i].poses.push_back(temp);
            }
            // std::cout << std::endl;
            i++;
        }
        for(int i=0;i< path_msgs_.size();i++ ){
            trajectory_pubs_[i].publish(path_msgs_[i]);
        }
        visual_tools_ -> deleteAllMarkers();
        visualization_circles(visual_tools_);
        visualization_collision(visual_tools_);
        visual_tools_ -> trigger();

    }

}