#pragma once

#ifndef PLANNER_H
#define PLANNER_H

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


namespace Plan{

    struct Position{
        double x_, y_;
        Position(): x_(0.0), y_(0.0){};
        Position(double x,double y) : x_(x),y_(y){};
    };
    struct Pos : Position{
        double yaw_;
        Pos(){};
        Pos(double x, double y, double yaw)
        : Position(x,y), yaw_(yaw){};
    };
    struct State : Pos{
        double vel_;
        double k_;
    };
    double distance_square(const Pos& pos1, const Pos& pos2);

    enum class Drive{
        HIGH,
        LOW,
    };
    struct object_flag{
        bool                    human;
        bool                    red_sign;
        bool                    green_sign;
        bool                    bycicle;
    };

    struct Object{
        double                  x_;
        double                  y_;
        double                  width_;
        double                  length_;
        double                  yaw_;
        std::string             flag_;
        std::vector<Position>   property_points_;

        int             int_flag_;
        enum class      Flag{
            HUMAN,
            CONTROL_LOSS,
            TRAFFIC_SIGN,
            OPEN_DOOR,
            CUT_IN,
            CONSTRUCTION_SITE
        };
        Flag Flag_;
        Object();
        Object(double x,double y, std::string flag);
        Object(double x,double y, int flag);
        Object(const double& x,const double&y ,const double& length,const double& width)
        :x_(x),y_(y),length_(length),width_(width){};
    };

    // Pos rotation(Pos& Ego, Pos& object);
    struct Objects{
        int             object_num_;
        std::vector<Object> objects_;
        Objects() : object_num_(0){};
        //test method >> ros msg형태에 맞춰 바꿔줘야함 아마 콜백함수가 될 예정
        void create_objects(){
            clear();
            objects_.push_back(Object(0,15,1));
            objects_.push_back(Object(-8,-10,1));
            objects_.push_back(Object(-6,20,1));
            objects_.push_back(Object(2,30,1));
            objects_.push_back(Object(5,-3,1));
            objects_.push_back(Object(10,-10,1));
            objects_.push_back(Object(-10,30,1));
            object_num_ = 7;
        }
        void clear(){
            object_num_ = 0;
            objects_.clear();
        }
        ~Objects(){}
    };
    struct Circle : Object{
        double radius;
        std::vector<Position> centerpoints_; 
    };
    struct Result{
        double collision_time;
        Pos collision_spot;
    };

}

#endif //PLANNER_H