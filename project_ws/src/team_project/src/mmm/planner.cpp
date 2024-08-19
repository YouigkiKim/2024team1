# include "planner.h"

namespace Plan{
    double distance_square(Pos& pos1, Pos& pos2){
        return (pos1.x_ - pos2.x_)*(pos1.x_ - pos2.x_) + (pos1.y_ - pos2.y_)*(pos1.y_ - pos2.y_);
    }
    Object::Object()
    : x_(0.0), y_(0.0){

    }
    Object::Object(double x,double y, std::string flag):x_(x), y_(y),flag_("HUMAN"){
            if(flag == "HUMAN" || flag == "BYCICLE")  Flag_ = Flag::HUMAN;
            else if(flag == "CONTROL_LOSS")  Flag_ = Flag::CONTROL_LOSS;
            else if(flag == "TRAFFIC_SIGN")  Flag_ = Flag::TRAFFIC_SIGN;
            else if(flag == "OPEN_DOOR")  Flag_ = Flag::OPEN_DOOR;
            else if(flag == "CUT_IN")  Flag_ = Flag::CUT_IN;
            else if(flag == "CONSTRUCTION_SITE") Flag Flag_ = Flag::CONSTRUCTION_SITE;
    };
    Object::Object(double x,double y, int flag)
    : x_(x), y_(y), int_flag_(flag){}
}