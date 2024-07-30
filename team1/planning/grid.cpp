#include <iostream>
#include <vector>
#include <cmath>
#include <queue>


struct Object{
    double x_;
    double y_;
    double yaw_;
    Object(double x,double y, double yaw):x_(x), y_(y), yaw_(yaw){}
};

struct Objects{
    int object_num_;
    std::queue<Object> objects_;
    Objects(){
        int i=0;
        for (i=0;i<4;i++){
            objects_.push(Object(i,i,0));
        }
        object_num_ = i+1;
    }
    ~Objects(){}
};


class Grid{

private:
    std::vector<std::vector<int>> grid_;
    double cell_row_size_, cell_col_size_, road_width_;
    double vel_, x_, y_, yaw_;
    int resolution_;
    void calculateCellRow(){
        double ref=vel_*4/resolution_;
        if (ref > 2) cell_row_size_ = ref;
        else cell_row_size_ = 2
    }
    void calculateCellCol(){
        cell_col_size_= road_width_*1.5;
    }


public:
    Grid(double initial_speed)
    :vel_(initial_speed), road_width_(2.5), resolution_(3) {
        grid_.resize(resolution_*resolution_, std::vector<int>(resolution_*resolution_));
        calculateCellRow();
        calculateCellCol();
    }

    void updateGrid(double vel){
        vel_ = vel;
        calculateCellRow();
        grid_.clear();
        objectRegister();
    }

    void objectRegister(Objects objects){
        grid_.clear();
        for (int i=0;i<objects.object_num_;i++){
            get_grid(objects.objects_.front());
            objects.objects_.pop();
        }
    }
    void get_grid(const Object& object){
        int row_pos = static_cast<int>(object.x_/cell_row_size_);
        int col_pos = static_cast<int>(object.y_/cell_col_size_);
        //좌하단부터 0 1 2 중단 3 4 5 앞 6 7  8
        int index =(row_pos+1)*resolution_ + col_pos;
        if(index < grid_.size()){
            grid_[index] += 1;
        }
    }

};