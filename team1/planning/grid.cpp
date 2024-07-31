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
        // int i=0;
        // for (i=0;i<4;i++){
        //     objects_.push(Object(i,i,0));
        // }
        // object_num_ = i+1;

        objects_.push(Object(3,3,0));
        objects_.push(Object(-3,-3,0));
        objects_.push(Object(-2,3,0));
        objects_.push(Object(4,1,0));
        objects_.push(Object(1,1,0));
        objects_.push(Object(3,-3,0));
        object_num_ = 6;
    }
    // Objects print_object(){
    //     Objects temp_s;
    //     std::cout << "num of object "<< object_num_ << std::endl;
    //     for(int i=0;i<object_num_;i++){
    //         Object temp  = objects_.front();
    //         std::cout << "object " << i<< " (x,y) "<<temp.x_<<temp.y_<<std::endl;
    //         objects_.pop();
    //         temp_s.objects_.push(temp);
    //         temp_s.object_num_ ++;
    //     }

    //     return temp_s;
    // }
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
        if ( ref > 1.5) cell_row_size_ = ref;
        else if (ref > 3) cell_row_size_ = 3;
        else cell_row_size_ = 1.5;
    }

    void calculateCellCol(){
        cell_col_size_= road_width_*1.2;
    }

    void get_grid(const Object& object){
        //x,y는 차량중심 local 좌표
        int row_pos = static_cast<int>(object.x_/cell_row_size_);
        int col_pos = static_cast<int>(object.y_/cell_col_size_);
        //차량 뒷부분부터 0,1,2행 차량 좌측부터 0,1,2열
        int index =(row_pos+1)*resolution_ + col_pos;
        grid_[row_pos+1][col_pos+1] +=1;
    }

    void objectRegister(Objects& objects){
        for(int i=0; i< resolution_;i++){
            grid_[i].clear();
        }
        grid_.clear();
        for (int i=0;i<objects.object_num_;i++){
            get_grid(objects.objects_.front());
            objects.objects_.pop();
        }
    }


public:
    Grid(double initial_speed)
    :vel_(initial_speed), road_width_(2.5), resolution_(3) {
        grid_.resize(resolution_*resolution_, std::vector<int>(resolution_*resolution_));
        calculateCellRow();
        calculateCellCol();
    }

    void updateGrid(double vel, Objects& objects){
        vel_ = vel;
        calculateCellRow();
        objectRegister(objects);
    }


    void print_grid(){

        std::cout << " --    --     -- " << "\n";
        for(int i=0;i<3;i++){
            std::cout << "| "<<grid_[2][i]<<" | ";
        }
        std::cout << "\n";
        std::cout << " --    --     -- " << "\n";
        for(int i=0;i<3;i++){
            std::cout << "| "<<grid_[1][i]<<" | ";
        }
        std::cout << "\n";
        std::cout << " --    --     -- " << "\n";
        for(int i=0;i<3;i++){
            std::cout << "| "<<grid_[0][i]<<" | ";
        }
        std::cout << "\n";
        std::cout << "current velocity :    "<<vel_<<std::endl;
        std::cout << "current cell size :   " << cell_row_size_<<std::endl;
    }
};

int main(){

    Objects objects;
    // objects = objects.print_object();
    int vel;
    Grid grid = Grid(vel);

    while (vel != 0){
        std::cout << "put in current velocity"<<std::endl;
        std::cin >> vel;
        grid.updateGrid(vel,objects);
        grid.print_grid();
    }

}