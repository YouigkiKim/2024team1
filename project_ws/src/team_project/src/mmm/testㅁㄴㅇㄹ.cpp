
#include "trajectorygenerator.h"
#include "Planner.h"

int main(){
    Plan::TrajectoryGenerator generator;
    generator.update_generator_state(6700,5400,-1.54)
    generator.get_trajectories(0,10);
    int i;
    for (std::vector<State> vec: trajectories_){
        std::cout <<"trajectory "<< i<< std::endl;
        for(State state: vec){
            std::cout <<state.x_<<" "<<state.y_<<" " << state.yaw_<<std::endl;
        }
        i++;
    }
}