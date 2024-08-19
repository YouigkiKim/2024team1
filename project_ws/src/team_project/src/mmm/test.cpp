
#include "trajectorygenerator.h"
#include "planner.h"

int main(){
    Plan::TrajectoryGenerator generator;
    generator.update_generator_state(6700,5400,M_PI/2);
    auto prev = std::chrono::high_resolution_clock::now();
    generator.get_trajectories(0,10);
    auto current = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds = current - prev;

    std::cout << "duration : " << elapsed_seconds.count()<<std::endl;
    int ii;
    while(generator.trajectories_.empty()){
        auto prev = current;
        for(Plan::State state : generator.trajectories_.front()){

            std::cout <<"trajectory "<< ii<< std::endl;
            std::cout <<"state "<< state.x_<<" "<< state.y_<<" " <<state.yaw_<<std::endl;
            generator.trajectories_.pop();
            ii++;
        }

    }
}