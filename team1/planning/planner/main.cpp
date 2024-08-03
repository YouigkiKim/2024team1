#include "planner.h"
#include <cmath>



using namespace Plan;

int main(){
    double vel = 5;
    double dt = 0.5;
    Planner planner(0.0,0.0,-M_PI/4);
    Objects objects;
    while(! (vel == 0)){

        objects.create_objects();
        planner.placement_object(objects);
        planner.print_objects();
        planner.check_collision();

    }

    return 0;
}