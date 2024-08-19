#include "plan_node.h"



int main(int argc, char** argv){
    ros::init(argc, argv, "plan_node");
    ros::NodeHandle nh;
    Plan::PlanNode  planner(nh);
    ros::Rate rate(10);  // 10Hz

    while (ros::ok()) {
        planner.publish();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}