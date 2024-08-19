#include "localization.h"



int main(int argc, char** argv){
    ros::init(argc, argv, "localization_node");
    ros::NodeHandle nh;
    Localization::Localization localization(nh);
    ros::Rate rate(10);  // 100Hz

    while (ros::ok()) {
        localization.publish();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}