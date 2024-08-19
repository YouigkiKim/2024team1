#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_broadcaster_node");
    ros::NodeHandle nh;

    tf2_ros::TransformBroadcaster tf_broadcaster;

    ros::Rate rate(10.0);
    while (nh.ok())
    {
        geometry_msgs::TransformStamped transformStampedHero;
        geometry_msgs::TransformStamped transformStampedBase;

        // Set the timestamp to the current ROS time
        ros::Time current_time = ros::Time::now();

        // Hero/front transformation
        transformStampedHero.header.stamp = current_time;
        transformStampedHero.header.frame_id = "map";      // Parent frame
        transformStampedHero.child_frame_id = "hero/front";  // Child frame

        // Transformation settings (example values)
        transformStampedHero.transform.translation.x = 0.0;
        transformStampedHero.transform.translation.y = 0.0;
        transformStampedHero.transform.translation.z = 0.0;
        transformStampedHero.transform.rotation.x = 0.0;
        transformStampedHero.transform.rotation.y = 0.0;
        transformStampedHero.transform.rotation.z = 0.0;
        transformStampedHero.transform.rotation.w = 1.0;

        // Base_link transformation
        transformStampedBase.header.stamp = current_time;
        transformStampedBase.header.frame_id = "map";      // Parent frame
        transformStampedBase.child_frame_id = "base_link";  // Child frame

        // Transformation settings (example values)
        transformStampedBase.transform.translation.x = 0.0;
        transformStampedBase.transform.translation.y = 0.0;
        transformStampedBase.transform.translation.z = 0.0;
        transformStampedBase.transform.rotation.x = 0.0;
        transformStampedBase.transform.rotation.y = 0.0;
        transformStampedBase.transform.rotation.z = 0.0;
        transformStampedBase.transform.rotation.w = 1.0;

        // Broadcast the transforms
        tf_broadcaster.sendTransform(transformStampedHero);
        tf_broadcaster.sendTransform(transformStampedBase);

        rate.sleep();
    }

    return 0;
}
