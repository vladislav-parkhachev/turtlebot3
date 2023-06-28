#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_gmapping_node");
    ros::NodeHandle nh;


    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Publisher path_publisher_odometry = nh.advertise<nav_msgs::Path>("/path_gmapping", 50);

    nav_msgs::Path path_gmapping;
    path_gmapping.header.frame_id = "odom";
    geometry_msgs::PoseStamped pose;

    ros::Rate r(10.0);

    while (nh.ok())
    {
        geometry_msgs::TransformStamped transformStamped;

        try
        {
            transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
        }

        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        pose.pose.position.x = transformStamped.transform.translation.x;
        pose.pose.position.y = transformStamped.transform.translation.y;
        pose.pose.position.z = transformStamped.transform.translation.z;

        pose.pose.orientation.w = transformStamped.transform.rotation.w;
        pose.pose.orientation.x = transformStamped.transform.rotation.x;
        pose.pose.orientation.y = transformStamped.transform.rotation.y;
        pose.pose.orientation.z = transformStamped.transform.rotation.z;

        path_gmapping.header.stamp = ros::Time::now();
        path_gmapping.poses.push_back(pose);
        path_publisher_odometry.publish(path_gmapping);

        ros::spinOnce();
        r.sleep();
    }
}
