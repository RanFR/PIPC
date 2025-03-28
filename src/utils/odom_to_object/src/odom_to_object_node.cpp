#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include "object_msgs/MultiObstacleBoxMsg.h"

ros::Subscriber odom_sub;
ros::Publisher  object_pub;

void receiveOdometry(const nav_msgs::OdometryConstPtr& ptr) {
    object_msgs::MultiObstacleBoxMsg obs_msg;

    obs_msg.header.frame_id = "world";
    obs_msg.header.stamp    = ros::Time::now();

    object_msgs::ObstacleBoxMsg msg;
    msg.x       = ptr->pose.pose.position.x;
    msg.y       = ptr->pose.pose.position.y;
    msg.z       = 2.5;
    msg.x_width = 0.5;
    msg.y_width = 0.5;
    msg.z_width = 5.0;

    obs_msg.box.push_back(msg);
    object_pub.publish(obs_msg);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "odom_to_object");
    ros::NodeHandle nh("~");

    odom_sub = nh.subscribe<nav_msgs::Odometry>("/qualisys/obs/odom", 1, &receiveOdometry);

    object_pub = nh.advertise<object_msgs::MultiObstacleBoxMsg>("/dynamic_obstacles", 1);

    ros::spin();

    return 0;
}
