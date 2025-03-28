#include <gazebo_msgs/ModelStates.h>
#include <ros/ros.h>

#include "object_msgs/MultiObstacleBoxMsg.h"

ros::Subscriber gazebo_msgs_subscriber;
ros::Publisher  object_msgs_publisher;

void receiveGazeboMsgsCallback(const gazebo_msgs::ModelStatesConstPtr& ptr) {
    size_t idx {0};
    for (size_t i = 0; i < ptr->name.size(); ++i) {
        if (ptr->name[i] == "person") {
            idx = i;
            break;
        }
    }

    object_msgs::ObstacleBoxMsg obs_msg;
    obs_msg.x       = ptr->pose[idx].position.x;
    obs_msg.y       = ptr->pose[idx].position.y;
    obs_msg.z       = 2.5;
    obs_msg.x_width = 1.0;
    obs_msg.y_width = 1.0;
    obs_msg.z_width = 5.0;

    object_msgs::MultiObstacleBoxMsg obs_msgs;
    obs_msgs.header.frame_id = "world";
    obs_msgs.header.stamp    = ros::Time::now();
    obs_msgs.box.push_back(obs_msg);

    object_msgs_publisher.publish(obs_msgs);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "gazebo_to_object");
    ros::NodeHandle nh("~");

    gazebo_msgs_subscriber = nh.subscribe("/gazebo/model_states", 1, &receiveGazeboMsgsCallback);

    object_msgs_publisher = nh.advertise<object_msgs::MultiObstacleBoxMsg>("/dynamic_obstacles", 1);

    ros::spin();

    return 0;
}
