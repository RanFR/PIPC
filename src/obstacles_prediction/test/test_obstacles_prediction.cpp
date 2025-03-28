#include "obstacles_prediction/obstacles_prediction.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_obstacles_prediction");
    ros::NodeHandle nh("~");

    ObstaclesPrediction obp;
    obp.init(nh);

    ros::spin();

    return 0;
}
