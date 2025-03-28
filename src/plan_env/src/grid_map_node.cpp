#include "plan_env/grid_map.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "plan_env");
    ros::NodeHandle nh("~");

    GridMap grid_map;
    grid_map.initMap(nh);

    ros::spin();

    return 0;
}
