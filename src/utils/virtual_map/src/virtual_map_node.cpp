#include "virtual_map/virtual_map.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "virtual_map");
    ros::NodeHandle nh("~");

    VirtualMap virtual_map;
    virtual_map.initialize(nh);

    ros::spin();

    return 0;
}
