#include "pointcloud_processor/pointcloud_processor.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_pointcloud_processor");
    ros::NodeHandle nh("~");

    PointCloudProcessor pcp;
    pcp.init(nh);

    ros::spin();

    return 0;
}
