#include "pointcloud_processor/pointcloud_processor.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_processor");
    ros::NodeHandle nh("~");

    PointCloudProcessorPtr ptr = std::make_unique<PointCloudProcessor>();
    ptr->init(nh);

    ros::spin();

    return 0;
}
