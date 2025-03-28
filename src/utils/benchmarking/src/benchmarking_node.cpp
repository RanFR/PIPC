#include "benchmarking/benchmarking.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "benchmarking_node");
    ros::NodeHandle nh("~");

    Benchmarking benchmarking;
    benchmarking.initialize(nh);

    ros::spin();

    return 0;
}
