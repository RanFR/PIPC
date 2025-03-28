#include "pipc/pipc.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "pipc");
    ros::NodeHandle nh("~");

    PIPC pipc(nh);
    ros::spin();

    // ros::AsyncSpinner spinner(8);
    // spinner.start();
    // ros::waitForShutdown();

    return 0;
}
