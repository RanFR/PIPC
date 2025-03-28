#include "mpc/mpc.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_mpc");
    ros::NodeHandle nh("~");

    MPCPtr mpc_ptr = std::make_unique<MPC>();
    mpc_ptr->init(nh);

    std::vector<Eigen::Vector3d> path = {{1, 0.5, 1}, {2, 1.0, 1}, {3, 0.5, 1}, {4, 0, 1}, {5, 0, 1}};
    std::vector<Eigen::Vector3d> vel  = {{1, 1, 0}, {1, 0, 0}, {1, -1, 0}, {1, 0, 0}, {0, 0, 0}};
    std::vector<Eigen::Vector3d> acc  = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    Eigen::Vector3d              p0 {0, 0, 1}, v0 {1, 0, 0}, a0 {0, 0, 0};

    mpc_ptr->setStatus(p0, v0, a0);
    for (int i = 0; i < mpc_ptr->MPC_HORIZON; ++i) {
        mpc_ptr->setGoal(path.at(i), vel.at(i), acc.at(i), i);
    }
    mpc_ptr->setSFC(path, 0.2);

    mpc_ptr->run();
    Eigen::Vector3d u_optimal;
    double          t = mpc_ptr->MPC_STEP;
    for (int i = 0; i < mpc_ptr->MPC_HORIZON; ++i) {
        mpc_ptr->getOptimCmd(u_optimal, i);
        p0 = p0 + t * v0 + t * t / 2.0 * a0 + t * t * t / 6.0 * u_optimal;
        v0 = v0 + t * a0 + t * t / 2.0 * u_optimal;
        a0 = a0 + t * u_optimal;
        std::cerr << "Index: " << i << ", pos: " << p0.transpose() << std::endl;
    }

    return 0;
}
