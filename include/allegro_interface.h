#ifndef _ALLEGRO_INTERFACE_H
#define _ALLEGRO_INTERFACE_H

#include <Eigen/Dense>
#include <BHand/BHand.h>
#include "rDeviceAllegroHandCANDef.h"
#include <array>
#include <thread>
#include <atomic>

// hsc: this is not thread-safe!!!
// TODO: sync
class AllegroInterface
{
public:
    AllegroInterface();
    ~AllegroInterface();
    bool start();
    void stop();

    void set_joint_positions(const Eigen::VectorXd &positions, bool use_delta = false);
    void set_pd_gains(const Eigen::VectorXd &kp, const Eigen::VectorXd &kd);
    void set_motion_type(eMotionType motionType);
    Eigen::VectorXd get_joint_positions();
    Eigen::VectorXd get_target_joint_positions();

private:
    BHand *pBHand;
    AllegroHand_DeviceMemory_t vars;

    std::array<double, MAX_DOF> q;
    std::array<double, MAX_DOF> q_des;
    std::array<double, MAX_DOF> tau_des;
    std::array<double, MAX_DOF> cur_des;

    int CAN_Ch;
    std::atomic_bool ioThreadRun;
    std::thread thread_io;

    void ComputeTorque();

    void CANCommunication();
    void CloseCAN();
};

#endif // _ALLEGRO_INTERFACE_H
