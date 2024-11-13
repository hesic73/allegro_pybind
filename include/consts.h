#ifndef _CONSTS_H
#define _CONSTS_H

// for CAN communication
constexpr double delT = 0.003;

// USER HAND CONFIGURATION
constexpr bool RIGHT_HAND = true;
constexpr int HAND_VERSION = 4;

constexpr double tau_cov_const_v4 = 1200.0; // 1200.0 for SAH040xxxxx

#endif // _CONSTS_H