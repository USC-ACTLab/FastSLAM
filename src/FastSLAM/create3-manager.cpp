/**
 * @file create3-manager.cpp
 * @brief defines the create3 manager class and methods
 */
#include "robot-manager.h"


void Create3Manager::sampleIMU(){
    (void) 0;
}
void Create3Manager::sampleLandMark() {
    (void) 0;
}
void Create3Manager::sampleControl() {
    (void) 0;
}
struct Pose2D Create3Manager::motionUpdate() {
    return {.x = 0, .y = 0, .theta_rad = 0};
}

struct Observation2D Create3Manager::getCurrObs() const {
    return m_curr_obs;
}

struct Point2D Create3Manager::inverseMeas(const struct Pose2D& rob_pose, const struct Observation2D &curr_obs) const{
    return {.x = 0, .y = 0};
}

struct Observation2D Create3Manager::predictMeas(const struct Point2D& mu_prev) {
    struct Observation2D ret = {.range_m = 0, .bearing_rad = 0, .landmarkID = 0};
    return ret;
}

Eigen::Matrix2f Create3Manager::measJacobian(const struct Point2D& mu_prev) const {
   return Eigen::Matrix2f::Zero();
}

Eigen::Matrix2f Create3Manager::getMeasNoise() const {
    return m_meas_noise;
}

Eigen::Matrix3f Create3Manager::getProcessNoise() const {
    return m_process_noise;
}
