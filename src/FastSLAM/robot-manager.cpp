#include "robot-manager.h"


Create3Manager::Create3Manager() {
    std::cout << "Create 3 Manager created" << std::endl;
}

void Create3Manager::sampleIMU(){
    (void) 0;
}
void Create3Manager::sampleLandMark() {
    (void) 0;
}
void Create3Manager::sampleControl() {
    (void) 0;
}
void Create3Manager::motionUpdate() {
    (void) 0;
}

struct Observation2D Create3Manager::getCurrObs() const {
    return m_curr_obs;
}
struct Observation2D Create3Manager::predictMeas() {
    struct Observation2D ret = {.range_m = 0, .bearing_rad = 0, .landmarkID = 0};
    return ret;
}

Eigen::Matrix2f Create3Manager::measJacobian(const struct Point2D mu_prev) const {
   return Eigen::Matrix2f::Zero();
}

Eigen::Matrix2f Create3Manager::getMeasNoise() const {
    return m_meas_noise;
}
