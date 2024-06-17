/**
 * @file mock-manager2d.cpp
 * @brief defines mock robot manager for testing EKF
 */
#ifdef USE_MOCK
#include "robot-manager.h"

void MockManager2D::sampleIMU(){
    std::cout << m_curr_pose.x << m_curr_pose.y << m_curr_pose.theta_rad << std::endl;
}

void MockManager2D::sampleLandMark() {
    std::cout << m_curr_obs.range_m << " " << m_curr_obs.bearing_rad << std::endl;
}

void MockManager2D::sampleControl() {
    std::cout << m_curr_command.vx_mps << " " << m_curr_command.wz_radps << std::endl;
}

struct Pose2D MockManager2D::motionUpdate() {
    struct Pose2D new_pose = m_curr_pose;
    new_pose.x = m_curr_command.vx_mps * cosf(m_curr_pose.theta_rad) + m_curr_pose.x;
    new_pose.y = m_curr_command.vx_mps * sinf(m_curr_pose.theta_rad) + m_curr_pose.y;
    new_pose.theta_rad = m_curr_command.wz_radps + m_curr_pose.theta_rad;

    m_curr_pose = new_pose;
    return new_pose;
}

struct Observation2D MockManager2D::getCurrObs() const {
    return m_curr_obs;
}

struct Observation2D MockManager2D::predictMeas(const struct Point2D mu_prev) {
    float range = sqrtf( powf((mu_prev.x - m_curr_pose.x), 2) + powf((mu_prev.y - m_curr_pose.y), 2) );
    float bearing = atan2f((mu_prev.y - m_curr_pose.y), (mu_prev.x - m_curr_pose.x)) - m_curr_pose.theta_rad;

    bearing = bearing < 0 ? bearing + 2*M_PI*floorf(-bearing / M_PI) : bearing - 2*M_PI*floorf(bearing / M_PI);

    return {.range_m = range, .bearing_rad = bearing, .landmarkID = static_cast<int>(NONE_OBS_LM::prediction)};
}

// TODO
Eigen::Matrix2f MockManager2D::measJacobian(const struct Point2D mu_prev) const {
   return Eigen::Matrix2f::Zero();
}

Eigen::Matrix2f MockManager2D::getMeasNoise() const {
    return m_meas_noise;
}

void MockManager2D::setObs(const struct Observation2D& new_obs){
    m_curr_obs.range_m = new_obs.range_m;
    m_curr_obs.bearing_rad = new_obs.bearing_rad;
    m_curr_obs.landmarkID = new_obs.landmarkID;
}

void MockManager2D::setControl(const struct VelocityCommand2D& new_ctrl){
    m_curr_command.vx_mps = new_ctrl.vx_mps;
    m_curr_command.wz_radps = new_ctrl.wz_radps;

}

void MockManager2D::setState(const struct Pose2D& new_pose){
    m_curr_pose.x = new_pose.x;
    m_curr_pose.y = new_pose.y;
    m_curr_pose.theta_rad = new_pose.theta_rad;

}

#endif
