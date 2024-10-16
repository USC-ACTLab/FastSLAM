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

struct Observation2D MockManager2D::predictMeas(const struct Point2D& mu_prev) {
    float range = sqrtf( powf((mu_prev.x - m_curr_pose.x), 2) + powf((mu_prev.y - m_curr_pose.y), 2) );
    float bearing = atan2f((mu_prev.y - m_curr_pose.y), (mu_prev.x - m_curr_pose.x)) - m_curr_pose.theta_rad;

    bearing = bearing < 0 ? bearing + 2*M_PI*floorf(-bearing / M_PI) : bearing - 2*M_PI*floorf(bearing / M_PI);

    return {.range_m = range, .bearing_rad = bearing, .landmarkID = static_cast<int>(NONE_OBS_LM::prediction)};
}

Eigen::Matrix2f MockManager2D::measJacobian(const struct Point2D& mu_prev) const {
    float dx = mu_prev.x - m_curr_pose.x;
    float dy = mu_prev.y - m_curr_pose.y;
    Eigen::Matrix2f G;
    if (dx == 0 && dy == 0) {
        G << NAN, NAN,
             NAN, NAN;
    }
    else {
        G(0,0) = (dx)/ (sqrtf( powf(dx, 2) + powf(dy, 2) ));
        G(0,1) = (dy)/ (sqrtf( powf(dx, 2) + powf(dy, 2) ));
        G(1,0) = -(dy)/ (powf(dy, 2) + powf(dx, 2));
        G(1,1) = (dx)/ (powf(dy, 2) + powf(dx, 2));
    };
    return G;
}

struct Point2D MockManager2D::inverseMeas(const struct Pose2D& rob_pose,
                           const struct Observation2D& curr_obs) const {
    float x = curr_obs.range_m * cosf( curr_obs.bearing_rad + rob_pose.theta_rad );
    float y = curr_obs.range_m * sinf( curr_obs.bearing_rad + rob_pose.theta_rad );

    return {.x = x + rob_pose.x, .y = y + rob_pose.y};
}

Eigen::Matrix2f MockManager2D::getMeasNoise() const {
    return m_meas_noise;
}

Eigen::Matrix3f MockManager2D::getProcessNoise() const {
    return m_process_noise;
}

void MockManager2D::setObs(const struct Observation2D& new_obs){
    m_curr_obs = new_obs;
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
