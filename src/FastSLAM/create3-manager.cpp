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
    float x = curr_obs.range_m * cosf( curr_obs.bearing_rad + rob_pose.theta_rad );
    float y = curr_obs.range_m * sinf( curr_obs.bearing_rad + rob_pose.theta_rad );

    return {.x = x, .y = y};
}

struct Observation2D Create3Manager::predictMeas(const struct Point2D& mu_prev) {
    float range = sqrtf( powf((mu_prev.x - m_curr_pose.x), 2) + powf((mu_prev.y - m_curr_pose.y), 2) );
    float bearing = atan2f((mu_prev.y - m_curr_pose.y), (mu_prev.x - m_curr_pose.x)) - m_curr_pose.theta_rad;

    bearing = bearing < 0 ? bearing + 2*M_PI*floorf(-bearing / M_PI) : bearing - 2*M_PI*floorf(bearing / M_PI);

    return {.range_m = range, .bearing_rad = bearing, .landmarkID = static_cast<int>(NONE_OBS_LM::prediction)};
}

Eigen::Matrix2f Create3Manager::measJacobian(const struct Point2D& mu_prev) const {
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

Eigen::Matrix2f Create3Manager::getMeasNoise() const {
    return m_meas_noise;
}

Eigen::Matrix3f Create3Manager::getProcessNoise() const {
    return m_process_noise;
}
