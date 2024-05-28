/**
 * @file robot-manager.h
 * @brief Defines RobotManager and its derived classes for robot-specific code
 */

#pragma once

#include "core-structs.h"
#include "math-util.h"
#include "Eigen/Dense"
#include <iostream>

class RobotManager {
public:
    virtual void sampleIMU() = 0;
    virtual void sampleLandMark() = 0;
    virtual void sampleControl() = 0;
};

class RobotManager2D : public RobotManager {

protected:
    struct Pose2D m_curr_pose;
    struct VelocityCommand2D m_curr_command;
    const Eigen::Matrix2f m_meas_noise;
    struct Observation2D m_curr_obs;

    RobotManager2D(struct Pose2D init_pose,
                   struct VelocityCommand2D init_cmd,
                   Eigen::Matrix2f meas_noise) : m_curr_pose(init_pose),
        m_curr_command(init_cmd), m_meas_noise(meas_noise){
        m_curr_obs = { .range_m = 0, .bearing_rad = 0 };
    }
    RobotManager2D() {
        m_curr_pose = {.x = 0, .y = 0, .theta_rad = 0};
        m_curr_command = {.vx_mps = 0, .wz_radps = 0};
    }
    virtual ~RobotManager2D() {};

public:
    virtual void motionUpdate() = 0;
    virtual struct Observation2D predictMeas() = 0;
    virtual struct Observation2D getCurrObs() const = 0;
    virtual Eigen::Matrix2f measJacobian(const struct Point2D mu_prev) const = 0;
    virtual Eigen::Matrix2f getMeasNoise() const = 0;
};

class Create3Manager final : public RobotManager2D {

public:
    Create3Manager();
    Create3Manager(struct Pose2D init_pose,
                   struct VelocityCommand2D init_cmd,
                   Eigen::Matrix2f meas_noise): RobotManager2D(init_pose, init_cmd, meas_noise) {}
    ~Create3Manager() {};


    void sampleIMU() override;
    void sampleLandMark() override;
    void sampleControl() override;
    void motionUpdate() override;
    struct Observation2D getCurrObs() const override;
    struct Observation2D predictMeas() override;
    Eigen::Matrix2f measJacobian(const struct Point2D mu_prev) const override;
    Eigen::Matrix2f getMeasNoise() const override;
};
