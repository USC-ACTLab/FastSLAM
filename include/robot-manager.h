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
    /**
     * @brief samples from the robot IMU distribution, pure virtual function
     */
    virtual void sampleIMU() = 0;

    /**
     * @brief obtain a landmark measurement from the appropriate sensor,
     * pure virtual function
     */
    virtual void sampleLandMark() = 0;

    /**
     * @brief get current robot control input, pure virtual function
     */
    virtual void sampleControl() = 0;
};

class RobotManager2D : public RobotManager {

protected:
    /**
     * @brief current robot pose member variable
     */
    struct Pose2D m_curr_pose;

    /**
     * @brief current robot command input
     */
    struct VelocityCommand2D m_curr_command;

    /**
     * @brief robot landmark sensor measurement noise, constant
     */
    const Eigen::Matrix2f m_meas_noise;

    /**
     * @brief robot landmark sensor measurement noise, constant
     */
    const Eigen::Matrix3f m_process_noise;

    /**
     * @brief current landmark observation data (robot frame)
     */
    struct Observation2D m_curr_obs;

    /**
     * @brief perceputal range of the robot, used for pruning dubious features
     */
     const float m_perceptual_range;

    /**
     * @brief abstract class constructor to instantiate member variables
     * only accessible to derived class
     */
    RobotManager2D(struct Pose2D init_pose,
                   struct VelocityCommand2D init_cmd,
                   Eigen::Matrix2f meas_noise, float robot_pr,
                   Eigen::Matrix3f process_noise) : m_curr_pose(init_pose),
        m_curr_command(init_cmd), m_meas_noise(meas_noise), m_perceptual_range(robot_pr),
        m_process_noise(process_noise){
        m_curr_obs = { .range_m = 0, .bearing_rad = 0 };
    }

    /**
     * @brief virtual destructor, necessary for accessing derived class through base class pointer
     */
    virtual ~RobotManager2D() {};

public:
    /**
     * @brief provide an update on robot trajectory given last pose
     * @details this function is probabilistic, and the result is a sample from the motion model distribution
     *
     * @return a 2D pose struct sampled from motion model distribution
     */
    virtual struct Pose2D motionUpdate() = 0;

    /**
     * @brief predicts the measurement based on sensor's measurement model and robot trajectory
     *
     * @param[in] mu_prev: previous landmark location belief
     * @return a predicted landmark observation measurement with special ID
     */
    virtual struct Observation2D predictMeas(const struct Point2D& mu_prev) = 0;

    /**
     * @brief use provided measurement and robot pose to infer landmakr location
     *
     * @param[in] rob_pose: current robot pose; note that the caller is responsible to provide a pose
     * @param[in] curr_obs: current landmark observation
     * @return a deduced landmark location in global frame
     */
    virtual struct Point2D inverseMeas(const struct Pose2D& rob_pose,
                                       const struct Observation2D& curr_obs) const = 0;

    /**
     * @brief get the latest robot observation
     * @return a bearing-range measurement
     */
    virtual struct Observation2D getCurrObs() const = 0;

    /**
     * @brief calculates the jacobian of the measurement function
     *
     * @param[in] mu_prev: previous landmark position belief
     * @return a 2x2 matrix with eq conditions plugged in
     */
    virtual Eigen::Matrix2f measJacobian(const struct Point2D& mu_prev) const = 0;

    /**
     * @brief get current robot measurement noise
     * @return a 2x2 sensor covariance matrix
     */
    virtual Eigen::Matrix2f getMeasNoise() const = 0;

    /**
     * @brief get current robot process(pose) noise
     * @return a 2x2 motion sensor covariance matrix
     */
    virtual Eigen::Matrix3f getProcessNoise() const = 0;

    /**
     * @brief get robot landmark sensor perception range
     *
     * @return landmark sensor maximum range
     */
    virtual const float& getPerceptualRange() const {return m_perceptual_range;};

};

class Create3Manager final : public RobotManager2D {

public:

    Create3Manager() = delete;

    /**
     * @brief class constructor; robot initial conditions required
     *
     * @param[in] init_pose: robot starting pose
     * @param[in] init_cmd: robot starting command
     * @param[in] meas_noise: robot landmark sensor measurement noise (constant)
     */
    Create3Manager(struct Pose2D init_pose,
                   struct VelocityCommand2D init_cmd,
                   Eigen::Matrix2f meas_noise, float robot_pr,
                   Eigen::Matrix3f process_noise):
        RobotManager2D(init_pose, init_cmd, meas_noise, robot_pr, process_noise) {}

    /**
     * @brief default Create3 class destructor
     */
    ~Create3Manager() {};


    /**
     * @brief get IMU reading from Create3 imu via ROS2
     */
    void sampleIMU() override;

    /**
     * @brief interface with LiDAR and get range-bearing reading on landmark
     */
    void sampleLandMark() override;

    /**
     * @brief get current control signal
     */
    void sampleControl() override;

    /**
     * @brief sample from Create3 motion model distribution
     *
     * @return a 2D pose struct sampled from the create3 pose distribution
     */
    struct Pose2D motionUpdate() override;

    /**
     * @brief get current landmark observation from Create3
     *
     * @return one landmark observation
     */
    struct Observation2D getCurrObs() const override;

    /**
     * @brief use inverse Create3 lidar measurement model to deduce landmark location
     *
     * @param[in] rob_pose: current robot pose; note that the caller is responsible to provide a pose
     * @param[in] curr_obs: current landmark observation
     * @return a deduced landmark location in global frame
     */
    struct Point2D inverseMeas(const struct Pose2D& rob_pose,
                               const struct Observation2D& curr_obs) const override;

    /**
     * @brief predict landmark measurement given robot states
     * @details estimation is conditioned on previous landmark position and robot pose
     *
     * @return a "proposed" landmark observation
     */
    struct Observation2D predictMeas(const struct Point2D& mu_prev) override;

    /**
     * @brief calculates the jacobian of the create3 landmark measurement model
     *
     * @param[in] mu_prev: previous landmark position belief
     * @return a 2x2 jacobian matrix given robot pose and previous landmark pose
     */
    Eigen::Matrix2f measJacobian(const struct Point2D& mu_prev) const override;

    /**
     * @brief get Create3 sensor measurement noise
     */
    Eigen::Matrix2f getMeasNoise() const override;

    /**
     * @brief get Create3 motion measurement noise
     */
     Eigen::Matrix3f getProcessNoise() const override;
};

#ifdef USE_MOCK
class MockManager2D final: public RobotManager2D {

public:

    MockManager2D() = delete;

    MockManager2D(struct Pose2D init_pose,
                   struct VelocityCommand2D init_cmd,
                   Eigen::Matrix2f meas_noise, float robot_pr,
                   Eigen::Matrix3f process_noise):
        RobotManager2D(init_pose, init_cmd, meas_noise, robot_pr, process_noise) {}

    ~MockManager2D() {};

    void sampleIMU() override;

    void sampleLandMark() override;

    void sampleControl() override;

    struct Pose2D motionUpdate() override;

    struct Observation2D getCurrObs() const override;

    struct Point2D inverseMeas(const struct Pose2D& rob_pose,
                               const struct Observation2D& curr_obs) const override;

    struct Observation2D predictMeas(const struct Point2D& mu_prev) override;

    Eigen::Matrix2f measJacobian(const struct Point2D& mu_prev) const override;

    Eigen::Matrix2f getMeasNoise() const override;

    Eigen::Matrix3f getProcessNoise() const override;


    // test commands
    void setObs(const struct Observation2D& new_obs);

    void setControl(const struct VelocityCommand2D& new_ctrl);

    void setState(const struct Pose2D& new_pose);

};
#endif // USE_MOCK
