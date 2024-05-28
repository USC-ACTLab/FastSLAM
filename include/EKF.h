/**
 * @file EKF.h
 * @brief Defines the EKF class
*/

#pragma once

#include "core-structs.h"
#include "math-util.h"
#include "robot-manager.h"
#include "Eigen/Dense"
#include <memory.h>

/**
 * @brief: Abstract EKF class that offers two core EKF functions
*/
class EKFBase {

public:

    /**
     * @brief updates internal state and covariance based on motion model
     * @details this function calculates the internal beliefs of the state and covariance
     */
    virtual void predict() = 0;

    /**
     * @brief corrects internal state and covariane based on measurement
     */
    virtual void update() = 0;

};

class LandMarkEKF: public EKFBase {

public:

    /**
     * @brief calculate measurement's likelihood of correspondence to the landmark
     * @details this function utilizes Maximum Likelihood Estimation (MLE) and is
     * used to solve the data association problem
     */
    virtual float calcCPD() = 0;

};

class LMEKF2D: public LandMarkEKF{

private:

    Eigen::Matrix2f m_mu;

    Eigen::Matrix2f m_sigma;

    std::shared_ptr<RobotManager> m_robot;

    Eigen::Matrix2f predictJacobian() const;

    Eigen::Matrix2f measJacobian() const;

    float calcKalmanGain() const;


public:

    LMEKF2D();

    ~LMEKF2D();

    void predict() override;

    void update() override;

    float calcCPD() override;

};
// candidate functions:
