/**
 * @file EKF.cpp
 * @brief Implements the EKF derived classes
 */

#include "EKF.h"


LMEKF2D::LMEKF2D() {

   m_mu = { .x = 0, .y = 0 };

   m_sigma = Eigen::Matrix2f::Zero();

   m_robot = nullptr;

}

LMEKF2D::LMEKF2D(struct Point2D init_obs, Eigen::Matrix2f init_cov,
                 std::shared_ptr<RobotManager2D> robot_ptr) : m_mu(init_obs),
   m_sigma(init_cov),
   m_robot(robot_ptr) {
}

LMEKF2D::~LMEKF2D() {
   m_robot.reset();
}

Eigen::Matrix2f LMEKF2D::measJacobian() const {
    if (m_robot == nullptr) {
        return Eigen::Matrix2f::Zero();
    }
    return m_robot->measJacobian(m_mu); // make sure this is called before update()
}

void LMEKF2D::calcMeasCov() {
    Eigen::Matrix2f G_n = this->measJacobian();
    m_meas_cov = G_n.transpose() * m_sigma * G_n + m_robot->getMeasNoise();
}

Eigen::Matrix2f LMEKF2D::calcKalmanGain() const {
    if (m_robot == nullptr || m_meas_cov.determinant() == 0) {
        return Eigen::Matrix2f::Zero();
    }
    return m_sigma * this->measJacobian() * (m_meas_cov.inverse());
}

KF_RET LMEKF2D::update() {
    if (m_robot == nullptr) {
        return KF_RET::EMPTY_ROBOT_MANAGER;
    }

    if (m_meas_cov.determinant() == 0) {
        return KF_RET::MATRIX_INVERSION_ERROR;
    } else {
        Eigen::Matrix2f K = this->calcKalmanGain();
        Eigen::Matrix2f G_n = this->measJacobian();

        m_mu += K * ( m_robot->getCurrObs() - m_robot->predictMeas(m_mu) );
        m_sigma = ( Eigen::Matrix2f::Identity() - K * G_n ) * m_sigma;
    }

    return KF_RET::SUCCESS;
}

float LMEKF2D::calcCPD() {
    if (m_robot == nullptr) {
        return -1.0f;
    }

    this->calcMeasCov();
    if (m_meas_cov.determinant() == 0) {
        return -1.0f;
    }

    Eigen::Vector2f residue = m_robot->getCurrObs() - m_robot->predictMeas(m_mu);
    float weight = sqrtf( (2 * M_PI * m_meas_cov).determinant() );
    weight = weight * expf( -0.5 * residue.transpose() * m_meas_cov.inverse() * residue );

    return weight;
}

struct Point2D LMEKF2D::getLMEst() const {
   return m_mu;
}
