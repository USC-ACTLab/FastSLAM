/**
 * @file EKF.h
 * @brief Defines the EKF class
 */

#pragma once

#include "core-structs.h"
#include "math-util.h"
#include "robot-manager.h"
#include "Eigen/Dense"
#include <memory>

enum class KF_RET { SUCCESS = 0, EMPTY_ROBOT_MANAGER = -1, MATRIX_INVERSION_ERROR = -2 };

/**
 * @brief: Abstract EKF class that offers two core EKF functions
 */
class EKFBase {

public:

   /**
    * @brief updates internal state and covariance based on motion model
    * @details this function calculates the internal beliefs of the state and covariance
    */
   virtual KF_RET predict() = 0;

   /**
    * @brief corrects internal state and covariane based on measurement
    * @details this function updates both the mean and the covariance based on the filter gain
    */
   virtual KF_RET update() = 0;

};

class LandMarkEKF : public EKFBase {

public:

   /**
    * @brief calculate measurement's likelihood of correspondence to the landmark
    * @details this function utilizes Maximum Likelihood Estimation (MLE) and is
    * used to solve the data association problem
    */
   virtual float calcCPD() = 0;

   /**
    * @brief virtual desturctor, necessary for accessing derived class through base class pointer
    */
   virtual ~LandMarkEKF() { };

};

class LMEKF2D final: public EKFBase {

private:

   /**
    * @brief 2D landmark coordinate estimate
    */
   struct Point2D m_mu;

   /**
    * @brief landmark estimate covariance matrix
    */
   Eigen::Matrix2f m_sigma;

   /**
    * @brief shared pointer to the robot manager instance, used to access measurement models
    */
   std::shared_ptr<RobotManager2D> m_robot;

   /**
    * @brief measurement covariance matrix, used for correspondence weight and K-gain calculation
    */
   Eigen::Matrix2f m_meas_cov;

   /**
    * @brief local copy of current robot observation
    */
   struct Observation2D m_curr_obs;

   /**
    * @brief measurement jacobian helper function, takes the Jacobian of g(s, mu)
    */
   Eigen::Matrix2f measJacobian() const;

   /**
    * @brief function to calculate Kalman Filter gain for measurement correction
    *
    * @return A 2x2 matrix of Kalman Filter gain
    */
   Eigen::Matrix2f calcKalmanGain() const;

   /**
    * @brief calculate measurement covariance, store result in member variable
    */
   void calcMeasCov();

public:

   /**
    * @brief default constructor, initialize mu and sigma to 0, robot to nullptr
    */
   LMEKF2D();

   /**
    * @brief parametrized constructor
    *
    * @param[in] init_obs: landmark starting location (initial observation)
    * @param[in] init_cov: P_0, aka initial covariance/uncertainty
    * @param[in] robot_ptr: shared pointer to the robot manager instance
    */
   LMEKF2D(struct Point2D init_obs, Eigen::Matrix2f init_cov,
           std::shared_ptr<RobotManager2D> robot_ptr);

   /**
    * @brief copy assignment operator, used in the particle filter
    * when drawing particles
    *
    * @param[in] ekf: EKF to be copied from
    */
    LMEKF2D(const LMEKF2D& ekf);

   /**
    * @brief advances internal beliefs of 2D Landmark using the prediction model
    * @details no-op in Landmark EKF, since target is non dynamic
    */
   KF_RET predict() override {
      return KF_RET::SUCCESS;
   }

   /**
    * @brief collects new measurement and update internal beliefs based on measurement
    */
   KF_RET update() override;

   /**
    * @brief calculates likelihood of correspondence given the measurement and prediction
    * @details we are not using robot manager to get measurement here to guarantee the timing of measurements
    * @return float; scalar value measuring likelihood that the observation matches internal state
    */
   float calcCPD();

   /**
    * @brief returns current landmark position estimate in world frame
    * @return a 2D point struct with landmark x and y position
    * */
   const struct Point2D& getLMEst() const;

   /**
    * @brief update internal copy of current robot observations
    * @details this ensures timing in constrast to the sampling approach. Must be called first every cycle
    */
    void updateObservation(const struct Observation2D& new_obs);

};
