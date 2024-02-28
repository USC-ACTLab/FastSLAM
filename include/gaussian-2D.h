/**
 * @file gaussian-2D.h
 * @brief Implements a two-dimensional Gaussian distribution using Eigen3.
 */

#pragma once

#include <Eigen/Core>

/**
 * @brief The Gaussian2D class implements a two-dimensional Gaussian.
 */
class Gaussian2D {

   public:

      /**
       * @brief Constructor for the Gaussian2D class.
       * 
       * @param[in]     aDefaultMean      Initial mean vector of the Gaussian
       */
      Gaussian2D( const Eigen::Vector2d& aDefaultMean );
   
   private:

      /** Mean vector (2 x 1) of the distribution. */
      Eigen::Vector2d _mMeanVector;

      /** Covariance matrix (2 x 2) representing uncertainty in the distribution. */
      Eigen::Matrix2d _mCovarianceMatrix;
};