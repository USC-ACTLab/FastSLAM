/**
 * @file math-util.h
 * @brief Defines common probability distributions and their samplers.
 */

#pragma once

#include <vector>
#include <utility>
#include <random>
#include <cmath>
#include <limits.h>
#include "core-structs.h"

/**
 * @brief Namespace for common math utility functions.
 */
namespace MathUtil {


/**
 * @brief Transforms a 2d quantity from world frame to body frame
 * @details Multiplies the 2d vector with rotational matrix given the heading in radians.
 * World Frame Coordinate: front x, right y, clockwise rotation from x to y as positive
 *
 * @param[in]    aWorld_quant  2D quantity in world frame
 * @param[in]    aTheta_rad    Robot heading angle in randians
 *
 * @return quantity in robot frame
 */
std::pair<float, float> worldToBody2D( const std::pair<float, float>& aWorld_quant,
                                       const float aTheta_rad);

/**
 * @brief Transforms a 2d quantity from robot frame to world frame
 * @details Multiplies the 2d vector with inverse rotational matrix given the heading in radians
 * Body Frame coordinate: front x, right y, clockwise rotation from x to y as positive
 *
 * @param[in]    aBody_pose   2D quantity in robot frame
 * @param[in]    aTheta_rad    Robot heading angle in randians
 *
 * @return quantity in world frame
 */
std::pair<float, float> bodyToWorld2D(const std::pair<float, float>& aBody_quant,
                                      const float aTheta_rad);

/**
 * @brief Generates a random sample from the Gaussian: N(mean, variance).
 *
 * @details Reference: Probabilistic Robotics, Ch. 5.3, Table 5.4 (pg. 124).
 *    Further reading from Winkler (1995; pg. 293) may be advised.
 *    C++ has a built in normal distribution template, see:
 *    https://en.cppreference.com/w/cpp/numeric/random/normal_distribution
 *
 * @param[in]     aMean       Mean of the sampled Normal distribution
 * @param[in]     aVariance   Variance of the sampled Normal distribution
 *
 * @return Random value, distributed according to the defined Gaussian
 */
float sampleNormal( const float aMean, const float aVariance );

/**
 * @brief Generates a random sample from the Uniform [min, max)
 *
 * @param[in] aMin: minimum value of the range
 * @param[in] aMax: maximum value of the range
 *
 * @return Random value, distributed according to the defined Uniform
 */
float sampleUniform( const float& aMin, const float& aMax );

/**
 * @brief Finds the distance between two points
 *
 * @param[in] aPointA: a 2D point
 * @param[in] aPointB: a 2D point
 * @return the absolute distance between two points
 */
float findDist( const struct Point2D& aPointA, const struct Point2D& aPointB );

/**
 * @brief overloaded version of findDist, used specifically for robot and landmark
 *
 * @param[in] aPointA:  landmark position
 * @param[in] aRobPose: robot pose
 * @return distance between landmark and robot
 */
float findDist( const struct Point2D& aLandMark, const struct Pose2D& aRobPose );

/**
 * @brief generate a cumulative cdf table based on pdf weights
 *
 * @param[in] aPdfVec: vector containing pdf values
 * @param[in] aTargetVec: reference to the vector containing results
 * @return sum of the all elements
 */
template< class T >
T genCDF( const std::vector<T>& aPdfVec, std::vector<T>& aTargetVec );

}; // namespace MathUtil
