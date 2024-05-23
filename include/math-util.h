/**
 * @file probability.h
 * @brief Defines common probability distributions and their samplers.
 */

#pragma once

#include <vector>
#include <utility>
#include <random>
#include <cmath>
#include "core-structs.h"

/**
 * @brief Namespace for common math utility functions.
 */
namespace MathUtil{


/**
 * @brief Transforms a 2d quantity from world frame to body frame
 * @details Multiplies the 2d vector with rotational matrix given the heading in radians.
 * World Frame Coordinate: front x, right y, clockwise rotation from x to y as positive
 *
 * @param[in]    aWorld_pose   2D quantity in world frame
 * @param[in]    aTheta_rad    Robot heading angle in randians
 *
 * @returns quantity in robot frame
 */
std::pair<float, float> worldToBody2D( const std::pair<float, float>& aWorld_quant, const float aTheta_rad);


/**
 * @brief Transforms a 2d quantity from robot frame to world frame
 * @details Multiplies the 2d vector with inverse rotational matrix given the heading in radians
 * Body Frame coordinate: front x, right y, clockwise rotation from x to y as positive
 *
 * @param[in]    aBody_pose   2D quantity in robot frame
 * @param[in]    aTheta_rad    Robot heading angle in randians
 *
 * @returns quantity in world frame
 */
std::pair<float, float> bodyToWorld2D(const std::pair<float, float>& aBody_quant, const float aTheta_rad);

/**
 * @brief Generates a random sample from the Gaussian: N(mean, variance).
 *
 * @details Reference: Probabilistic Robotics, Ch. 5.3, Table 5.4 (pg. 124).
 *    Further reading from Winkler (1995; pg. 293) may be advised.
 *    C++ has a built in normal distribution template, see:
 *    https://en.cppreference.com/w/cpp/numeric/random/normal_distribution
 *
 *    TODO: Ensure we're talking about stddev and variance correctly,
 *       because ProbRob has a typo somewhere in Table 5.4. Beware!
 *
 * @param[in]     aMean       Mean of the sampled Normal distribution
 * @param[in]     aVariance   Variance of the sampled Normal distribution
 *
 * @returns Random value, distributed according to the defined Gaussian
 */
float sampleNormal( const float aMean, const float aVariance );


}; // namespace MathUtil
