/**
 * @file probability.cpp
 * @brief Implements common probability distributions and their samplers.
 */

#include "math-util.h"

std::pair<float, float> MathUtil::worldToBody2D( const std::pair<float,
                                                                 float>& aWorld_quant,
                                                 const float aTheta_rad ){

   float x_body = cosf(aTheta_rad) * aWorld_quant.first + sinf(aTheta_rad) *
                  aWorld_quant.second;
   float y_body = -sinf(aTheta_rad) * aWorld_quant.first + cosf(aTheta_rad) *
                  aWorld_quant.second;
   return std::make_pair(x_body, y_body);

}

std::pair<float, float> MathUtil::bodyToWorld2D( const std::pair<float,
                                                                 float>& aBody_quant,
                                                 const float aTheta_rad ){

   float x_world = cosf(aTheta_rad) * aBody_quant.first - sinf(aTheta_rad) *
                   aBody_quant.second;
   float y_world = sinf(aTheta_rad) * aBody_quant.first + cosf(aTheta_rad) *
                   aBody_quant.second;
   return std::make_pair(x_world, y_world);

}

float MathUtil::sampleNormal( const float aMean, const float aVariance ){

   if (aVariance < 0.0) return std::numeric_limits<double>::quiet_NaN();

   std::random_device rd{};
   std::mt19937 gen{rd()};
   std::normal_distribution d{aMean, sqrtf(aVariance)};

   return d(gen);
}
