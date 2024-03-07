/**
 * @file motion-model.h
 * @brief The MotionModel class models how the robot moves through the world.
 */

#pragma once

#include "core-structs.h"

/**
 * @brief Provides models for the robot's kinematics, with and without noise.
 */
class MotionModel {

public:

   /**
    * @brief Constructor for the MotionModel class.
    *    TODO: Is anything actually needed here? The alpha noise terms!
    *    TODO: Design: Who owns the timestep? Should it vary per-update?
    */
   MotionModel( void );

   /**
    * @brief Simulates robot motion using a motion command (without noise).
    *
    * @details This implementation is in-place: it updates the given state!
    *    Reference: Probabilistic Robotics, Ch. 5.3, Eq. 5.9 (pg. 127)
    *
    * @param[in/out]    aState         Robot state from which motion begins
    * @param[in]        aCommand       Motion command defining the motion
    * @param[in]        aTimestep_s    Duration of the motion (seconds)
    */
   void MotionModel::simulateExact( Pose2D& aState,
                                    const VelocityCommand2D& aCommand,
                                    const float aTimestep_s );

   /**
    * @brief Simulates robot motion using a motion command (with noise).
    *
    * @details This implementation is in-place: it updates the given state!
    *    Reference: Probabilistic Robotics, Ch. 5.3, Table 5.3 (pg. 124)
    *
    * @param[in/out]    aState         Robot state from which motion begins
    * @param[in]        aCommand       Motion command defining nominal motion
    * @param[in]        aTimestep_s    Duration of the motion (seconds)
    */
   void MotionModel::simulateNoisy( Pose2D& aState,
                                    const VelocityCommand2D& aCommand,
                                    const float aTimestep_s );

};