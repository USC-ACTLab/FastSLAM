/**
 * @file measurement-model.h
 * @brief The MeasurementModel class models a robot's sensor observations.
 */

#pragma once

#include "core-structs.h"
#include "probability.h"

/**
 * @brief Models the robot's sensor observations of surrounding landmarks.
 */
class MeasurementModel {

public:

   /**
    * @brief Constructor for the MeasurementModel class.
    *    TODO: Anything needed here?
    */
   MeasurementModel( void );

   /**
    * @brief Computes the likelihood of an observation of a particular landmark.
    *
    * @param[in]  robotPose      Pose of the robot when the observation was made
    * @param[in]  measurement    Sensor measurement of the observed landmark
    * @param[in]  landmark       Nominal location of the landmark
    *
    * @return Likelihood of the measurement, given the robot state and landmark
    */
   double computeLikelihood( const Pose2D& robotPose,
                             const Observation2D& measurement,
                             const Point2D& landmark );

};