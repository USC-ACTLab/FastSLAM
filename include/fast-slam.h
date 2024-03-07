/**
 * @file fast-slam.h
 * @brief The FastSLAM class implements the FastSLAM 1.0 algorithm, as described
 *      by Montemerlo et al. (2002), using the original problem setting.
 *
 * @details Following the original paper, we assume the following:
 *      - Robot pose for a 2D planar robot: (x, y, heading)
 *      - Landmarks are 2D locations: (x, y)
 *      - Observations are range-bearing-signature triplets: (r, bearing, s)
 */

#pragma once

#include "core-structs.h"
#include "motion-model.h"
#include "measurement-model.h"

/**
 * @brief Implements FastSLAM 1.0, as described by Montemerlo et al. (2002).
 */
class FastSLAM {

public:

   /**
    * @brief Each particle represents a hypothesis of the true posterior.
    */
   struct Particle2D {
      Pose2D robotState;                  // Robot pose estimate
      std::vector<Gaussian2D> landmarks;  // Posteriors over landmark locations
   };

   /**
    * @brief Constructor for the FastSLAM class.
    *      TODO: What's needed here?
    */
   FastSLAM( MotionModel& aMotionModel, MeasurementModel& aMeasurementModel );

   /**
    * @brief Runs a single FastSLAM 1.0 update using the given sensor data.
    *
    * @details Updates the posterior contained in the FastSLAM class.
    *
    * @param[in]    aMotionCommand  Velocity command preceding the timestep
    * @param[in]    aObservation    Current landmark observations
    */
   void update( const VelocityCommand2D& aMotionCommand,
                const Observation2D& aObservation );

   /**
    * @brief Samples a proposal distribution by simulating noisy motion.
    *
    * @details Progresses (in-place) each posterior particle's state estimate.
    *
    *   Formally, we're sampling a new state (s_t^[m]) based on the previous
    *   state estimate of each particle (s_t-1^[m]), using motion command u_t.
    *
    *   TLDR: Samples new particles s_t^[m] ~ p(s_t | s_t-1^[m], u_t).
    *
    * @param[in]    aCommand    Velocity command used in motion simulation
    */
   void sampleProposal( const VelocityCommand2D& aCommand );

   /**
    * @brief Computes the most-likely data association for the given particle.
    *
    * @details Formally, we're finding the landmark (theta, with ID n_t^i,[m])
    *   that's most likely to correspond with the current observation (z_t^i),
    *   given some particle's estimated robot state (s_t^[m]) and landmark map.
    *
    *   If the maximum likelihood is below some threshold (alpha), we consider
    *       the observed landmark to be new and add it to the particle's map.
    *
    *   TLDR: Find n_t^i,[m] to maximize p(z_t^i | s_t^[m], theta_{n_t^i}).
    *
    * @param[in]        aParticle       Particle containing a robot state estimate
    * @param[in/out]    aObservation    Current observation of some landmark
    *
    * @returns Map index of the most-likely observation-corresponding landmark
    */
   int computeDataAssociationMLE( const Particle2D& aParticle,
                                  const Observation2D& aObservation );

   /**
    * @brief Updates a particle's landmark posterior(s) given an observation.
    *
    * @details Assumes known landmark-observation correspondence for the given
    *   particle (recall: number of landmarks may differ across particles).
    *
    *   Formally, updates the landmark posterior p(theta_k) using an EKF. This
    *   operation will occur for each particle, with each observation, every
    *   update step (i.e., timestep) for the overall FastSLAM algorithm.
    *
    *   TODO: We need to write the EKF class underlying this particular step.
    *
    * @param[in/out]    aParticle       Particle with landmark(s) to update
    * @param[in]        aObservation    Observation with known correspondence
    */
   void updateLandmarks( Particle2D& aParticle,
                         const Observation2D& aObservation );

   /**
    * @brief Computes the importance weight for the given particle.
    *
    * @details Formally, the importance weight w_t^[m] represents the particle's
    *   likelihood based on the observed landmarks in the current timestep.
    *
    * Reference: Probabilistic Robotics, Ch. 13.3, Table 13.1, Line 18 (pg. 450)
    *   TODO: Seems likely to depend on the EKF or Gaussian classes.
    *
    * @param[in]    aParticle       Particle to be importance-weighted
    * @param[in]    aObservation    Observation for the current timestep
    *
    * @returns Importance weight w_t^[m] for the given particle.
    */
   double weightImportance( const Particle2D& aParticle,
                            const Observation2D& aObservation );

private:

   /** SLAM posterior represented using a set of particles ("hypotheses") */
   std::vector<Particle2D>  _mParticles;

}; // class FastSLAM