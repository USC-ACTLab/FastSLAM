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
#include "particle-filter.h"

/**
 * @brief Implements FastSLAM 1.0, as described by Montemerlo et al. (2002).
 */
class FastSLAM {

public:

   /**
    * @brief Constructor for the FastSLAM class.
    *      TODO: What's needed here?
    */
   FastSLAM( MotionModel& aMotionModel );

   /**
    * @brief Samples a proposal distribution by simulating noisy motion.
    *
    * @details Progresses (in-place) each posterior particle's state estimate.
    *
    *   Formally, we're sampling a new state (s_t^[m]) based on the previous
    *   state estimate of each particle (s_t-1^[m]), using motion command u_t.
    *
    *   TLDR: Noisily samples a new s_t^[m] from p(s_t | s_t-1^[m], u_t).
    *
    * @param[in]    aCommand    Velocity command used in motion simulation
    */
   void sampleProposal( const VelocityCommand2D& aCommand );

   /**
    * @brief Computes the most-likely data association for the given particle.
    *
    * @details Updates or populates the particle's stored data association.
    *
    *   Formally, we're finding whichever landmark (theta), with some unique
    *   identifier (n_t^i,[m]), is most likely to correspond with the current
    *   observation (z_t^i), given our knowledge of the robot state (s_t^[m])
    *   and that particular landmark (theta_{n_t^i,[m]}).
    *
    *   If the maximum likelihood is below some threshold (alpha), we consider
    *       the observed landmark to be new and add it to the particle's map.
    *
    *   TLDR: Find n_t^i to maximize p(z_t^i | s_t^[m], theta_{n_t^i}).
    *
    * @param[in]    aParticle       Particle with unknown data association
    * @param[in]    aObservation    Robot's current observation of a landmark
    */
   void computeDataAssociation( Particle& aParticle,
                                const Observation2D& aObservation );

   /**
    * @brief Updates a particle's landmark posterior based on an observation.
    *
    * @details Assumes known landmark-observation correspondence for the given
    *   particle (recall: number of landmarks may differ across particles).
    *
    *   Formally, updates the landmark posterior p(theta_k) using an EKF. This
    *   operation will occur for each particle, with each observation, every
    *   update step (i.e., timestep) within the overall FastSLAM algorithm.
    *
    *   TODO: We need to write the EKF class underlying this particular step.
    *
    * @param[in/out]    aParticle       Particle with landmark posterior(s) to update
    * @param[in]        aObservation    Observation with known correspondence
    */
   void updateLandmarks( Particle& aParticle,
                         const Observation2D& aObservation );


   /**
    * @brief Computes an importance weight for the given particle.
    *
    * @details Formally, the importance weight w_t^[m] roughly corresponds
    *   to... TODO!
    *
    *  probability that a particular particle's state and landmark estimates
    */
   void weightImportance( Particle& aParticle );

   /**
    * @TODO: This class needs to implement the following:
    * 4. Compute importance weights for each sampled particle.
    * 5. Resample (with replacement) new particles based on the weights.
    */

}; // class FastSLAM