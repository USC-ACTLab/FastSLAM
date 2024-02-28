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

/**
 * @brief Implements FastSLAM 1.0, as described by Montemerlo et al. (2002).
 */
class FastSLAM {

    public:

        /**
         * @brief Each particle represents a hypothesis of the true posterior.
         */
        struct Particle {
            Pose2D robotState;                  // Robot pose estimate
            std::vector<Gaussian2D> landmarks;  // Landmark location posteriors
        };

        /**
         * @brief Constructor for the FastSLAM class.
         *      TODO: What's needed here?
         */
        FastSLAM( void );

        /**
         * @brief Samples a proposal distribution by simulating noisy motion.
         */
        void sampleProposal( const VelocityCommand2D& aCommand );

   /**
    * @TODO: This class needs to implement the following:
    * 
    * 1. Create a proposal distribution by sampling using the motion model.
    *       - Does this function exist?
    * 2. Compute per-particle data associations using measurement likelihood.
    *       - Does the measurement likelihood exist?
    * 3. Update posteriors of each landmark, using their EKFs.
    *       - What functions does the EKF need?
    * 4. Compute importance weights for each sampled particle.
    * 5. Resample (with replacement) new particles based on the weights.
    */

}; // class FastSLAM