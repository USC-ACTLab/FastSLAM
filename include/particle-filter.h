/**
 * @file particle-filter.h
 * @brief The ParticleFilter class represents a particle filter, as described
 *    by the Probabilistic Robotics textbook (Thrun, Burgard, and Fox 2005).
 */

#pragma once

#include <vector>
#include "core-structs.h"
#include "gaussian-2D.h"

/**
 * @brief Implements a particle filter for 2D planar robot motion.
 */
class ParticleFilter {

public:

   /**
    * @brief A particle represents a single hypothesis of the true posterior.
    */
   struct Particle2D {
      Pose2D robotState;                  // Robot pose estimate
      std::vector<Gaussian2D> landmarks;  // Posteriors over landmark locations
   };

   /**
    * @brief Constructor for the ParticleFilter class.
    *
    * @param[in]     aNumParticles     Initial number of particles created
    */
   ParticleFilter( const int aNumParticles );

   /**
    * @brief Updates the posterior given new sensor information.
    *
    * @param[in]     aCommand       Motion command preceding current state
    * @param[in]     aObservation   Observation from current state
    */
   void updatePosterior( const VelocityCommand2D& aCommand,
                         const Observation2D& aObservation );

private:

   /** Vector of particles providing a distribution over possible posteriors. */
   std::vector<Particle2D> _mParticles;
};