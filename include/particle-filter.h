/**
 * @file particle-filter.h
 * @brief The ParticleFilter class represents a particle filter, as described
 *    by the Probabilistic Robotics textbook (Thrun, Burgard, and Fox 2005).
 */

#pragma once

#include <vector>

/**
 * @brief Represents the SLAM posterior as a set of particles.
 * 
 * @tparam  MotionCommand_T   Generic type for a motion command
 * @tparam  Observation_T     Generic type for an observation
 * @tparam  Particle_T        Generic type for a particle
 */
template <class MotionCommand_T, class Observation_T, class Particle_T>
class ParticleFilter {

   public:

      /**
       * @brief Constructor for the ParticleFilter class.
       * 
       * @param[in]     aNumParticles     Initial number of particles created
       */
      ParticleFilter( const int aNumParticles );

      /**
       * @brief Updates the posterior given a new timestep of sensor information.
       * 
       * @param[in]     aCommand       Motion command preceding current state
       * @param[in]     aObservation   Observation from current state
       */
      void updatePosterior( const MotionCommand_T& aCommand,
                            const Observation_T& aObservation );

   private:

      /** Vector of particles, each an independent "guess" of the posterior. */
      std::vector<Particle_T> _mParticles;
};