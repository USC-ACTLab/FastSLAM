/**
 * @file probability.cpp
 * @brief Implements common probability distributions and their samplers.
 */

#include "probability.h"

/**
 * @brief Generates a random sample from the Gaussian: N(mean, variance).
 *
 * @details Reference: Probabilistic Robotics, Ch. 5.3, Table 5.4 (pg. 124).
 *    Further reading from Winkler (1995; pg. 293) may be advised.
 *
 *    TODO: Ensure we're talking about stddev and variance correctly,
 *       because ProbRob has a typo somewhere in Table 5.4. Beware!
 *
 * @param[in]     aMean       Mean of the sampled Normal distribution
 * @param[in]     aVariance   Variance of the sampled Normal distribution
 *
 * @returns Random value, distributed according to the defined Gaussian
 */
double Probability::sampleNormal( const double aMean, const double aVariance ) {

   // TODO

   return 0.0;
}

/**
 * @brief Computes the value's probability from the Gaussian: N(0, variance).
 *
 * @details Reference: Probabilistic Robotics, Ch. 5.3, Table 5.2 (pg. 123)
 *    Any reference on Gaussian distributions would do just as well.
 *
 *    TODO: Review whether we should pass a variance or a standard dev?
 *
 * @param[in]  aValue      Observed value sampled from the specified Gaussian
 * @param[in]  aVariance   Variance of the Gaussian distribution of the sample
 *
 * @returns Approximate probability of the observed sample from the Gaussian
 */
double Probability::calcGaussianProbability( const double aValue,
                                             const double aVariance ) {

   // TODO

   return 0.0;
}

/**
 * @brief Samples a vector (with replacement) using a low-variance strategy.
 *
 * @details New samples are included with probability proportional to their
 *    weight, with repeats allowed. This sampler is efficient: O(particles).
 *
 *    Reference: Probabilistic Robotics, Ch. 4.3, Table 4.4 (pg. 110)
 *
 * @tparam     Particle_T           Type of the particle being re-sampled
 *
 * @param[in]  aWeightedParticles   Vector of generic [particle, weight] pairs
 *
 * @returns Newly sampled vector with probabilities based on given weights
 */
template<typename Particle_T>
std::vector<Particle_T> Probability::resampleLowVariance(
   const std::vector< std::pair<Particle_T, double> >& aWeightedParticles ) {

   std::vector<Particle_T> output;

   // TODO: For now, copy the given particles into the output vector
   for ( const auto & WeightedParticle : aWeightedParticles ) {
      output.push_back(WeightedParticle.first);
   }

   return output;
}
