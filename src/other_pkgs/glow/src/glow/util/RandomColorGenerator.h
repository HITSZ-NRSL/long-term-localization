#ifndef SRC_UTIL_RANDOMCOLORGENERATOR_H_
#define SRC_UTIL_RANDOMCOLORGENERATOR_H_

#include <glow/GlColor.h>
#include <random>

namespace glow {

/** \brief random colors generated from a RNG.
 *
 *  \author behley
 */
class RandomColorGenerator {
 public:
  RandomColorGenerator(uint32_t seed = 0);

  void reset();

  /** \brief generates a random color **/
  GlColor nextColor();

 protected:
  uint32_t seed;
  std::default_random_engine rng_;
  std::uniform_real_distribution<double> uniform_;
};
}
#endif /* SRC_UTIL_RANDOMCOLORGENERATOR_H_ */
