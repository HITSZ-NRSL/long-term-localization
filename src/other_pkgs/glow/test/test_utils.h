#ifndef TEST_TEST_UTILS_H_
#define TEST_TEST_UTILS_H_

#include <random>

namespace {
class Random {
 public:
  Random() : uniform_(0.0, 1.0), normal_(0.0, 1.0) {}
  explicit Random(uint32_t seed) : rng_(seed), uniform_(0.0, 1.0), normal_(0.0, 1.0) {}

  /** \brief Returns a random integer value in the range [0,\a n - 1] inclusive. */
  int32_t getInt(int32_t n) { return static_cast<int32_t>(n * uniform_(rng_)); }
  /** \brief Returns a random integer value in the range [\a min,\a max] inclusive. */
  int32_t getInt(int32_t min, int32_t max) { return static_cast<int32_t>((max - min) * uniform_(rng_) + min); }

  /** \brief Function operator for use with STL.
   * Returns a random value between 0 and \a n - 1 inclusive.
   */
  int32_t operator()(int32_t n) { return getInt(n); }

  /** \brief Returns a random float in the range [0,1] inclusive. */
  float getFloat() { return static_cast<float>(uniform_(rng_)); }

  /** \brief Returns a random float from the Gaussian distribution with mean 0 and std. deviation 1 */
  float getGaussianFloat() { return static_cast<float>(uniform_(rng_)); }
  /** \brief Returns a random double in the range [0,1] inclusive. */
  double getDouble() { return uniform_(rng_); }

  /** \brief Returns a random double from the Gaussian distribution with mean 0 and std. deviation 1 */
  double getGaussianDouble() { return normal_(rng_); }

 protected:
  std::default_random_engine rng_;
  std::uniform_real_distribution<double> uniform_;
  std::normal_distribution<double> normal_;
};
}

#endif /* TEST_TEST_UTILS_H_ */
