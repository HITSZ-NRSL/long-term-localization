#include "RandomColorGenerator.h"
#include <cmath>

namespace glow {

RandomColorGenerator::RandomColorGenerator(uint32_t _seed) : seed(_seed), rng_(seed) {}

void RandomColorGenerator::reset() { rng_ = std::default_random_engine(seed); }

GlColor RandomColorGenerator::nextColor() { return GlColor::FromHSV(2.0f * M_PI * uniform_(rng_), 1.0f, 1.0f); }
}
