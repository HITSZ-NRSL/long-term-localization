#include <cmath>

#include "GlColor.h"
#include "GlUniform.h"

namespace glow {

const GlColor GlColor::WHITE = GlColor(1.0f, 1.0f, 1.0f);
const GlColor GlColor::GRAY = GlColor(0.5f, 0.5f, 0.5f);
const GlColor GlColor::BLACK = GlColor(0.0f, 0.0f, 0.0f);
const GlColor GlColor::RED = GlColor(1.0f, 0.0f, 0.0f);
const GlColor GlColor::GREEN = GlColor(0.0f, 1.0f, 0.0f);
const GlColor GlColor::BLUE = GlColor(0.0f, 0.0f, 1.0f);
const GlColor GlColor::YELLOW = GlColor(1.0f, 1.0f, 0.0f);
const GlColor GlColor::PINK = GlColor(1.0f, 0.0f, 1.0f);
const GlColor GlColor::ORANGE = GlColor(1.0f, 0.65f, 0.0f);
const GlColor GlColor::CYAN = GlColor(0.0f, 1.0f, 1.0f);
const GlColor GlColor::GOLD = GlColor(0.85f, 0.65, 0.13f);

template <>
void GlUniform<GlColor>::bind(GLuint program_id) const {
  GLint loc = glGetUniformLocation(program_id, name_.c_str());
  //  assert(loc >= 0 && "Warning: Uniform unknown or unused in program.");
  glUniform4fv(loc, 1, &data_.R);
}

GlColor GlColor::FromRGB(uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
  return GlColor((float)r / 255.f, (float)g / 255.f, (float)b / 255.f, (float)a / 255.f);
}

void GlColor::toHSV(float& h, float& s, float& v, float& a) {
  float M = std::max(R, std::max(G, B));
  float m = std::min(R, std::min(G, B));
  float C = M - m;
  float sixty = 1.04719755f;

  h = s = v = 0.0f;
  if (M == R)
    h = std::fmod((G - B) / C, 6.0f);
  else if (M == G)
    h = (B - R) / C + 2.0f;
  else if (M == B)
    h = (R - G) / C + 4.0f;

  h = sixty * h;
  v = M;
  if (v > 0.0f) s = C / v;
  a = A;
}

void GlColor::setHSV(float h, float s, float v, float a) {
  R = G = B = 0.0f;
  A = a;

  if (std::abs(s) < 0.00001) {
    R = G = B = v;
    return;
  }

  float sixty = 1.04719755f;           // == 60 deg
  int32_t h_i = std::floor(h / sixty); /** integer part of h **/
  float f = h / sixty - h_i;           /** fractional part of h **/
  float p = v * (1 - s), q = v * (1 - s * f), t = v * (1 - s * (1 - f));

  switch (h_i) {
    case 0:
    case 6:
      R = v;
      G = t;
      B = p;
      break;
    case 1:
      R = q;
      G = v;
      B = p;
      break;
    case 2:
      R = p;
      G = v;
      B = t;
      break;
    case 3:
      R = p;
      G = q;
      B = v;
      break;
    case 4:
      R = t;
      G = p;
      B = v;
      break;
    case 5:
      R = v;
      G = p;
      B = q;
      break;
  }
}

GlColor GlColor::FromHSV(float h, float s, float v, float a) {
  GlColor col;
  col.setHSV(h, s, v, a);

  return col;
}

void GlColor::lighter(float factor) {
  float v = std::pow(1.0 / 0.7, factor);
  R *= v;
  G *= v;
  B *= v;
}

void GlColor::darker(float factor) {
  float v = std::pow(0.7, factor);
  R *= v;
  G *= v;
  B *= v;
}
}
