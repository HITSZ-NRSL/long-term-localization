#ifndef GLCOLOR_H_
#define GLCOLOR_H_

#include <cassert>
#include <stdint.h>
#include "glutil.h"

namespace glow {

/**
 * \brief GlColor represents a RGB color given by [0.0,1.0] values and an alpha value also in [0.0,1.0].
 *
 * This class represents RGB colors and provides some helper functions.
 *
 * Usage example setting color to red:
 *
 * program.setUniform(GlUniform<GlColor>("my_color", GlColor::RED));
 *
 *
 * \author behley
 */
class GlColor {
 public:
  GlColor() : R(0.0f), G(0.0f), B(0.0f), A(1.0f) {}

  GlColor(int32_t r, int32_t g, int32_t b, int32_t a = 255)
      : R(float(r) / 255.), G(float(g) / 255.), B(float(b) / 255.), A(float(a) / 255.) {}

  /** \brief Initialize color with rgb in values in [0,1] **/
  GlColor(float r, float g, float b, float a = 1.0f) : R(r), G(g), B(b), A(a) {}

  /** \brief convert to float*
   *  NOTE: array is valid as long as object is not destroyed.
   */
  operator float*() { return &R; }

  operator const float*() const { return &R; }

  /** \brief access to the RGB values as in an array **/
  inline float operator[](int i) { return (&R)[i]; }

  float toFloat() {
    int32_t rgb = int32_t(round(R * 255.0f));
    rgb = (rgb << 8) + int32_t(round(G * 255.0f));
    rgb = (rgb << 8) + int32_t(round(B * 255.0f));

    return float(rgb);
  }

  /** \brief generate Color from HSV values.
   *	The values must be provided as follows:
   *		- hue in [0,2pi],
   *		- saturation in [0.0, 1.0f]
   *		- value in [0.0, 1.0f]
   **/
  static GlColor FromHSV(float h, float s, float v, float a = 1.0f);
  /** \brief generate Color from RGB values in [0, 255] **/
  static GlColor FromRGB(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255);

  void lighter(float factor = 1.0);
  void darker(float factor = 1.0);

  float R, G, B, A;

  /** some common predefined colors. **/
  static const GlColor WHITE;
  static const GlColor GRAY;
  static const GlColor BLACK;
  static const GlColor RED;
  static const GlColor GREEN;
  static const GlColor BLUE;
  static const GlColor YELLOW;
  static const GlColor PINK;
  static const GlColor ORANGE;
  static const GlColor CYAN;
  static const GlColor GOLD;

 protected:
  void toHSV(float& h, float& s, float& v, float& a);
  void setHSV(float h, float s, float v, float a);
};
}
#endif /* GlColor_H */
