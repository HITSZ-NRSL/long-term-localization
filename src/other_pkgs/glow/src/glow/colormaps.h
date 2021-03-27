#ifndef SRC_GLOW_COLORMAPS_H_
#define SRC_GLOW_COLORMAPS_H_

#include "GlColorMap.h"

namespace glow {

/** \brief approximate Matplotlib's viridis colormap.
 *
 *  \author behley
 **/
class ViridisColorMap : public GlColorMap {
 public:
  GlColor operator()(float t) {
    vec4 tt(t * t * t, t * t, t, 1.0);

    GlColor result;
    result.R = tt.x * rviridis.x + tt.y * rviridis.y + tt.z * rviridis.z + tt.w * rviridis.w;
    result.G = tt.x * gviridis.x + tt.y * gviridis.y + tt.z * gviridis.z + tt.w * gviridis.w;
    result.B = tt.x * bviridis.x + tt.y * bviridis.y + tt.z * bviridis.z + tt.w * bviridis.w;

    return result;
  }

 protected:
  const vec4 rviridis{2.90912735, -2.14404531, 0.04439198, 0.29390206};
  const vec4 gviridis{-0.17293242, -0.16906214, 1.24131122, 0.01871256};
  const vec4 bviridis{0.17848859, -1.72405244, 1.23042564, 0.34479632};
};
}

#endif /* SRC_GLOW_COLORMAPS_H_ */
