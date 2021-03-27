#ifndef SRC_GLOW_GLCOLORMAP_H_
#define SRC_GLOW_GLCOLORMAP_H_

namespace glow {

/** \brief colormap producing GlColors for values in [0.0,1.0]
 *
 *  \author behley
 */
class GlColorMap {
 public:
  virtual ~GlColorMap() {}
  virtual GlColor operator()(float value) = 0;
};
}
#endif /* SRC_GLOW_GLCOLORMAP_H_ */
