#ifndef INCLUDE_RV_GLCAPABILITIES_H_
#define INCLUDE_RV_GLCAPABILITIES_H_

#include <memory>

#include "GlState.h"

namespace glow {

/** \brief Get the limits and capabilities of the associated OpenGL context.
 *
 *  Queries all the fixed capabilities of the currently associated OpenGL context,
 *  such as maximal number of available texture units, maximum number of buffer bindings, etc.
 *
 *  This query is quite expensive, therefore only once really performed when generating the
 *  singleton. Thus, if you need to query the capabilities of the underlying GL context, do
 *  it once in not time critical.
 *
 *  In contrast to the state of the OpenGL context, these values can not be changed. Thus,
 *  GlState captures all values that can be set by the driver and GlCapabilities contains all
 *  constants that cannot be adjusted.
 *
 *  \author behley
 */
class GlCapabilities {
 public:
  static GlCapabilities& getInstance();

  /** \brief get value for given parameter name. **/
  template <typename T>
  T get(GLenum variable) const;

  friend std::ostream& operator<<(std::ostream& out, const GlCapabilities& cap);

 protected:
  GlCapabilities();

  std::string stringify_name(GLenum name) const;
  std::string stringify_value(const std::pair<GLenum, GlState::GlStateVariable>& entry) const;

  template <typename T>
  void initGlParameter(GLenum name, uint32_t num_values);

  static std::unique_ptr<GlCapabilities> instance_;
  std::map<GLenum, GlState::GlStateVariable> state_;
};

template <typename T>
T GlCapabilities::get(GLenum variable) const {
  if (state_.find(variable) == state_.end()) throw std::runtime_error("No such variable found in GlState.");
  auto it = state_.find(variable);

  if (it->second.type == GlState::GlStateVariable::INT) {
    return T(it->second.vali[0]);
  } else if (it->second.type == GlState::GlStateVariable::FLOAT) {
    return T(it->second.valf[0]);
  } else if (it->second.type == GlState::GlStateVariable::BOOL) {
    return T(it->second.valb);
  }

  // should never reach this...
  throw std::runtime_error("Unkown state variable type.");
}

} /* namespace rv */

#endif /* INCLUDE_RV_GLCAPABILITIES_H_ */
