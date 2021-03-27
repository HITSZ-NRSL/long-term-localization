#ifndef INCLUDE_RV_GLSTATE_H_
#define INCLUDE_RV_GLSTATE_H_

#include "glbase.h"
#include <cstdarg>
#include <ostream>
#include <map>

namespace glow {

/** \brief Representation of current OpenGL state that can be queried by glGet.
 *
 *  In OpenGL, we sometimes screw up the state of OpenGL, but we don't know exactly where.
 *  Here, the GlState class comes into play to get a snapshot of the current state that is
 *  accessible via glGet*. On initialization all available information is gathered, therefore
 *  it is not a good idea to call this frequently in production and time-critical code.
 *
 *  TODO: implement querying of single values and specific set of values.
 *
 *  \author behley
 **/
class GlState {
 public:
  class GlStateVariable {
   public:
    GlStateVariable(bool value);
    GlStateVariable(int32_t i1);
    GlStateVariable(int32_t i1, int32_t i2);
    GlStateVariable(int32_t i1, int32_t i2, int32_t i3);
    GlStateVariable(int32_t i1, int32_t i2, int32_t i3, int32_t i4);
    GlStateVariable(float f1);
    GlStateVariable(float f1, float f2);
    GlStateVariable(float f1, float f2, float f3);
    GlStateVariable(float f1, float f2, float f3, float f4);

    bool operator==(const GlStateVariable& other) const;
    bool operator!=(const GlStateVariable& other) const;

    enum DataType { BOOL, INT, FLOAT };

    DataType type;

    union {
      bool valb;
      int32_t vali[4];
      float valf[4];
    };

    uint32_t size;
  };

  /** \brief get value of given state variable. */
  template <typename T>
  T get(GLenum variable) const;

  /** \brief resore the values to the values inside the state. **/
  void restore();

  /** \brief get all state variables of current OpenGL state. **/
  static GlState queryAll();

  /** \brief compare state and print variables that differ on std::cout. **/
  void difference(const GlState& other) const;

  /** \brief determine if states are the same for all variables.
   *
   * \return true, if states match; false otherwise.
   **/
  bool operator==(const GlState& other) const;

  /** \brief determine if states are not the same, at least for one variable.
   *
   * \return true, if states match; false otherwise.
   **/
  bool operator!=(const GlState& other) const;

  /** \brief get some state variables. **/
  //   TODO: static GlState query(GLenum vars...);
  friend std::ostream& operator<<(std::ostream& stream, const GlState& state);

 protected:
  GlState();

  std::string stringify_name(GLenum value) const;
  std::string stringify_value(const std::pair<GLenum, GlState::GlStateVariable>& entry) const;

  template <typename T>
  void initGlParameter(GLenum name, uint32_t num_values);

  std::map<GLenum, GlStateVariable> state_;
};

template <typename T>
T GlState::get(GLenum variable) const {
  if (state_.find(variable) == state_.end()) throw std::runtime_error("No such variable found in GlState.");
  auto it = state_.find(variable);

  if (it->second.type == GlStateVariable::INT) {
    return T(it->second.vali[0]);
  } else if (it->second.type == GlStateVariable::FLOAT) {
    return T(it->second.valf[0]);
  } else if (it->second.type == GlStateVariable::BOOL) {
    return T(it->second.valb);
  }

  // should never reach this...
  throw std::runtime_error("Unkown state variable type.");
}
}
/* namespace rv */

#endif /* INCLUDE_RV_GLSTATE_H_ */
