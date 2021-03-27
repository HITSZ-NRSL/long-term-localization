#ifndef INCLUDE_RV_GLUNIFORM_H_
#define INCLUDE_RV_GLUNIFORM_H_

#include "glbase.h"

namespace glow {

class GlProgram;

/**
 * \brief Interface of a GlUniform.
 *
 * Each uniform has to implement a bind method enabling programs to
 * bind the value to their shaders. With this indirection, we are able to store
 * uniform values in a single vector and still have the convenience of the `bind`
 * method.
 *
 * \see GlProgram::setUniform(GlAbstractUniform& uniform)
 *
 * \author behley
 */
class GlAbstractUniform {
 public:
  friend class GlProgram;

  GlAbstractUniform(const std::string& name) : name_(name) {}

  virtual ~GlAbstractUniform() {}

  /** \brief get the name of the uniform. **/
  const std::string& name() const { return name_; }

 protected:
  /** \brief use uniform for the specified active program.
   *
   *  We assume that the program is currently in use via the
   *  method glUseProgram() or using GlProgram's bind() method.
   *
   *  Therefore, we will not try to "activate" the program before
   *  setting the uniform.
   **/
  virtual void bind(GLuint program_id) const = 0;

  std::string name_;
};

/**
 * \brief Explicit representation of an uniform for OpenGL shader program.
 *
 * The idea is to enable an simple to use setting of uniforms including:
 *   1. Getting the location of the uniform via glUniformLocation
 *   2. Setting the variable with bind for the given program id with the correct/appropriate glUniform* method.
 *
 * A \a GlUniform holds therefore the name of the uniform and its value. A specific uniform implements the
 * `bind(GLuint id)` method, which enables a \a GlProgam to set the value with its id.
 *
 * A concrete uniform also has convenient getter, i.e., conversion functions, and setters, i.e., overloading of
 * the assignment operator, for access of the internal value.
 *
 * Example:
 *   // set uniform's value and get it again.
 *   GlUniform<int32_t> integer_uniform("my_uniform");
 *   integer_uniform = 1234;
 *   int32_t integer = integer_uniform;
 *   std::cout << "The uniform's value is " << integer << std::endl;
 *
 * \see GlProgram
 *
 * \author behley
 */
template <class T>
class GlUniform : public GlAbstractUniform {
 public:
  friend class GlProgram;

  /** \brief construct uniform with specific name and specific value. **/
  GlUniform(const std::string& name, const T& data);

  GlUniform<T>& operator=(const T& rhs);
  // FIXME: introduce rvalue assignment operator?

  /** \brief conversion functions returning the value of the uniform **/
  operator T();
  operator T() const;
  const T& value() const;

 protected:
  /** \brief use uniform for the specified active program.
   *
   *  We assume that the program is currently in use via the
   *  method glUseProgram() or using GlProgram's bind() method.
   *
   *  Therefore, we will not try to "activate" the program before
   *  setting the uniform.
   **/
  void bind(GLuint program_id) const override;

  T data_;
};

// generic definitions

template <class T>
GlUniform<T>::GlUniform(const std::string& name, const T& value)
    : GlAbstractUniform(name), data_(value) {
}

template <class T>
GlUniform<T>& GlUniform<T>::operator=(const T& rhs) {
  data_ = rhs;

  return *this;
}

template <class T>
GlUniform<T>::operator T() {
  return data_;
}

template <class T>
GlUniform<T>::operator T() const {
  return data_;
}

template <class T>
const T& GlUniform<T>::value() const {
  return data_;
}

} /* namespace rv */

#endif /* INCLUDE_RV_GLUNIFORM_H_ */
