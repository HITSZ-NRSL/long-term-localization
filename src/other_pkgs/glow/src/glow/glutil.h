#ifndef INCLUDE_RV_GLUTIL_H_
#define INCLUDE_RV_GLUTIL_H_

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <ostream>

/** \brief some utility functions and classes. **/

namespace glow {

/** \brief two-dimensional vector **/
struct vec2 {
 public:
  vec2() : x(0.0f), y(0.0f) {}

  vec2(float xx, float yy) : x(xx), y(yy) {}

  float x, y;
};

/** \brief three-dimensional vector **/
struct vec3 {
 public:
  vec3() : x(0.0f), y(0.0f), z(0.0f) {}

  vec3(float xx, float yy, float zz) : x(xx), y(yy), z(zz) {}
  float x, y, z;
};

/** \brief four-dimensional vector **/
struct vec4 {
 public:
  vec4() : x(0.0f), y(0.0f), z(0.0f), w(0.0f) {}

  vec4(float xx, float yy, float zz, float ww) : x(xx), y(yy), z(zz), w(ww) {}
  float x, y, z, w;
};

/** \bried translate in x-,y-, and z-direction. **/
Eigen::Matrix4f glTranslate(float x, float y, float z);

Eigen::Matrix4f glScale(float x, float y, float z);

/** \brief rotate about x-axis by \a angle (radians). **/
Eigen::Matrix4f glRotateX(float angle);

/** \brief rotate about y-axis by \a angle (radians). **/
Eigen::Matrix4f glRotateY(float angle);

/** \brief rotate about z-axis by \a angle (radians). **/
Eigen::Matrix4f glRotateZ(float angle);

/** \brief perspective projection matrix. /gluPerspective...**/
Eigen::Matrix4f glPerspective(float fov, float aspect, float znear, float zfar);

/** \brief orthographic project matrix. /gluOrthographic.. **/
Eigen::Matrix4f glOrthographic(float left, float right, float bottom, float top, float znear, float zfar);

/** \brief rotate about axis (x,y,z) by \a angle (radians) **/
Eigen::Matrix4f glRotateAxis(float angle, float x, float y, float z);

/** \brief convert an angle given in degrees to radian **/
inline float radians(float deg) { return deg * M_PI / 180.0f; }

/** \brief convert an angle given in radian to degrees **/
inline float degrees(float rad) { return rad * 180.0f / M_PI; }

// coordinate transformations:

/** \brief coordinate transform from RoSe to OpenGL coordinates. **/
struct RoSe2GL {
 public:
  RoSe2GL() = delete;

  static Eigen::Matrix4f matrix;
};

/** \brief coordinate transform from RoSe to OpenGL coordinates. **/
struct GL2RoSe {
 public:
  GL2RoSe() = delete;

  static Eigen::Matrix4f matrix;
};

/** \brief encode (r,g,b) value in [0,1]^3 to a single float (24-bits). **/
float rgb2float(float r, float g, float b);

std::string extension(const std::string& path, int32_t level = 1);
}

std::ostream& operator<<(std::ostream& stream, glow::vec2& vec);
std::ostream& operator<<(std::ostream& stream, glow::vec3& vec);
std::ostream& operator<<(std::ostream& stream, glow::vec4& vec);

std::ostream& operator<<(std::ostream& stream, const glow::vec2& vec);
std::ostream& operator<<(std::ostream& stream, const glow::vec3& vec);
std::ostream& operator<<(std::ostream& stream, const glow::vec4& vec);

#endif /* INCLUDE_RV_GLUTIL_H_ */
