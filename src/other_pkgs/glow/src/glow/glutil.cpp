#include "glutil.h"
#include <boost/filesystem.hpp>
#include <cmath>

namespace glow {

Eigen::Matrix4f glTranslate(float x, float y, float z) {
  Eigen::Matrix4f m;
  m << 1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1;
  return m;
}

Eigen::Matrix4f glScale(float x, float y, float z) {
  Eigen::Matrix4f m;
  m << x, 0, 0, 0, 0, y, 0, 0, 0, 0, z, 0, 0, 0, 0, 1;
  return m;
}

Eigen::Matrix4f glRotateX(float angle) {
  float sin_t = std::sin(angle);
  float cos_t = std::cos(angle);

  Eigen::Matrix4f m;
  m << 1, 0, 0, 0, 0, cos_t, -sin_t, 0, 0, sin_t, cos_t, 0, 0, 0, 0, 1;

  return m;
}

Eigen::Matrix4f glRotateY(float angle) {
  float sin_t = std::sin(angle);
  float cos_t = std::cos(angle);
  Eigen::Matrix4f m;
  m << cos_t, 0, sin_t, 0, 0, 1, 0, 0, -sin_t, 0, cos_t, 0, 0, 0, 0, 1;

  return m;
}

Eigen::Matrix4f glRotateZ(float angle) {
  float sin_t = std::sin(angle);
  float cos_t = std::cos(angle);
  Eigen::Matrix4f m;
  m << cos_t, -sin_t, 0, 0, sin_t, cos_t, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  return m;
}

Eigen::Matrix4f glRotateAxis(float angle, float x, float y, float z) {
  Eigen::Matrix4f m;

  float s = std::sin(angle);
  float c = std::cos(angle);

  m(0, 0) = x * x * (1 - c) + c;
  m(0, 1) = x * y * (1.0f - c) - z * s;
  m(0, 2) = x * z * (1.0f - c) + y * s;
  m(0, 3) = 0.0f;

  m(1, 0) = x * y * (1.0f - c) + z * s;
  m(1, 1) = c + (1.0f - c) * y * y;
  m(1, 2) = y * z * (1.0f - c) - x * s;
  m(1, 3) = 0.0f;

  m(2, 0) = x * z * (1.0f - c) - y * s;
  m(2, 1) = y * z * (1.0f - c) + x * s;
  m(2, 2) = z * z + (1.0f - z * z) * c;
  m(2, 3) = 0.0f;

  m(3, 0) = 0.0f;
  m(3, 1) = 0.0f;
  m(3, 2) = 0.0f;
  m(3, 3) = 1.0f;

  return m;
}

Eigen::Matrix4f glPerspective(float fov, float aspect, float znear, float zfar) {
  // https://www.opengl.org/sdk/docs/man2/xhtml/gluPerspective.xml
  assert(znear > 0.0f);
  Eigen::Matrix4f M = Eigen::Matrix4f::Zero();

  // Copied from gluPerspective
  float f = 1.0f / std::tan(0.5f * fov);

  M(0, 0) = f / aspect;
  M(1, 1) = f;
  M(2, 2) = (znear + zfar) / (znear - zfar);
  M(2, 3) = (2.0f * zfar * znear) / (znear - zfar);
  M(3, 2) = -1.0f;

  return M;
}

Eigen::Matrix4f glOrthographic(float left, float right, float bottom, float top, float znear, float zfar) {

  // copied from https://www.khronos.org/registry/OpenGL-Refpages/gl2.1/xhtml/glOrtho.xml

  Eigen::Matrix4f M = Eigen::Matrix4f::Zero();

  M(0, 0) = 2.0f / (right - left);
  M(1, 1) = 2.0f / (top - bottom);
  M(2, 2) = -2.0f / (zfar - znear);

  M(0, 3) = -(right + left) / (right - left);
  M(1, 3) = -(top + bottom) / (top - bottom);
  M(2, 3) = -(zfar + znear) / (zfar - znear);
  M(3, 3) = 1.0f;

  return M;
}

// clang-format off
Eigen::Matrix4f initializeMatrix(float t00, float t01, float t02, float t03,
                                 float t10, float t11, float t12, float t13,
                                 float t20, float t21, float t22, float t23,
                                 float t30, float t31, float t32, float t33) {
  Eigen::Matrix4f m;
  m << t00, t01, t02, t03, t10, t11, t12, t13, t20, t21, t22, t23, t30, t31, t32, t33;
  return m;
}

Eigen::Matrix4f RoSe2GL::matrix = initializeMatrix(0, -1, 0, 0,
                                                   0,  0, 1, 0,
                                                  -1,  0, 0, 0,
                                                   0,  0, 0, 1);


Eigen::Matrix4f GL2RoSe::matrix = initializeMatrix(0, 0, -1, 0,
                                                  -1, 0,  0, 0,
                                                   0, 1,  0, 0,
                                                   0, 0,  0, 1);
// clang-format on

float rgb2float(float r, float g, float b) {
  int32_t rgb = int32_t(round(r * 255.0f));
  rgb = (rgb << 8) + int32_t(round(g * 255.0f));
  rgb = (rgb << 8) + int32_t(round(b * 255.0f));

  return float(rgb);
}

std::string extension(const std::string& path, int32_t level) {
  std::string filename = boost::filesystem::path(path).filename().string();
  if (filename == "" || filename == "." || filename == "..") return "";

  std::string ext;
  while (level-- > 0) {
    std::string::size_type idx = filename.rfind(".");
    if (idx == std::string::npos) break;
    ext.insert(0, filename.substr(idx));
    filename.resize(idx);
  }

  return ext;
}
}

std::ostream& operator<<(std::ostream& stream, glow::vec2& vec) {
  stream << "(" << vec.x << ", " << vec.y << ")";

  return stream;
}

std::ostream& operator<<(std::ostream& stream, glow::vec3& vec) {
  stream << "(" << vec.x << ", " << vec.y << ", " << vec.z << ")";

  return stream;
}

std::ostream& operator<<(std::ostream& stream, glow::vec4& vec) {
  stream << "(" << vec.x << ", " << vec.y << ", " << vec.z << ", " << vec.w << ")";

  return stream;
}

std::ostream& operator<<(std::ostream& stream, const glow::vec2& vec) {
  stream << "(" << vec.x << ", " << vec.y << ")";

  return stream;
}

std::ostream& operator<<(std::ostream& stream, const glow::vec3& vec) {
  stream << "(" << vec.x << ", " << vec.y << ", " << vec.z << ")";

  return stream;
}

std::ostream& operator<<(std::ostream& stream, const glow::vec4& vec) {
  stream << "(" << vec.x << ", " << vec.y << ", " << vec.z << ", " << vec.w << ")";

  return stream;
}
