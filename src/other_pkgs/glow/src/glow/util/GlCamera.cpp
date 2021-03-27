#include "GlCamera.h"
#include "glow/glutil.h"

namespace glow {

GlCamera::GlCamera() {
}

GlCamera::~GlCamera() {
}

const Eigen::Matrix4f& GlCamera::matrix() {
  return view_;
}

void GlCamera::setMatrix(const Eigen::Matrix4f& m) {
  view_ = m;
}

void GlCamera::setPosition(float x, float y, float z) {
  view_(0, 3) = 0;
  view_(1, 3) = 0;
  view_(2, 3) = 0;
  view_ = view_ * glTranslate(-x, -y, -z);
}

Eigen::Vector4f GlCamera::getPosition() const {
  return Eigen::Vector4f(view_(0, 3), view_(1, 3), view_(2, 3), 1.0f);
}

void GlCamera::lookAt(float x_ref, float y_ref, float z_ref) {
  Eigen::Matrix4f m = view_.inverse();
  lookAt(m(0, 3), m(1, 3), m(2, 3), x_ref, y_ref, z_ref);
}

void GlCamera::lookAt(float x_cam, float y_cam, float z_cam, float x_ref, float y_ref, float z_ref) {
  // determine rotation from current location:
  Eigen::Vector3f pos_cam(x_cam, y_cam, z_cam);
  Eigen::Vector3f pos(x_ref, y_ref, z_ref);
  Eigen::Vector3f up(0.0f, 1.0f, 0.0f);
  Eigen::Vector3f f = (pos - pos_cam).normalized();
  Eigen::Vector3f x_axis = f.cross(up).normalized();
  Eigen::Vector3f y_axis = x_axis.cross(f).normalized();
  Eigen::Vector3f z_axis = -f;

  view_ = Eigen::Matrix4f::Zero();

  view_(0, 0) = x_axis[0];
  view_(0, 1) = x_axis[1];
  view_(0, 2) = x_axis[2];

  view_(1, 0) = y_axis[0];
  view_(1, 1) = y_axis[1];
  view_(1, 2) = y_axis[2];

  view_(2, 0) = z_axis[0];
  view_(2, 1) = z_axis[1];
  view_(2, 2) = z_axis[2];

  view_(3, 3) = 1.0f;

  // effectively => R * T
  view_(0, 3) = -pos_cam.dot(x_axis);
  view_(1, 3) = -pos_cam.dot(y_axis);
  view_(2, 3) = -pos_cam.dot(z_axis);
}

bool GlCamera::mousePressed(float x, float y, MouseButton btn, KeyboardModifier modifier) {
  return false;
}

bool GlCamera::mouseReleased(float x, float y, MouseButton btn, KeyboardModifier modifier) {
  return false;
}

bool GlCamera::mouseMoved(float x, float y, MouseButton btn, KeyboardModifier modifier) {
  return false;
}

bool GlCamera::wheelEvent(float delta, KeyboardModifier modifier) {
  return false;
}

bool GlCamera::keyPressed(KeyboardKey key, KeyboardModifier modifier) {
  return false;
}

bool GlCamera::keyReleased(KeyboardKey key, KeyboardModifier modifier) {
  return false;
}

} /* namespace glow */
