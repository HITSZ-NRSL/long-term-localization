// Copyright 2017 Emanuele Palazzolo (emanuele.palazzolo@uni-bonn.de)
#include "FpsCamera.h"
#include <glow/glutil.h>
#include <iostream>

namespace glow {

FpsCamera::FpsCamera() {
  pressed_keys_.insert(std::make_pair(KeyboardKey::KeyW, false));
  pressed_keys_.insert(std::make_pair(KeyboardKey::KeyA, false));
  pressed_keys_.insert(std::make_pair(KeyboardKey::KeyS, false));
  pressed_keys_.insert(std::make_pair(KeyboardKey::KeyD, false));
  pressed_keys_.insert(std::make_pair(KeyboardKey::KeyC, false));
  pressed_keys_.insert(std::make_pair(KeyboardKey::KeySpace, false));
}

const Eigen::Matrix4f& FpsCamera::matrix() {
  mutex_.lock();

  auto end = std::chrono::system_clock::now();
  double dt = 0.0001 * std::chrono::duration_cast<std::chrono::milliseconds>(end - startTime_).count();

  if (dt > 0) {
    // apply velocity & reset timer...
    rotate(turnVel_ * dt, 0.0f);
    translate(forwardVel_, upVel_, sideVel_, dt);
    startTime_ = end;
  }

  forwardVel_ = 0.0;
  upVel_ = 0.0;
  sideVel_ = 0.0;
  turnVel_ = 0.0;

  if (pressed_keys_[KeyboardKey::KeyW]) {
    forwardVel_ = 1;
  }
  if (pressed_keys_[KeyboardKey::KeyS]) {
    forwardVel_ = -1;
  }
  if (pressed_keys_[KeyboardKey::KeyC]) {
    upVel_ = -1;
  }
  if (pressed_keys_[KeyboardKey::KeySpace]) {
    upVel_ = 1;
  }
  if (pressed_keys_[KeyboardKey::KeyA]) {
    sideVel_ = -1;
  }
  if (pressed_keys_[KeyboardKey::KeyD]) {
    sideVel_ = 1;
  }

  // recompute the view matrix (Euler angles) Remember: Inv(AB) = Inv(B)*Inv(A)
  // Inv(translate*rotateYaw*rotatePitch) = Inv(rotatePitch)*Inv(rotateYaw)*Inv(translate)
  
  view_ = glRotateX(-pitch_);
  view_ = view_ * glRotateY(-yaw_);
  view_ = view_ * glTranslate(-x_, -y_, -z_);

  mutex_.unlock();

  return view_;
}

void FpsCamera::setMatrix(const Eigen::Matrix4f& m) {
  mutex_.lock();

  Eigen::Vector4f view_dir = m.transpose() * Eigen::Vector4f(0, 0, -1, 0);
  Eigen::Vector4f new_cam = m.inverse() * Eigen::Vector4f(0, 0, 0, 1);
  Eigen::Vector4f new_ref = new_cam + view_dir;

  mutex_.unlock();

  lookAt(new_cam.x(), new_cam.y(), new_cam.z(), new_ref.x(), new_ref.y(), new_ref.z());
}

// since we are using here a different representation of the camera model we
// have to re-implement everything to allow an appropriate modification.
void FpsCamera::setPosition(float x, float y, float z) {
  mutex_.lock();

  x_ = x;
  y_ = y;
  z_ = z;

  mutex_.unlock();
}

Eigen::Vector4f FpsCamera::getPosition() const {
  return Eigen::Vector4f(x_, y_, z_, 1.0f);
}

void FpsCamera::lookAt(float x_ref, float y_ref, float z_ref) {
  mutex_.lock();

  float x = x_ref - x_;
  float y = y_ref - y_;
  float z = z_ref - z_;

  Eigen::Vector3f dir(x, y, z);
  dir.normalize();

  pitch_ = std::asin(dir.y());
  yaw_ = atan2(-x, -z);

  mutex_.unlock();
}

void FpsCamera::lookAt(float x_cam, float y_cam, float z_cam, float x_ref, float y_ref, float z_ref) {
  mutex_.lock();

  x_ = x_cam;
  y_ = y_cam;
  z_ = z_cam;

  float x = x_ref - x_;
  float y = y_ref - y_;
  float z = z_ref - z_;

  Eigen::Vector3f dir(x, y, z);
  dir.normalize();

  pitch_ = std::asin(dir.y());
  yaw_ = atan2(-x, -z);

  mutex_.unlock();
}

void FpsCamera::setYaw(float yaw) {
  mutex_.lock();
  yaw_ = yaw;
  mutex_.unlock();
}

void FpsCamera::setPitch(float pitch) {
  mutex_.lock();
  pitch_ = pitch;
  mutex_.unlock();
}

void FpsCamera::getCameraParameters(float& x, float& y, float& z, float& yaw, float& pitch) {
  mutex_.lock();

  x = x_;
  y = y_;
  z = z_;
  yaw = yaw_;
  pitch = pitch_;

  mutex_.unlock();
}

bool FpsCamera::mousePressed(float x, float y, MouseButton btn, KeyboardModifier modifier) {
  if (btn == MouseButton::RightButton) {
    startx_ = x;
    starty_ = y;
    startyaw_ = yaw_;
    startpitch_ = pitch_;
    // startTime_ = std::chrono::system_clock::now();
  }
  return true;
}

bool FpsCamera::mouseReleased(float x, float y, MouseButton btn, KeyboardModifier modifier) {
  return true;
}

bool FpsCamera::keyPressed(KeyboardKey key, KeyboardModifier modifier) {
  if (pressed_keys_.count(key) > 0) {
    pressed_keys_[key] = true;
  }
  // TODO movement speed parametric
  if (modifier == KeyboardModifier::ShiftDown) {
    movement_speed_ = 200.0f;
  } else {
    movement_speed_ = 100.0f;
  }
  return true;
}

bool FpsCamera::keyReleased(KeyboardKey key, KeyboardModifier modifier) {
  if (pressed_keys_.count(key) > 0) {
    pressed_keys_[key] = false;
  }
  return true;
}

void FpsCamera::translate(float forward, float up, float sideways, float dt) {
  Eigen::Vector4f hom_translation(sideways, 0, -forward, 1);
  Eigen::Matrix4f R = glow::glRotateY(yaw_) * glow::glRotateX(pitch_);
  hom_translation = R * hom_translation;
  Eigen::Vector3f translation = hom_translation.block<3, 1>(0, 0);
  // translation /= hom_translation(3);
  translation(1) += up;
  if (translation.norm() != 0) {
    translation.normalize();
  }

  translation.block<3, 1>(0, 0) *= movement_speed_ * dt;
  x_ += translation(0);
  y_ += translation(1);
  z_ += translation(2);
}

/*****************************************************************************/

void FpsCamera::rotate(float yaw, float pitch) {
  yaw_ += yaw;
  pitch_ += pitch;
  if (pitch_ < -M_PI_2) pitch_ = -M_PI_2;
  if (pitch_ > M_PI_2) pitch_ = M_PI_2;
}

bool FpsCamera::mouseMoved(float x, float y, MouseButton btn, KeyboardModifier modifier) {
  mutex_.lock();

  // TODO: expose parameters:
  static const float LOOK_SENSITIVITY = 0.01f;
  static const float FREE_TURN_SENSITIVITY = 0.01f;

  float dx = x - startx_;
  float dy = y - starty_;

  if (btn == MouseButton::RightButton) {
    yaw_ = startyaw_ - FREE_TURN_SENSITIVITY * dx;
    pitch_ = startpitch_ - LOOK_SENSITIVITY * dy;

    // ensure valid values.
    if (pitch_ <= -M_PI_2) pitch_ = -M_PI_2;
    if (pitch_ >= M_PI_2) pitch_ = M_PI_2;
  }

  mutex_.unlock();

  return true;
}

bool FpsCamera::wheelEvent(float delta, KeyboardModifier modifier) {
  return true;
}

}  // namespace glow
