// Copyright 2017 Emanuele Palazzolo (emanuele.palazzolo@uni-bonn.de)
#pragma once
#include "GlCamera.h"

#include <chrono>
#include <mutex>
#include <unordered_map>

namespace glow {

/** \brief FPS-Style OpenGL Camera
 *
 *  \author Emanuele Palazzolo (emanuele.palazzolo@uni-bonn.de)
 **/
class FpsCamera : public glow::GlCamera {
 public:
  FpsCamera();
  /** \brief update the view matrix and return it. **/
  const Eigen::Matrix4f& matrix() override;

  /** \brief set camera to specified transformation matrix. **/
  void setMatrix(const Eigen::Matrix4f& m) override;

  /** \brief set location of the camera. **/
  void setPosition(float x, float y, float z) override;

  /** \brief get current location of the camera. **/
  Eigen::Vector4f getPosition() const override;

  /** \brief set camera directions such that the reference point is centered **/
  void lookAt(float x_ref, float y_ref, float z_ref) override;
  /** \brief set camera to specified position looking at given reference point.
   * **/
  void lookAt(float x_cam, float y_cam, float z_cam, float x_ref, float y_ref, float z_ref) override;

  /** \brief set yaw angle.
   *
   *  yaw = rotation about y-axis.
   ***/
  void setYaw(float yaw);

  /** \brief set pitch angle.
   *
   *  pitch = rotation about x-axis.
   **/
  void setPitch(float pitch);

  /** \brief get camera parameters.
   *
   *  \see setPosition, setYaw, setPitch.
   **/
  void getCameraParameters(float& x, float& y, float& z, float& yaw, float& pitch);

  /** \brief process mouse pressed at position (x,y) with given KeyModifier
   *
   * \return true, if event was processed. false, otherwise.*
   **/
  bool mousePressed(float x, float y, MouseButton btn, KeyboardModifier modifier) override;

  /** \brief process mouse released at position (x,y) with given KeyModifier
   *
   *  \return true, if event was processed. false, otherwise.
   **/
  bool mouseReleased(float x, float y, MouseButton btn, KeyboardModifier modifier) override;

  /** \brief process mouse moved event at position (x,y) with given KeyModifier
   *
   *  \return true, if event was processed. false, otherwise.
   **/
  bool mouseMoved(float x, float y, MouseButton btn, KeyboardModifier modifier) override;

  /**  \brief process mouse wheel events by delta values, i.e., how much the
   * wheel position changed. **/
  bool wheelEvent(float delta, KeyboardModifier modifier) override;

  /** \brief process key pressed with given KeyboardModifier
   *
   *  \return true, if event was processed. false, otherwise.*
   **/
  bool keyPressed(KeyboardKey key, KeyboardModifier modifier) override;

  /** \brief process key released with given KeyboardModifier
   *
   *  \return true, if event was processed. false, otherwise.
   **/
  bool keyReleased(KeyboardKey key, KeyboardModifier modifier) override;

 protected:
  struct KeyHash {
    template <typename T>
    std::size_t operator()(T t) const {
      return static_cast<std::size_t>(t);
    }
  };

  void translate(float forward, float up, float sideways, float dt);
  void rotate(float yaw, float pitch);

  std::chrono::system_clock::time_point startTime_;  // of mouse pressed.
  float startx_{0.0f}, starty_{0.0f};
  float startyaw_{0.0f}, startpitch_{0.0f};

  float x_{0.0f}, y_{0.0f}, z_{0.0f};
  float yaw_{0.0f}, pitch_{0.0f};

  // velocity vector:
  float forwardVel_{0.0f}, upVel_{0.0f}, sideVel_{0.0f}, turnVel_{0.0f};

  // TODO: expose other parameters (sensitivity, etc.)

  std::mutex mutex_;
  std::unordered_map<KeyboardKey, bool, KeyHash> pressed_keys_;
  float movement_speed_{0.15f};
};

}  // namespace glow
