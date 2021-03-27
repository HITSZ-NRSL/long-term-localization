#ifndef SRC_OPENGL_GLCAMERA_H_
#define SRC_OPENGL_GLCAMERA_H_

#include "enum_utils.h"

#include <eigen3/Eigen/Dense>

namespace glow {

/** \brief Camera model for specifying a view matrix using mouse events.
 *
 *  A general camera which can be modified by the provided methods. We want to keep the interface very, very general
 *  allowing the implementation of a preferred camera model.
 *
 *  The general idea is that the camera gets screen coordinates and modifiers, like left mouse button,
 *  right mouse button, etc. With these inputs one can now call the methods mousePressed, mouseReleased and mouseMoved
 *  to update the current mouse position. Depending on the underlying implementation, this triggers some update of
 *  the view matrix. (velocity-based, i.e., regularly updated view positions, or simple position based
 *  updates, where a change in mouse position directly corresponds to a change in camera location, are possible.)
 *
 *  Each event handler returns true, if the corresponding event has been consumed and therefore might be now irrelevant
 *  for further event processing.
 *
 *  A call to matrix() returns the current view matrix, which might be updated since the last call to matrix().
 *
 *
 *  \see RoSeCamera for a velocity-based navigation.
 *  \see FpsCamera for a camera with additional key bindings.
 *  TODO: \see IsometricCamera for a direct modification of the camera, similar to Blender.
 *
 *  \author behley
 */
class GlCamera {
 public:
  enum class MouseButton { NoButton = 0, LeftButton = 1, MiddleButton = 2, RightButton = 4 };

  enum class KeyboardModifier { None = 0, CtrlDown = 1, AltDown = 2, ShiftDown = 4 };

  enum class KeyboardKey { KeyA = 0, KeyB = 1, KeyC = 2, KeyD = 3, KeyE = 4, KeyF = 5, KeyG = 6, KeyH = 7, KeyI = 8,
                           KeyJ = 9, KeyK = 10, KeyL = 11, KeyM = 12, KeyN = 13, KeyO = 14, KeyP = 15, KeyQ = 16,
                           KeyR = 17, KeyS = 18, KeyT = 19, KeyU = 20, KeyV = 21, KeyW = 22, KeyX = 23, KeyY = 24,
                           KeyZ = 25, Key1 = 26, Key2 = 27, Key3 = 28, Key4 = 29, Key5 = 30, Key6 = 31, Key7 = 32,
                           Key8 = 33, Key9 = 34, Key0 = 35, KeyF1 = 36, KeyF2 = 37, KeyF3 = 38, KeyF4 = 39, KeyF5 = 40,
                           KeyF6 = 41, KeyF7 = 42, KeyF8 = 43, KeyF9 = 44, KeyF10 = 45, KeyF11 = 46, KeyF12 = 47,
                           KeyEsc = 48, KeyEnter = 49, KeyUpArrow = 50, KeyDownArrow = 51, KeyLeftArrow = 52,
                           KeyRightArrow = 53, KeySpace = 54, KeyNotSupported = -1 };
  virtual ~GlCamera();

  /** \brief return the current view matrix. **/
  virtual const Eigen::Matrix4f& matrix();

  /** \brief update position of camera such that camera's origin is at (x,y,z) **/
  virtual void setPosition(float x, float y, float z);

  virtual Eigen::Vector4f getPosition() const;

  /** \brief set camera such that it looks at specific location. **/
  virtual void lookAt(float x_ref, float y_ref, float z_ref);
  virtual void lookAt(float x_cam, float y_cam, float z_cam, float x_ref, float y_ref, float z_ref);

  /** \brief process mouse pressed at position (x,y) with given KeyboardModifier
   *
   *  \return true, if event was processed. false, otherwise.*
   **/
  virtual bool mousePressed(float x, float y, MouseButton btn, KeyboardModifier modifier);

  /** \brief process mouse released at position (x,y) with given KeyboardModifier
   *
   *  \return true, if event was processed. false, otherwise.
   **/
  virtual bool mouseReleased(float x, float y, MouseButton btn, KeyboardModifier modifier);

  /** \brief process mouse moved event at position (x,y) with given KeyboardModifier
   *
   *  \return true, if event was processed. false, otherwise.
   **/
  virtual bool mouseMoved(float x, float y, MouseButton btn, KeyboardModifier modifier);

  /**  \brief process mouse wheel events by delta values, i.e., how much the wheel position changed. **/
  virtual bool wheelEvent(float delta, KeyboardModifier modifier);

  /** \brief process key pressed with given KeyboardModifier
   *
   *  \return true, if event was processed. false, otherwise.*
   **/
  virtual bool keyPressed(KeyboardKey key, KeyboardModifier modifier);

  /** \brief process key released with given KeyboardModifier
   *
   *  \return true, if event was processed. false, otherwise.
   **/
  virtual bool keyReleased(KeyboardKey key, KeyboardModifier modifier);

  virtual void setMatrix(const Eigen::Matrix4f& m);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  GlCamera();  // cannot be instanced, only inherited from.

  Eigen::Matrix4f view_{Eigen::Matrix4f::Identity()};
};

// TODO: more elegant way than this? Allowing essentially something like if(v & ev) instead of if((v & ev) = ev)?
// ENUM_BIT_OPERATIONS(GlCamera::MouseButton)
// ENUM_BIT_OPERATIONS(GlCamera::KeyModifier)

} /* namespace glow */

#endif /* SRC_OPENGL_GLCAMERA_H_ */
