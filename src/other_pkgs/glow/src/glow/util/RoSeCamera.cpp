#include "RoSeCamera.h"
#include "glow/glutil.h"

namespace glow {

const Eigen::Matrix4f& RoSeCamera::matrix() {
  mutex_.lock();

  auto end = std::chrono::system_clock::now();
  double dt = 0.0001 * std::chrono::duration_cast<std::chrono::milliseconds>(end - startTime_).count();

  if (dt > 0 && startdrag_) {
    // apply velocity & reset timer...
    rotate(turnVel_ * dt, 0.0f);
    translate(forwardVel_ * dt, upVel_ * dt, sideVel_ * dt);
    startTime_ = end;
  }

  // recompute the view matrix (Euler angles) Remember: Inv(AB) = Inv(B)*Inv(A)
  // Inv(translate*rotateYaw*rotatePitch) = Inv(rotatePitch)*Inv(rotateYaw)*Inv(translate)
  view_ = glRotateX(-pitch_);
  view_ = view_ * glRotateY(-yaw_);
  view_ = view_ * glTranslate(-x_, -y_, -z_);

  mutex_.unlock();

  return view_;
}

void RoSeCamera::setMatrix(const Eigen::Matrix4f& m) {
  mutex_.lock();

  Eigen::Vector4f view_dir = m.transpose() * Eigen::Vector4f(0, 0, -1, 0);
  Eigen::Vector4f new_cam = m.inverse() * Eigen::Vector4f(0, 0, 0, 1);
  Eigen::Vector4f new_ref = new_cam + view_dir;

  mutex_.unlock();

  lookAt(new_cam.x(), new_cam.y(), new_cam.z(), new_ref.x(), new_ref.y(), new_ref.z());
}

// since we are using here a different representation of the camera model we
// have to re-implement everything to allow an appropriate modification.
void RoSeCamera::setPosition(float x, float y, float z) {
  mutex_.lock();

  x_ = x;
  y_ = y;
  z_ = z;

  mutex_.unlock();
}

Eigen::Vector4f RoSeCamera::getPosition() const {
  return Eigen::Vector4f(x_, y_, z_, 1.0f);
}

void RoSeCamera::lookAt(float x_ref, float y_ref, float z_ref) {
  mutex_.lock();

  float x = x_ref - x_;
  float y = y_ref - y_;
  float z = z_ref - z_;

  Eigen::Vector3f dir(x, y, z);
  dir.normalize();

  // = std::acos(-dir.y()) - M_PI_2 = M_PI - std::acos(dir.y()) - M_PI_2; // in [-pi/2,pi/2]
  pitch_ = std::asin(dir.y());
  yaw_ = atan2(-x, -z);

  mutex_.unlock();
}

void RoSeCamera::lookAt(float x_cam, float y_cam, float z_cam, float x_ref, float y_ref, float z_ref) {
  mutex_.lock();

  x_ = x_cam;
  y_ = y_cam;
  z_ = z_cam;

  float x = x_ref - x_;
  float y = y_ref - y_;
  float z = z_ref - z_;

  Eigen::Vector3f dir(x, y, z);
  dir.normalize();

  pitch_ = std::asin(dir.y());  // = std::acos(-dir.y()) - M_PI_2 in [-pi/2,pi/2]
  yaw_ = atan2(-x, -z);

  mutex_.unlock();

  //  lookAt(x_ref, y_ref, z_ref);
}

void RoSeCamera::setYaw(float yaw) {
  mutex_.lock();
  yaw_ = yaw;
  mutex_.unlock();
}

void RoSeCamera::setPitch(float pitch) {
  mutex_.lock();
  pitch_ = pitch;
  mutex_.unlock();
}

void RoSeCamera::getCameraParameters(float& x, float& y, float& z, float& yaw, float& pitch) {
  mutex_.lock();

  x = x_;
  y = y_;
  z = z_;
  yaw = yaw_;
  pitch = pitch_;

  mutex_.unlock();
}

bool RoSeCamera::mousePressed(float x, float y, MouseButton btn, KeyboardModifier modifier) {
  startx_ = x;
  starty_ = y;
  startyaw_ = yaw_;
  startpitch_ = pitch_;
  startTime_ = std::chrono::system_clock::now();
  startdrag_ = true;

  return true;
}

bool RoSeCamera::mouseReleased(float x, float y, MouseButton btn, KeyboardModifier modifier) {
  forwardVel_ = 0.0;
  upVel_ = 0.0;
  sideVel_ = 0.0;
  turnVel_ = 0.0;
  startdrag_ = false;

  return true;
}

void RoSeCamera::translate(float forward, float up, float sideways) {
  // forward = -z, sideways = x , up = y. Remember: inverse of yaw is applied, i.e., we have to apply yaw (?)
  // Also keep in mind: sin(-alpha) = -sin(alpha) and cos(-alpha) = -cos(alpha)
  // We only apply the yaw to move along the yaw direction;
  //  x' = x*cos(yaw) - z*sin(yaw)
  //  z' = x*sin(yaw) + z*cos(yaw)
  float s = std::sin(yaw_);
  float c = std::cos(yaw_);

  x_ += sideways * c - forward * s;
  y_ += up;
  z_ -= sideways * s + forward * c;
}

/*****************************************************************************/

void RoSeCamera::rotate(float yaw, float pitch) {
  yaw_ += yaw;
  pitch_ += pitch;
  if (pitch_ < -M_PI_2) pitch_ = -M_PI_2;
  if (pitch_ > M_PI_2) pitch_ = M_PI_2;
}

bool RoSeCamera::mouseMoved(float x, float y, MouseButton btn, KeyboardModifier modifier) {
  mutex_.lock();

  // TODO: expose parameters:
  static const float MIN_MOVE = 0;
  static const float WALK_SENSITIVITY = 0.5f;
  static const float TURN_SENSITIVITY = 0.01f;
  static const float SLIDE_SENSITIVITY = 0.5f;
  static const float RAISE_SENSITIVITY = 0.5f;

  static const float LOOK_SENSITIVITY = 0.01f;
  static const float FREE_TURN_SENSITIVITY = 0.01f;

  float dx = x - startx_;
  float dy = y - starty_;

  if (dx > 0.0f) dx = std::max(0.0f, dx - MIN_MOVE);
  if (dx < 0.0f) dx = std::min(0.0f, dx + MIN_MOVE);
  if (dy > 0.0f) dy = std::max(0.0f, dy - MIN_MOVE);
  if (dy < 0.0f) dy = std::min(0.0f, dy + MIN_MOVE);

  // idea: if the velocity changes, we have to reset the start_time and update the camera parameters.

  if (btn == MouseButton::RightButton) {
    forwardVel_ = 0;
    upVel_ = 0;
    sideVel_ = 0;
    turnVel_ = 0;

    yaw_ = startyaw_ - FREE_TURN_SENSITIVITY * dx;
    pitch_ = startpitch_ - LOOK_SENSITIVITY * dy;

    // ensure valid values.
    if (pitch_ < -M_PI_2) pitch_ = -M_PI_2;
    if (pitch_ > M_PI_2) pitch_ = M_PI_2;
  } else if (btn == MouseButton::LeftButton) {
    // apply transformation:
    auto end = std::chrono::system_clock::now();
    double dt = std::chrono::duration_cast<std::chrono::seconds>(end - startTime_).count();

    if (dt > 0.0) {
      rotate(turnVel_ * dt, 0.0f);
      translate(forwardVel_ * dt, upVel_ * dt, sideVel_ * dt);

      startTime_ = end;  // reset timer.
    }

    forwardVel_ = -WALK_SENSITIVITY * dy;
    upVel_ = 0;
    sideVel_ = 0;
    turnVel_ = -(TURN_SENSITIVITY * dx);
  } else if (btn == MouseButton::MiddleButton) {
    // apply transformation:
    auto end = std::chrono::system_clock::now();
    double dt = std::chrono::duration_cast<std::chrono::seconds>(end - startTime_).count();

    if (dt > 0.0) {
      rotate(turnVel_ * dt, 0.0f);
      translate(forwardVel_ * dt, upVel_ * dt, sideVel_ * dt);

      startTime_ = end;  // reset timer.
    }

    forwardVel_ = 0;
    upVel_ = -RAISE_SENSITIVITY * dy;
    sideVel_ = SLIDE_SENSITIVITY * dx;
    turnVel_ = 0;
  }

  mutex_.unlock();

  return true;
}

bool RoSeCamera::wheelEvent(float delta, KeyboardModifier modifier) {
  mutex_.lock();
  static const float ZOOM_SENSITIVITY = 3.f;
  // move along the viewing direction specified by yaw and pitch.
  
    
    float forward = ZOOM_SENSITIVITY * delta * std::cos(pitch_);    
    float up = ZOOM_SENSITIVITY * delta * std::sin(pitch_) * (-1);
    float s = std::sin(yaw_);
    float c = std::cos(yaw_);



    x_ -= forward * s;
    y_ -= up;
    z_ += forward * c * (-1);
  // TODO: implement me!
   mutex_.unlock();

  return true;
}

bool RoSeCamera::keyPressed(KeyboardKey key, KeyboardModifier modifier){  
  float factor = (y_>50?50:(y_>1?y_:1));
  switch(key){
    case KeyboardKey::KeyA:
      startTime_ = std::chrono::system_clock::now();
      startdrag_ = true;
      sideVel_ = -10*factor;
      return true;
    case KeyboardKey::KeyD:
      startTime_ = std::chrono::system_clock::now();
      startdrag_ = true;
      sideVel_ = 10*factor;
      return true;
    case KeyboardKey::KeyW:
      startTime_ = std::chrono::system_clock::now();
      startdrag_ = true;
      forwardVel_ = 10*factor;
      return true;
    case KeyboardKey::KeyS:
      startTime_ = std::chrono::system_clock::now();
      startdrag_ = true;
      forwardVel_ = -10*factor;
      return true;
    default: 
      return false;
  }
}

   
bool RoSeCamera::keyReleased(KeyboardKey key, KeyboardModifier modifier){ 
  switch(key){
    case KeyboardKey::KeyA:
    case KeyboardKey::KeyD:
      startTime_ = std::chrono::system_clock::now();
        
      sideVel_ = 0;
      if (forwardVel_==0) startdrag_ = false;
      return true;
    case KeyboardKey::KeyW:
    case KeyboardKey::KeyS:
      startTime_ = std::chrono::system_clock::now();

      forwardVel_ = 0;
      if (sideVel_==0) startdrag_ = false;
      return true;
    default: 
      return false;
  }
}

} /* namespace rv */
