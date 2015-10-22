/*
 * Copyright (c) 2015, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: Erik Nelson            ( eanelson@eecs.berkeley.edu )
 *          David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// This class defines a camera's extrinsic parameters according to the OpenCV
// camera model:
// http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
//
///////////////////////////////////////////////////////////////////////////////

#include "camera/camera_extrinsics.h"

#include "../geometry/rotation.h"

namespace bsfm {

// Constructor. Initialize to identity.
CameraExtrinsics::CameraExtrinsics() {
  world_to_camera_ = CameraExtrinsics::DefaultWorldToCamera();
}

// Constructor. Initialize world_to_camera_ .
CameraExtrinsics::CameraExtrinsics(const Pose& world_to_camera)
    : world_to_camera_(world_to_camera) {}

// Set world_to_camera_.
void CameraExtrinsics::SetWorldToCamera(const Pose& world_to_camera) {
  world_to_camera_ = world_to_camera;
}

// Extract poses.
Pose CameraExtrinsics::WorldToCamera() const {
  return world_to_camera_;
}

Pose CameraExtrinsics::CameraToWorld() const {
  return CameraExtrinsics::WorldToCamera().Inverse();
}

// For use in the following methods: From H&Z page 156, the extrinsics matrix
// can be represented as
//   [R -Rc]
//   [0  1 ]
// where c is the camera centroid. From this we get t = -Rc and c = -R't
void CameraExtrinsics::SetRotation(const Eigen::Matrix3d& rotation) {
  const Eigen::Vector3d t = world_to_camera_.Translation();
  const Eigen::Matrix3d R = world_to_camera_.Rotation();
  const Eigen::Vector3d c = -R.transpose() * t;

  world_to_camera_.SetRotation(rotation);
  world_to_camera_.SetTranslation(-rotation * c);
}

void CameraExtrinsics::SetRotation(double phi, double theta, double psi) {
  SetRotation(EulerAnglesToMatrix(phi, theta, psi));
}

void CameraExtrinsics::Rotate(const Eigen::Matrix3d& delta) {
  const Eigen::Matrix3d R = world_to_camera_.Rotation();
  SetRotation(delta * R);
}

void CameraExtrinsics::Rotate(double dphi, double dtheta, double dpsi) {
  Rotate(EulerAnglesToMatrix(dphi, dtheta, dpsi));
}

void CameraExtrinsics::SetTranslation(const Eigen::Vector3d& translation) {
  const Eigen::Matrix3d R = world_to_camera_.Rotation();
  world_to_camera_.SetTranslation(-R * translation);
}

void CameraExtrinsics::SetTranslation(double x, double y, double z) {
  SetTranslation(Eigen::Vector3d(x, y, z));
}

void CameraExtrinsics::Translate(const Eigen::Vector3d& delta) {
  const Eigen::Vector3d t = world_to_camera_.Translation();
  const Eigen::Matrix3d R = world_to_camera_.Rotation();
  Eigen::Vector3d c = -R.transpose() * t;

  c += delta;
  world_to_camera_.SetTranslation(-R*c);
}

void CameraExtrinsics::Translate(double dx, double dy, double dz) {
  Translate(Eigen::Vector3d(dx, dy, dz));
}

void CameraExtrinsics::TranslateX(double dx) {
  Translate(Eigen::Vector3d(dx, 0, 0));
}

void CameraExtrinsics::TranslateY(double dy) {
  Translate(Eigen::Vector3d(0, dy, 0));
}

void CameraExtrinsics::TranslateZ(double dz) {
  Translate(Eigen::Vector3d(0, 0, dz));
}

// The extrinsics matrix is 3x4 matrix: [R | t].
Eigen::Matrix<double, 3, 4> CameraExtrinsics::Rt() const {
  return WorldToCamera().Dehomogenize();
}

// Convert a world frame point into the camera frame.
void CameraExtrinsics::WorldToCamera(double wx, double wy, double wz,
                                     double* cx, double* cy, double* cz) const {
  if (cx == nullptr || cy == nullptr || cz == nullptr) return;

  const Eigen::Vector4d w_h(wx, wy, wz, 1.0);
  const Eigen::Vector4d c_h = CameraExtrinsics::WorldToCamera().Project(w_h);

  *cx = c_h(0);
  *cy = c_h(1);
  *cz = c_h(2);
}

// Convert a camera frame point into the world frame.
void CameraExtrinsics::CameraToWorld(double cx, double cy, double cz,
                                     double* wx, double* wy, double* wz) const {
  if (wx == nullptr || wy == nullptr || wz == nullptr) return;

  const Eigen::Vector4d c_h(cx, cy, cz, 1.0);
  const Eigen::Vector4d w_h = CameraExtrinsics::CameraToWorld().Project(c_h);

  *wx = w_h(0);
  *wy = w_h(1);
  *wz = w_h(2);
}

} // namespace bsfm
