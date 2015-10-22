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
// The default camera frame is such that the camera stares down its own +Z axis.
// +X and +Y are the camera's right-facing and downward-facing vectors in this
// coordinate frame. The camera's +Z axis is the world +Y axis (and therefore
// world +X = camera +X). This convention can be seen in the static
// DefaultWorldToCamera() member function.
//
//         Camera ---------> +Z  (camera facing this direction)
//        /  |
//       /   |
//      /    |
//     /     |
//    v      v
//   +X      +Y
//
//        +Z
//         ^
//         |
//         |
//         |
//         |
//       World -------->+Y
//       /
//      /
//     /
//    /
//   v
//   +X
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_CAMERA_CAMERA_EXTRINSICS_H
#define BSFM_CAMERA_CAMERA_EXTRINSICS_H

#include <pose/pose.h>

namespace bsfm {

class CameraExtrinsics {

public:
  // Constructor. Initialize to identity.
  CameraExtrinsics();

  // Constructor. Initialize world_to_camera_.
  CameraExtrinsics(const Pose& world_to_camera);

  static Pose DefaultWorldToCamera() {
    // Conversion does the following:
    //   world +X --> camera +X
    //   world +Y --> camera +Z
    //   world +Z --> camera -Y
    Eigen::Matrix4d w2c;
    w2c << 1, 0,  0, 0,
           0, 0, -1, 0,
           0, 1,  0, 0,
           0, 0,  0, 1;
    return Pose(w2c);
  }

  // Set world_to_camera_.
  void SetWorldToCamera(const Pose& world_to_camera);

  // Extract poses.
  Pose WorldToCamera() const;
  Pose CameraToWorld() const;

  // Rotate the world-to-camera frame.
  void SetRotation(const Eigen::Matrix3d& rotation);
  void SetRotation(double phi, double theta, double psi);
  void Rotate(const Eigen::Matrix3d& delta);
  void Rotate(double dphi, double dtheta, double dpsi);

  // Translate the world-to-camera frame. All inputs correspond to the
  // coordinates of the camera in world-frame.
  void SetTranslation(const Eigen::Vector3d& translation);
  void SetTranslation(double wx, double wy, double wz);
  void Translate(const Eigen::Vector3d& delta);
  void Translate(double dx, double dy, double dz);
  void TranslateX(double dx);
  void TranslateY(double dy);
  void TranslateZ(double dz);

  // The extrinsics matrix is 3x4 matrix: [R | t].
  Eigen::Matrix<double, 3, 4> Rt() const;

  // Convert a world frame point into the camera frame.
  void WorldToCamera(double wx, double wy, double wz,
                     double* cx, double* cy, double* cz) const;

  // Convert a camera frame point into the world frame.
  void CameraToWorld(double cx, double cy, double cz,
                     double* wx, double* wy, double* wz) const;

private:
  Pose world_to_camera_;

};  //\class CameraExtrinsics

}  //\namespace bsfm

#endif
