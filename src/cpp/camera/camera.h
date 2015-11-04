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
// This class defines a camera model implemented using computations and
// parameters from the OpenCV camera model:
// http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
//
// By default, the camera is staring down its +Z axis. +X and +Y are the
// camera's right-facing and upward-facing vectors in this coordinate frame.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_CAMERA_CAMERA_H
#define BSFM_CAMERA_CAMERA_H

#include <matching/feature_match.h>

#include "camera_extrinsics.h"
#include "camera_intrinsics.h"
#include "../util/types.h"

namespace bsfm {

using Eigen::Matrix3d;

class Camera {
 public:

  // Constructor and destructor.
  Camera() { };
  ~Camera() { };

  // Constructor given extrinsics and intrinsics.
  Camera(CameraExtrinsics, CameraIntrinsics);

  // Set extrinsics.
  void SetExtrinsics(const CameraExtrinsics&);

  // Set instrinsics.
  void SetIntrinsics(const CameraIntrinsics&);

  // Extract mutable/immutable extrinsics/intrinsics.
  CameraExtrinsics &MutableExtrinsics();
  CameraIntrinsics &MutableIntrinsics();
  const CameraExtrinsics& Extrinsics() const;
  const CameraIntrinsics& Intrinsics() const;

  // Get the projection matrix by multiplying intrinsics and extrinsics.
  Matrix34d P() const;

  // Get the camera intrinsics matrix, K.
  Matrix3d K() const;

  // Get the camera extrinsics matrix, [R | t].
  Matrix34d Rt() const;

  // Get the camera's world frame translation from extrinsics.
  Vector3d Translation() const;

  // Get the camera's world frame rotation from extrinsics.
  Matrix3d Rotation() const;

  // Get the camera's world frame rotation in axis angle parameterization.
  Vector3d AxisAngleRotation() const;

  // Transform points from world to camera coordinates.
  void WorldToCamera(double wx, double wy, double wz, double* cx, double* cy,
                     double* cz) const;

  // Transform points from camera to world coordinates.
  void CameraToWorld(double cx, double cy, double cz,
                     double* wx, double* wy, double* wz) const;

  // Transform points from camera to image coordinates. Return whether the input
  // point is in front of the camera.
  bool CameraToImage(double cx, double cy, double cz,
                     double* u_distorted, double* v_distorted) const;

  // Transform points from world to image coordinates. Return whether the input
  // point is in front of the camera.
  bool WorldToImage(double wx, double wy, double wz,
                    double* u_distorted, double* v_distorted) const;

  // Convert a normalized unit direction into the image by distorting it with
  // the camera's radial distortion parameters.
  bool DirectionToImage(double u_normalized, double v_normalized,
                        double* u_distorted, double* v_distorted) const;

  // Convert a distorted image coordinate pair to a normalized direction
  // vector using the camera's radial distortion parameters.
  void ImageToDirection(double u_distorted, double v_distorted,
                        double* u_normalized, double* v_normalized) const;

  // Warp a point into the image.
  void Distort(double u, double v,
               double* u_distorted, double* v_distorted) const;

  // Rectilinearize point.
  void Undistort(double u_distorted, double v_distorted,
                 double* u, double* v) const;

private:
  CameraExtrinsics extrinsics_;
  CameraIntrinsics intrinsics_;
};  //\class Camera

}  //\bsfm
#endif
