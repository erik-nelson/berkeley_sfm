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

#ifndef BSFM_CAMERA_CAMERA_EXTRINSICS_H
#define BSFM_CAMERA_CAMERA_EXTRINSICS_H

// #include <mathematics/square_matrix.h>

using mathematics::Mat34;
using mathematics::Mat44;
using mathematics::Mat41;
using mathematics::Matrix;

namespace bsfm {

class CameraExtrinsics {
public:
  CameraExtrinsics() {
    world_to_body_ = Mat44::IdentityMatrix();
    body_to_camera_ = Mat44::IdentityMatrix();
  }

  CameraExtrinsics(const Mat44 &world_to_camera)
    : world_to_body_(world_to_camera),
      body_to_camera_(Mat44::IdentityMatrix()) {}

  CameraExtrinsics(const Mat44 &world_to_body, const Mat44 &body_to_camera)
    : world_to_body_(world_to_body), body_to_camera_(body_to_camera) {}

  void SetWorldToCamera(const Mat44 &world_to_camera) {
    world_to_body_ = world_to_camera;
    body_to_camera_ = Mat44::IdentityMatrix();
  }

  void SetWorldToBody(const Mat44 &world_to_body) {
    world_to_body_ = world_to_body;
  }

  void SetBodyToCamera(const Mat44 &body_to_camera) {
    body_to_camera_ = body_to_camera;
  }

  Mat44 WorldToCamera() const { return body_to_camera_ * world_to_body_; }
  Mat44 WorldToBody() const { return world_to_body_; }
  Mat44 BodyToCamera() const { return body_to_camera_; }

  Mat44 CameraToWorld() const { return WorldToCamera().Inverse(); }
  Mat44 BodyToWorld() const { return WorldToBody().Inverse(); }
  Mat44 CameraToBody() const { return BodyToCamera().Inverse(); }

  // The extrinsics matrix is 3x4 matrix: [R | t].
  Mat34 ExtrinsicsMatrix() const {
    return WorldToCamera().Submatrix<3, 4>();
  }

  // Convert a world frame point into the camera frame.
  void WorldToCamera(double wx, double wy, double wz,
                     double* cx, double* cy, double* cz) const
  {
    if (cx == nullptr || cy == nullptr || cz == nullptr) return;

    Mat41 w_homogeneous;
    w_homogeneous(0) = wx;
    w_homogeneous(1) = wy;
    w_homogeneous(2) = wz;
    w_homogeneous(3) = 1.0;

    const Mat41 c_homogeneous = WorldToCamera() * w_homogeneous;

    *cx = c_homogeneous(0);
    *cy = c_homogeneous(1);
    *cz = c_homogeneous(2);
  }

  // Convert a camera frame point into the world frame.
  void CameraToWorld(double cx, double cy, double cz,
                     double* wx, double* wy, double* wz) const
  {
    if (wx == nullptr || wy == nullptr || wz == nullptr) return;

    Mat41 c_homogeneous;
    c_homogeneous(0) = cx;
    c_homogeneous(1) = cy;
    c_homogeneous(2) = cz;
    c_homogeneous(3) = 1.0;

    const Mat41 w_homogeneous = CameraToWorld() * c_homogeneous;

    *wx = w_homogeneous(0);
    *wy = w_homogeneous(1);
    *wz = w_homogeneous(2);
  }

private:
  Mat44 world_to_body_;
  Mat44 body_to_camera_;
};  //\class CameraExtrinsics

}  //\namespace bsfm

#endif
