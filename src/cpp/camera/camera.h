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
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_CAMERA_CAMERA_H
#define BSFM_CAMERA_CAMERA_H

#include "camera_extrinsics.h"
#include "camera_intrinsics.h"

namespace bsfm {

class Camera {
 public:
  Camera() { }
  ~Camera() { }

  Camera(CameraExtrinsics extrinsics,
         CameraIntrinsics intrinsics)
    : extrinsics_(extrinsics),
      intrinsics_(intrinsics) {}

  void SetExtrinsics(const CameraExtrinsics &extrinsics) {
    extrinsics_ = extrinsics;
  }

  void SetIntrinsics(const CameraIntrinsics &intrinsics) {
    intrinsics_ = intrinsics;
  }

  CameraExtrinsics &MutableExtrinsics() { return extrinsics_; }
  CameraIntrinsics &MutableIntrinsics() { return intrinsics_; }
  const CameraExtrinsics& Extrinsics() const { return extrinsics_; }
  const CameraIntrinsics& Intrinsics() const { return intrinsics_; }

  void WorldToCamera(double wx, double wy, double wz,
                     double *cx, double *cy, double *cz) const {
    extrinsics_.WorldToCamera(wx, wy, wz, cx, cy, cz);
  }

  void CameraToWorld(double cx, double cy, double cz,
                     double *wx, double *wy, double *wz) const {
    extrinsics_.CameraToWorld(cx, cy, cz, wx, wy, wz);
  }

  bool CameraToImage(double cx, double cy, double cz,
                     double *u_distorted, double *v_distorted) const {
    intrinsics_.CameraToImage(cx, cy, cz, u_distorted, v_distorted);
  }

  bool DirectionToImage(double u_normalized, double v_normalized,
                        double *u_distorted, double *v_distorted) const {
    intrinsics_.DirectionToImage(u_normalized, v_normalized,
                                 u_distorted, v_distorted);
  }

  void ImageToDirection(double u_distorted, double v_distorted,
                        double *u_normalized, double *v_normalized) const {
    intrinsics_.ImageToDirection(u_distorted, v_distorted,
                                 u_normalized, v_normalized);
  }

  void Distort(double u, double v,
               double *u_distorted, double *v_distorted) const {
    intrinsics_.Distort(u, v, u_distorted, v_distorted);
  }

  void Undistort(double u_distorted, double v_distorted,
                 double *u, double *v) const {
    intrinsics_.Undistort(u_distorted, v_distorted, u, v);
  }

 private:
  CameraExtrinsics extrinsics_;
  CameraIntrinsics intrinsics_;
};  //\class Camera

}  //\bsfm
#endif
