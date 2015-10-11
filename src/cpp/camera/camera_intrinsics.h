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
// This class defines a camera's intrinsic parameters, calibration, and
// computations using the OpenCV camera model, with the minor modification that
// we will use a 5th-order distortion model rather than a 6th-order model. Math
// for the 5th-order distortion model can be found at the caltech vision page:
//
// http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
// http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_CAMERA_CAMERA_INTRINSICS_H
#define BSFM_CAMERA_CAMERA_INTRINSICS_H

#include <Eigen/Dense>

namespace bsfm {

class CameraIntrinsics {
 public:
  // Initialize to zero.
  CameraIntrinsics();

  // Assume no radial distortion, and image left and top are both zero.
  CameraIntrinsics(const Eigen::Matrix3d& K, int image_width, int image_height);

  // Full initialization.
  CameraIntrinsics(int image_left, int image_top, int image_width,
                   int image_height, double f_u, double f_v, double c_u,
                   double c_v, double k1, double k2, double k3, double k4,
                   double k5);

  // Set individual parameters.
  void SetImageLeft(int image_left);
  void SetImageTop(int image_top);
  void SetImageWidth(int image_width);
  void SetImageHeight(int image_height);

  void SetFU(double f_u);
  void SetFV(double f_v);
  void SetCU(double c_u);
  void SetCV(double c_v);

  void SetK(double k1, double k2, double k3, double k4, double k5);
  void SetK1(double k1);
  void SetK2(double k2);
  void SetK3(double k3);
  void SetK4(double k4);
  void SetK5(double k5);

  void SetHorizontalFOV(double horizontal_fov);
  void SetVerticalFOV(double vertical_fov);

  // Extract parameters.
  int ImageLeft() const;
  int ImageTop() const;
  int ImageWidth() const;
  int ImageHeight() const;
  double f_u() const;
  double f_v() const;
  double c_u() const;
  double c_v() const;
  double k1() const;
  double k2() const;
  double k3() const;
  double k4() const;
  double k5() const;
  double HorizontalFOV() const;
  double VerticalFOV() const;

  // Get intrinsics matrix.
  Eigen::Matrix3d IntrinsicsMatrix() const;

  // Get inverse of intrinsics matrix.
  Eigen::Matrix3d InverseIntrinsicsMatrix() const;

  // Test if a point is in the image.
  bool PointInImage(double u, double v) const;

  // Check if a point is in front of the camera.
  bool CameraToImage(double cx, double cy, double cz, double *u_distorted,
                     double *v_distorted) const;

  bool DirectionToImage(double u_normalized, double v_normalized,
                        double *u_distorted, double *v_distorted) const;

  void ImageToDirection(double u_distorted, double v_distorted,
                        double *u_normalized, double *v_normalized) const;

  // Warp a point into the image.
  void Distort(double u, double v, double *u_distorted,
               double *v_distorted) const;

  // Rectilinearize point.
  void Undistort(double u_distorted, double v_distorted, double *u, double *v,
                 int iterations = 10) const;

 private:
  int image_left_;
  int image_top_;
  int image_width_;
  int image_height_;
  double f_u_;
  double f_v_;
  double c_u_;
  double c_v_;
  double k1_;
  double k2_;
  double k3_;
  double k4_;
  double k5_;
  double horizontal_fov_;
  double vertical_fov_;
};  //\class CameraIntrinsics

}  //\namespace bsfm

#endif
