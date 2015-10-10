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

// #include <math.h>

// #include <mathematics/square_matrix.h>

using mathematics::Mat33;
using mathematics::Mat31;

namespace vision {

class CameraIntrinsics {
 public:
  CameraIntrinsics()
      : image_left_(0),
        image_top_(0),
        image_width_(0),
        image_height_(0),
        f_u_(0.0),
        f_v_(0.0),
        c_u_(0.0),
        c_v_(0.0),
        k1_(0.0),
        k2_(0.0),
        k3_(0.0),
        k4_(0.0),
        k5_(0.0),
        horizontal_fov_(0.0),
        vertical_fov_(0.0) {}

  // Assume no radial distortion, and image left and top are both zero.
  CameraIntrinsics(const Mat33 &K, int image_width, int image_height)
      : image_left_(0),
        image_top_(0),
        image_width_(image_width),
        image_height_(image_height),
        f_u_(K(0, 0)),
        f_v_(K(1, 1)),
        c_u_(K(0, 2)),
        c_v_(K(1, 2)),
        k1_(0.0),
        k2_(0.0),
        k3_(0.0),
        k4_(0.0),
        k5_(0.0) {
    horizontal_fov_ = 2.0 * atan2(0.5 * image_width_, f_u_);
    vertical_fov_ = 2.0 * atan2(0.5 * image_height_, f_v_);
  }

  CameraIntrinsics(int image_left, int image_top, int image_width,
                   int image_height, double f_u, double f_v, double c_u,
                   double c_v, double k1, double k2, double k3, double k4,
                   double k5)
      : image_left_(image_left),
        image_top_(image_top),
        image_width_(image_width),
        image_height_(image_height),
        f_u_(f_u),
        f_v_(f_v),
        c_u_(c_u),
        c_v_(c_v),
        k1_(k1),
        k2_(k2),
        k3_(k3),
        k4_(k4),
        k5_(k5) {
    horizontal_fov_ = 2.0 * atan2(0.5 * image_width_, f_u_);
    vertical_fov_ = 2.0 * atan2(0.5 * image_height_, f_v_);
  }

  void SetImageLeft(int image_left) { image_left_ = image_left; }
  void SetImageTop(int image_top) { image_top_ = image_top; }
  void SetImageWidth(int image_width) {
    image_width_ = image_width;
    horizontal_fov_ = 2.0 * atan2(0.5 * image_width_, f_u_);
  }
  void SetImageHeight(int image_height) {
    image_height_ = image_height;
    vertical_fov_ = 2.0 * atan2(0.5 * image_height_, f_v_);
  }
  void SetFU(double f_u) {
    f_u_ = f_u;
    horizontal_fov_ = 2.0 * atan2(0.5 * image_width_, f_u_);
  }
  void SetFV(double f_v) {
    f_v_ = f_v;
    vertical_fov_ = 2.0 * atan2(0.5 * image_height_, f_v_);
  }
  void SetCU(double c_u) { c_u_ = c_u; }
  void SetCV(double c_v) { c_v_ = c_v; }
  void SetK(double k1, double k2, double k3, double k4, double k5) {
    k1_ = k1;
    k2_ = k2;
    k3_ = k3;
    k4_ = k4;
    k5_ = k5;
  }
  void SetK1(double k1) { k1_ = k1; }
  void SetK2(double k2) { k2_ = k2; }
  void SetK3(double k3) { k3_ = k3; }
  void SetK4(double k4) { k4_ = k4; }
  void SetK5(double k5) { k5_ = k5; }
  void SetHorizontalFOV(double horizontal_fov) {
    horizontal_fov_ = horizontal_fov;
    f_u_ = 0.5 * image_width_ / tan(0.5 * horizontal_fov_);
  }
  void SetVerticalFOV(double vertical_fov) {
    vertical_fov_ = vertical_fov;
    f_v_ = 0.5 * image_height_ / tan(0.5 * vertical_fov_);
  }

  int ImageLeft() const { return image_left_; }
  int ImageTop() const { return image_top_; }
  int ImageWidth() const { return image_width_; }
  int ImageHeight() const { return image_height_; }
  double f_u() const { return f_u_; }
  double f_v() const { return f_v_; }
  double c_u() const { return c_u_; }
  double c_v() const { return c_v_; }
  double k1() const { return k1_; }
  double k2() const { return k2_; }
  double k3() const { return k3_; }
  double k4() const { return k4_; }
  double k5() const { return k5_; }
  double HorizontalFOV() const { return horizontal_fov_; }
  double VerticalFOV() const { return vertical_fov_; }

  Mat33 IntrinsicsMatrix() const {
    return Mat33(f_u_, 0.0, c_u_, 0.0, f_v_, c_v_, 0.0, 0.0, 1.0);
  }

  Mat33 InverseIntrinsicsMatrix() const { return IntrinsicsMatrix().Inverse(); }

  bool PointInImage(double u, double v) const {
    const bool in_cols = u >= image_left_ && u < image_left_ + image_width_;
    const bool in_rows = v >= image_top_ && v < image_top_ + image_height_;
    return in_cols && in_rows;
  }

  bool CameraToImage(double cx, double cy, double cz, double *u_distorted,
                     double *v_distorted) const {
    if (u_distorted == nullptr || v_distorted == nullptr) return false;

    // We can't project points that lie behind the camera.
    if (cz < 0.0) return false;

    // Convert the camera frame point into a normalized direction vector (see
    // OpenCV help page at the top of this file).
    const double u_normalized = cx / cz;
    const double v_normalized = cy / cz;

    return DirectionToImage(u_normalized, v_normalized, u_distorted,
                            v_distorted);
  }

  bool DirectionToImage(double u_normalized, double v_normalized,
                        double *u_distorted, double *v_distorted) const {
    if (u_distorted == nullptr || v_distorted == nullptr) return false;

    // Distort the normalized direction vector;
    double u = 0.0, v = 0.0;
    Distort(u_normalized, v_normalized, &u, &v);

    // Flip v - image space coordinates begin at the top.
    v = -v;

    // Make a homogeneous vector from the output.
    Mat31 p;
    p(0) = u;
    p(1) = v;
    p(2) = 1.0;

    // Multiply the distorted direction vector by camera intrinsic matrix to get
    // the image space point.
    const Mat31 p_out = IntrinsicsMatrix() * p;
    *u_distorted = p_out(0);
    *v_distorted = p_out(1);

    // Make sure that the resulting point is in the image.
    return PointInImage(*u_distorted, *v_distorted);
  }

  void ImageToDirection(double u_distorted, double v_distorted,
                        double *u_normalized, double *v_normalized) const {
    if (u_normalized == nullptr | v_normalized == nullptr) return;

    // Make a homogeneous image space point.
    Mat31 p_distorted;
    p_distorted(0) = u_distorted;
    p_distorted(1) = v_distorted;
    p_distorted(2) = 1.0;

    // Multiply the distorted homogeneous image space point by the inverse
    // of the camera intrinsic matrix to get a distorted ray.
    const Mat31 p = InverseIntrinsicsMatrix() * p_distorted;

    // Undistort the ray to get the normalized direction vector.
    Undistort(p(0), p(1), u_normalized, v_normalized);
  }

  void Distort(double u, double v, double *u_distorted,
               double *v_distorted) const {
    if (u_distorted == nullptr | v_distorted == nullptr) return;

    // Get the camera's radial distortion (see OpenCV help page at the top of
    // this file).
    const double r_sq = u * u + v * v;
    const double radial_dist =
        1.0 + k1_ * r_sq + k2_ * r_sq * r_sq + k5_ * r_sq * r_sq * r_sq;

    // If radial distortion is too extreme, our 5th-order model might not be a
    // good fit. Instead, estimate the distortion with a simpler model (linear),
    // and apply radial corrections.
    if (radial_dist < 0.85 || radial_dist > 1.15) {
      // Distortion too extreme. Warp the point with estimated radial
      // distortion.
      const double radius = hypot(image_width_, image_height_);
      *u_distorted = u / sqrt(r_sq) * radius;
      *v_distorted = v / sqrt(r_sq) * radius;
    } else {
      // Compute radial distortion with 5th order model.
      const double dx0 = 2.0 * k3_ * u * v + k4_ * (r_sq + 2.0 * u * u);
      const double dx1 = k3_ * (r_sq + 2.0 * v * v) + 2.0 * k4_ * u * v;

      // Homogeneous distorted direction vector.
      *u_distorted = radial_dist * u + dx0;
      *v_distorted = radial_dist * v + dx1;
    }
  }

  void Undistort(double u_distorted, double v_distorted, double *u, double *v,
                 int iterations = 10) const {
    if (u == nullptr || v == nullptr) return;

    // Iteratively attempt to undo the radial distortion (see OpenCV help page
    // at the top of this file).
    double u_refine = u_distorted;
    double v_refine = v_distorted;
    for (int i = 0; i < iterations; ++i) {
      const double r_sq = u_refine * u_refine + v_refine * v_refine;
      const double dx0 = 2.0 * k3_ * u_refine * v_refine +
                         k4_ * (r_sq + 2.0 * u_refine * u_refine);
      const double dx1 = k3_ * (r_sq + 2.0 * v_refine * v_refine) +
                         2.0 * k4_ * u_refine * v_refine;
      u_refine = u_distorted - dx0;
      v_refine = v_distorted - dx1;
    }

    // Return the undistorted direction vector.
    *u = u_refine;
    *v = v_refine;
  }

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
