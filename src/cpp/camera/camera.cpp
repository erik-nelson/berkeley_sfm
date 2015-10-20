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

#include "camera.h"

namespace bsfm {

// Constructor given extrinsics and intrinsics.
Camera::Camera(CameraExtrinsics extrinsics, CameraIntrinsics intrinsics)
    : extrinsics_(extrinsics), intrinsics_(intrinsics) {}

// Set extrinsics.
void Camera::SetExtrinsics(const CameraExtrinsics &extrinsics) {
  extrinsics_ = extrinsics;
}

// Set instrinsics.
void Camera::SetIntrinsics(const CameraIntrinsics &intrinsics) {
  intrinsics_ = intrinsics;
}

// Extract mutable/immutable extrinsics/intrinsics.
CameraExtrinsics &Camera::MutableExtrinsics() { return extrinsics_; }
CameraIntrinsics &Camera::MutableIntrinsics() { return intrinsics_; }
const CameraExtrinsics &Camera::Extrinsics() const { return extrinsics_; }
const CameraIntrinsics &Camera::Intrinsics() const { return intrinsics_; }

// Transform points from world to camera coordinates.
void Camera::WorldToCamera(double wx, double wy, double wz, double *cx,
                           double *cy, double *cz) const {
  extrinsics_.WorldToCamera(wx, wy, wz, cx, cy, cz);
}

// Transform points from camera to world coordinates.
void Camera::CameraToWorld(double cx, double cy, double cz, double *wx,
                           double *wy, double *wz) const {
  extrinsics_.CameraToWorld(cx, cy, cz, wx, wy, wz);
}

// Transform points from world to image coordinates. Return whether the point
// was visible to the camera.
bool Camera::CameraToImage(double cx, double cy, double cz, double *u_distorted,
                           double *v_distorted) const {
  return intrinsics_.CameraToImage(cx, cy, cz, u_distorted, v_distorted);
}

bool Camera::WorldToImage(double wx, double wy, double wz, double *u_distorted,
                          double *v_distorted) const {
  double cx = 0.0, cy = 0.0, cz = 0.0;
  WorldToCamera(wx, wy, wz, &cx, &cy, &cz);
  return CameraToImage(cx, cy, cz, u_distorted, v_distorted);
}

// Convert a normalized unit direction into the image by distorting it with the
// camera's radial distortion parameters.
bool Camera::DirectionToImage(double u_normalized, double v_normalized,
                              double *u_distorted, double *v_distorted) const {
  return intrinsics_.DirectionToImage(u_normalized, v_normalized, u_distorted,
                                      v_distorted);
}

// Convert a distorted image coordinate pair to a normalized direction vector
// using the camera's radial distortion parameters.
void Camera::ImageToDirection(double u_distorted, double v_distorted,
                              double *u_normalized,
                              double *v_normalized) const {
  intrinsics_.ImageToDirection(u_distorted, v_distorted, u_normalized,
                               v_normalized);
}

// Warp a point into the image.
void Camera::Distort(double u, double v, double *u_distorted,
                     double *v_distorted) const {
  intrinsics_.Distort(u, v, u_distorted, v_distorted);
}

// Rectilinearize point.
void Camera::Undistort(double u_distorted, double v_distorted, double *u,
                       double *v) const {
  intrinsics_.Undistort(u_distorted, v_distorted, u, v);
}

// Triangulate a feature match.
Eigen::Vector3d Camera::Triangulate(const FeatureMatch& match, const Camera& other) {

  // Unpack the FeatureMatch.
  double u1, v1, u2, v2;
  u1 = match.feature1_.u_;
  v1 = match.feature1_.v_;
  u2 = match.feature2_.u_;
  v2 = match.feature2_.v_;

  // Set up cross product matrices.
  Eigen::Matrix3d p1_cross, p2_cross;
  p1_cross <<
    0.0, -1.0, v1,
    1.0, 0.0, -u1,
    -v1, u1, 0.0;
  p2_cross <<
    0.0, -1.0, v2,
    1.0, 0.0, -u2,
    -v2, u2, 0.0;

  // Unpack extrinscs and intrinsics.
  CameraExtrinsics other_extrinsics = other.Extrinsics();
  CameraIntrinsics other_intrinsics = other.Intrinsics();

  typedef Eigen::Matrix<double, 3, 4> Matrix34d;
  Matrid34d other_extrinsics_matrix = other_extrinsics.ExtrinsicsMatrix();
  Matrix34d extrinsics_matrix = extrinsics_.ExtrinsicsMatrix();

  Eigen::Matrix3d other_intrinsics_matrix = other_intrinsics.IntrinsicsMatrix();
  Eigen::Matrix3d intrinsics_matrix = intrinsics_.IntrinsicsMatrix();

  // Set up linear least squares.
  Eigen::MatrixXd M(6, 4);
  M.topRows(3) = p1_cross * intrinsics_matrix * extrinsics_matrix;
  M.bottomRows(3) =
      p2_cross * other_intrinsics_matrix * other_extrinsics_matrix;

  Eigen::MatrixXd A = M.leftCols(7);
  Eigen::VectorXd b = -M.rightCols(1);

  // Solve using SVD.
  Eigen::Vector3d pt3d =
      A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
  return pt3d;
}

}  // namespace bsfm
