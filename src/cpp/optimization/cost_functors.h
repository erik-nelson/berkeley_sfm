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
// This file defines cost functors that can be used in conjunction with Google
// Ceres solver to solve non-linear least-squares problems. Each functor must
// define a public templated operator() method that takes in arguments const T*
// const INPUT_VARIABLE, and T* OUTPUT_RESIDUAL, and returns a bool.
// 'INPUT_VARIABLE' will be the optimization variable, and 'OUTPUT_RESIDUAL'
// will be the cost associated with the input.
//
// One can define more specific cost functions by adding other structure to the
// functor, e.g. by passing in other parameters of the cost function to the
// functor's constructor.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_OPTIMIZATION_COST_FUNCTORS_H
#define BSFM_OPTIMIZATION_COST_FUNCTORS_H

#include <Eigen/Core>
#include <glog/logging.h>

#include "../geometry/point_3d.h"
#include "../matching/feature.h"

namespace bsfm {

// Geometric error is the distance in image space between a point x, and a
// projected point PX, where X is a 3D homogeneous point (4x1), P is a camera
// projection matrix (3x4), and x is the corresponding image-space point
// expressed in homogeneous coordinates (3x1). The geometric error can be
// expressed as sum_i d(x_i, PX_i)^2, where d( , ) is the Euclidean distance
// metric.
struct GeometricError {
  // We are trying to adjust our camera projection matrix, P, to satisfy
  // x - PX = 0. Input is a 2D point in image space ('x'), and a 3D point in
  // world space ('X').
  Feature x_;
  Point3D X_;
  GeometricError(const Feature& x, const Point3D& X) : x_(x), X_(X) {}

  // Residual is 2-dimensional, P is 12-dimensional (number of elements in a
  // camera projection matrix).
  template <typename T>
  bool operator()(const T* const P, T* geometric_error) const {

    // Matrix multiplication: x - PX. Assume homogeneous coordinates are 1.0.
    // Matrix P is stored in column-major order.
    T scale = P[2] * X_.X() + P[5] * X_.Y() + P[8] * X_.Z() + P[11];
    geometric_error[0] =
       x_.u_ - (P[0] * X_.X() + P[3] * X_.Y() + P[6] * X_.Z() + P[9]) / scale;
    geometric_error[1] =
       x_.v_ - (P[1] * X_.X() + P[4] * X_.Y() + P[7] * X_.Z() + P[10]) / scale;

    return true;
  }

  // Factory method.
  static ceres::CostFunction* Create(const Feature& x, const Point3D& X) {
    static const int kNumResiduals = 2;
    static const int kNumCameraParameters = 12;
    return new ceres::AutoDiffCostFunction<GeometricError,
           kNumResiduals,
           kNumCameraParameters>(new GeometricError(x, X));
  }
};  //\struct GeometricError


// Bundle adjustment error is similar to geometric error, but the position of
// the 3D point is also optimized. The error is defined as the image-space
// geometric distance between an image point x, and a projected 3D landmark X
// projected according to x = K [R | t] X, where K and [R | t] are camera
// intrinsic and extrinsic parameter matrices, respectively. K is held constant
// during the optimization, and only the camera pose matrix [R | t] is optimized
// over.
struct BundleAdjustmentError {
  // Inputs are the image space point x and the intrinsics matrix K.
  // Optimization variables are the 3D landmark position X, and the camera
  // extrinsics matrix [R | t].
  Feature x_;
  Matrix3d K_;
  BundleAdjustmentError(const Feature& x, const Matrix3d& K) : x_(x), K_(K) {}

  template <typename T>
  bool operator()(const T* const Rt, const T* const X,
                  T* bundle_adjustment_error) const {

    // Matrix multiplication: P = K * [R | t]. Note that [R | t] is provided in
    // column-major order.
    T P11 = K_(0, 0)*Rt[0] + K_(0, 1)*Rt[1] + K_(0, 2)*Rt[2];
    T P12 = K_(0, 0)*Rt[3] + K_(0, 1)*Rt[4] + K_(0, 2)*Rt[5];
    T P13 = K_(0, 0)*Rt[6] + K_(0, 1)*Rt[7] + K_(0, 2)*Rt[8];
    T P14 = K_(0, 0)*Rt[9] + K_(0, 1)*Rt[10] + K_(0, 2)*Rt[11];

    T P21 = K_(1, 0)*Rt[0] + K_(1, 1)*Rt[1] + K_(1, 2)*Rt[2];
    T P22 = K_(1, 0)*Rt[3] + K_(1, 1)*Rt[4] + K_(1, 2)*Rt[5];
    T P23 = K_(1, 0)*Rt[6] + K_(1, 1)*Rt[7] + K_(1, 2)*Rt[8];
    T P24 = K_(1, 0)*Rt[9] + K_(1, 1)*Rt[10] + K_(1, 2)*Rt[11];

    T P31 = K_(2, 0)*Rt[0] + K_(2, 1)*Rt[1] + K_(2, 2)*Rt[2];
    T P32 = K_(2, 0)*Rt[3] + K_(2, 1)*Rt[4] + K_(2, 2)*Rt[5];
    T P33 = K_(2, 0)*Rt[6] + K_(2, 1)*Rt[7] + K_(2, 2)*Rt[8];
    T P34 = K_(2, 0)*Rt[9] + K_(2, 1)*Rt[10] + K_(2, 2)*Rt[11];

    // Compute image space scale, assuming homogeneous coordinate is 1.0.
    T scale = P31 * X[0] + P32 * X[1] + P33 * X[2] + P34;

    // Compute geometric error in image-space.
    bundle_adjustment_error[0] =
        x_.u_ - (P11 * X[0] + P12 * X[1] + P13 * X[2] + P14) / scale;
    bundle_adjustment_error[1] =
        x_.v_ - (P21 * X[0] + P22 * X[1] + P23 * X[2] + P24) / scale;

    return true;
  }

  // Factory method.
  static ceres::CostFunction* Create(const Feature& x, const Matrix3d& K) {
    static const int kNumResiduals = 2;
    static const int kNumExtrinsicParameters = 12;
    static const int kNumLandmarkParameters = 3;
    return new ceres::AutoDiffCostFunction<BundleAdjustmentError,
           kNumResiduals,
           kNumExtrinsicParameters,
           kNumLandmarkParameters>(new BundleAdjustmentError(x, K));
  }
};  //\BundleAdjustmentError

}  //\namespace bsfm

#endif
