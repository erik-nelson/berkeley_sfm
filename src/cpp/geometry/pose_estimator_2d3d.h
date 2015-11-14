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
// This class implements the Gold Standard Algorithm for computing the pose of
// a camera, given 3D points in the world and 2D image-space features
// corresponding to those 3D points. The algorithm requires at least 6
// non-degenerate (e.g. not colinear) point matches to compute a solution.
//
// Inputs are:
// - A set of 2D image-space features.
// - A set of corresponding 3D world-space points (same number and order).
// - A set of camera intrinsic parameters.
//
// Outputs are:
// - A boolean for success or failure.
// - The pose of the camera.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_GEOMETRY_POSE_ESTIMATOR_2D_3D_H
#define BSFM_GEOMETRY_POSE_ESTIMATOR_2D_3D_H

#include <Eigen/Core>
#include <memory>

#include "../camera/camera_intrinsics.h"
#include "../geometry/point_3d.h"
#include "../matching/feature.h"
#include "../pose/pose.h"
#include "../util/disallow_copy_and_assign.h"
#include "../util/types.h"

namespace bsfm {

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

class PoseEstimator2D3D {
 public:
  PoseEstimator2D3D();
  ~PoseEstimator2D3D();

  // Initialize the solver with a list of 2D <--> 3D point correspondences and a
  // set of camera intrinsic parameters.
  bool Initialize(const FeatureList& points_2d, const Point3DList& points_3d,
                  const CameraIntrinsics& intrinsics);

  // Computes the camera pose, returning success or failure. If a solution was
  // not found, the input pose remains unchanged.
  bool Solve(Pose& camera_pose);

 private:
  DISALLOW_COPY_AND_ASSIGN(PoseEstimator2D3D)

  // Computes an initial valid solution for the projection matrix P = K[R|t]
  // using Eq. 7.2 from H&Z: Multiple-View Geometry.
  bool ComputeInitialSolution(Matrix34d& initial_solution) const;

  // Refines the solution using the Levenberg-Marquardt iterative algorithm for
  // non-linear least-squares.
  bool OptimizeSolution(Matrix34d& solution) const;

  // Uses the camera intrinsics (stored locally) to extract the camera pose from
  // the projection matrix P. This amounts to solving [R|t] = K^{-1} * P.
  // Returns false if for some reason the computed pose has a rotation matrix
  // with a determinant of 0, which is not a valid rotation matrix.
  bool ExtractPose(const Matrix34d& P, Pose& pose) const;

  // Check if a projection matrix is a good enough initialization to optimize.
  // This check is performed by evaluating the reprojection error (squared pixel
  // distance) between all 2D and 3D feature matches. The error threshold is
  // defined as a flag in the source file.
  bool TolerableReprojectionError(const Matrix34d& P) const;

  // (Un-)normalized versions of the inputs to the algorithm.
  FeatureList points_2d_, normalized_points_2d_;
  Point3DList points_3d_, normalized_points_3d_;

  // Input camera intrinsics matrix.
  CameraIntrinsics intrinsics_;

  // Normalization matrices for points_2d_ and points_3d_.
  Matrix3d T_;
  Matrix4d U_;

};  //\class PoseEstimator2D3D

}  //\namespace bsfm

#endif
