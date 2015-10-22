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

#include "pose_estimator_2d_3d.h"

#include <ceres/ceres.h>
#include <Eigen/SVD>
#include <glog/logging.h>

#include "normalization.h"
#include "../optimization/cost_functors.h"

namespace bsfm {

PoseEstimator2D3D::PoseEstimator2D3D()
    : T_(Matrix3d::Identity()), U_(Matrix4d::Identity()) {}

PoseEstimator2D3D::~PoseEstimator2D3D() {}

bool PoseEstimator2D3D::Initialize(const FeatureList& points_2d,
                                   const Point3DList& points_3d,
                                   const CameraIntrinsics& intrinsics) {
  if (points_2d.size() != points_3d.size()) {
    VLOG(1) << "Inputs 'points_2d' and 'points_3d' do not contain the "
               "same number of elements.";
    return false;
  }

  // Copy camera intrinsics.
  intrinsics_ = intrinsics;

  // Normalize the 3D points.
  T_ = ComputeNormalization(points_2d);
  U_ = ComputeNormalization(points_3d);

  points_2d_.clear();
  for (size_t ii = 0; ii < points_2d.size(); ++ii) {
    // Get (u, v) image-space point and normalize.
    double u = points_2d[ii].u_;
    double v = points_2d[ii].v_;
    u = T_(0, 0) * u + T_(0, 2);
    v = T_(1, 1) * v + T_(1, 2);

    points_2d_.push_back(Feature(u, v));
  }

  points_3d_.clear();
  for (size_t ii = 0; ii < points_3d.size(); ++ii) {
    // Get (x, y, z) world-space point and normalize.
    double x = points_3d[ii].X();
    double y = points_3d[ii].Y();
    double z = points_3d[ii].Z();
    x = U_(0, 0) * x + U_(0, 3);
    y = U_(1, 1) * y + U_(1, 3);
    z = U_(2, 2) * z + U_(2, 3);

    points_3d_.push_back(Point3D(x, y, z));
  }

  return true;
}

bool PoseEstimator2D3D::Solve(Pose& camera_pose) {
  // Get an initial P.
  Matrix34d P;
  if (!ComputeInitialSolution(P)) {
    VLOG(1) << "Failed to compute an initial solution for P.";
    return false;
  }

  // Refine P with non-linear optimization.
  Matrix34d P_opt = P;
  if (!OptimizeSolution(P_opt)) {
    VLOG(1) << "Failed to optimize P. Continuing using the initial solution.";
    P_opt = P;
  }

  // Get the camera pose from the computed projection matrix.
  if (!ExtractPose(P_opt, camera_pose)) {
    VLOG(1) << "Computed rotation is non-invertible.";
    return false;
  }

  return true;
}

bool PoseEstimator2D3D::ComputeInitialSolution(Matrix34d& initial_solution) {
  // Use Eq. 7.2 from H&Z: Multiple-View Geometry to determine an
  // over-determined solution for the camera projection matrix P.

  // First build the A matrix, which is 2n x 12.
  MatrixXd A;
  A.resize(points_2d_.size() * 2, 12);
  for (size_t ii = 0; ii < points_2d_.size(); ii++) {
    // Get (u, v) image-space point.
    const double u = points_2d_[ii].u_;
    const double v = points_2d_[ii].v_;

    // Get (x, y, z) world-space point.
    const double x = points_3d_[ii].X();
    const double y = points_3d_[ii].Y();
    const double z = points_3d_[ii].Z();

    // First 4 columns, top row of block.
    A(2*ii+0, 0) = 0;
    A(2*ii+0, 1) = 0;
    A(2*ii+0, 2) = 0;
    A(2*ii+0, 3) = 0;

    // Second 4 columns, top row of block.
    A(2*ii+0, 4) = -x;
    A(2*ii+0, 5) = -y;
    A(2*ii+0, 6) = -z;
    A(2*ii+0, 7) = -1;

    // Third 4 columns, top row of block.
    A(2*ii+0, 8) = v*x;
    A(2*ii+0, 9) = v*y;
    A(2*ii+0, 10) = v*z;
    A(2*ii+0, 11) = v;

    // First 4 columns, bottom row of block.
    A(2*ii+1, 0) = x;
    A(2*ii+1, 1) = y;
    A(2*ii+1, 2) = z;
    A(2*ii+1, 3) = 1;

    // Second 4 columns, bottom row of block.
    A(2*ii+1, 4) = 0;
    A(2*ii+1, 5) = 0;
    A(2*ii+1, 6) = 0;
    A(2*ii+1, 7) = 0;

    // Third 4 columns, bottom row of block.
    A(2*ii+1, 8) = -u*x;
    A(2*ii+1, 9) = -u*y;
    A(2*ii+1, 10) = -u*z;
    A(2*ii+1, 11) = -u;
  }

  // Get svd(A). Save some time and compute a thin U. We still need a full V.
  Eigen::JacobiSVD<Eigen::MatrixXd> svd;
  svd.compute(A, Eigen::ComputeThinU | Eigen::ComputeFullV);
  if (!svd.computeV()) {
    VLOG(1) << "Failed to compute a singular value decomposition of A matrix.";
    return false;
  }

  // Get the projection matrix elements from the SVD decomposition.
  const VectorXd P_vec = svd.matrixV().col(11).normalized();

  // Reshape the vector into the initial solution.
  initial_solution.row(0) = P_vec.topRows(4).transpose();
  initial_solution.row(1) = P_vec.middleRows(4, 4).transpose();
  initial_solution.row(2) = P_vec.bottomRows(4).transpose();

  return true;
}

bool PoseEstimator2D3D::OptimizeSolution(Matrix34d& solution) {
  // Create the cost function.
  ceres::Problem problem;

  // Create the output container and initialize with the least-squares solution.
  double P[12] = {solution(0,0), solution(0,1), solution(0,2), solution(0,3),
                  solution(1,0), solution(1,1), solution(1,2), solution(1,3),
                  solution(2,0), solution(2,1), solution(2,2), solution(2,3)};

  // Set up the problem and solve it.
  const int kNumResiduals = 1;  // each correspondence generates 1 residual.
  const int kNumVariables = 1;  // only optimizing P.

  // Make a cost function for each correspondence using the
  // GeometricProjectionError cost functor.
  for (size_t ii = 0; ii < points_2d_.size(); ++ii) {
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<
                             GeometricProjectionError, kNumResiduals, kNumVariables>(
        new GeometricProjectionError(points_2d_[ii], points_3d_[ii])), NULL, P);
  }

  ceres::Solver::Summary summary;
  ceres::Solver::Options options;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  ceres::Solve(options, &problem, &summary);

  // Store the solved variable back in 'solution'.
  if (summary.IsSolutionUsable()) {
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 4; ++col)
        solution(row, col) = P[row * 4 + col];
  }

  return summary.IsSolutionUsable();
}

bool PoseEstimator2D3D::ExtractPose(const Matrix34d& P, Pose& pose) {
  // Un-normalize the projection matrix.
  const  Matrix34d P_unnormalized = T_.inverse() * P * U_;

  // Extract camera extrinsics matrix.
  Matrix34d Rt = intrinsics_.Kinv() * P_unnormalized;

  // [R|t] is only determined up to scale. To get this scale, note that det(R)
  // must equal 1. Also note that for an nxn matrix, c^n*det(R) = det(cR).
  // Use this property to scale our matrix.
  double det = Rt.block(0, 0, 3, 3).determinant();
  if (std::abs(det) < 1e-8) {
    LOG(WARNING) << "Computed rotation has a determinant of 0.";
    return false;
  }

  // Make sure the determinant is positive.
  if (det < 0.0) {
    det *= -1.0;
    Rt *= -1.0;
  }

  // Normalize the rotation and translation.
  Rt *= std::pow(1.0 / det, 1.0 / 3.0);

  // Initialize the output pose from the rotation and translation blocks.
  pose = Pose(Rt);
  return true;
}

}  //\namespace bsfm
