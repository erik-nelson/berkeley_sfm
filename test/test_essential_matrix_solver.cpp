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
 * Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 *          Erik Nelson            ( eanelson@eecs.berkeley.edu )
 */

#include <geometry/essential_matrix_solver.h>

#include <camera/camera.h>
#include <camera/camera_extrinsics.h>
#include <camera/camera_intrinsics.h>
#include <geometry/eight_point_algorithm_solver.h>
#include <geometry/rotation.h>
#include <matching/feature_match.h>
#include <matching/distance_metric.h>
#include <matching/naive_feature_matcher.h>
#include <matching/pairwise_image_match.h>
#include <math/random_generator.h>
#include <ransac/fundamental_matrix_ransac_problem.h>
#include <ransac/ransac.h>
#include <ransac/ransac_options.h>

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <gflags/gflags.h>

namespace bsfm {

using Eigen::Matrix3d;
using Eigen::Vector3d;

namespace {
const int kImageWidth = 1920;
const int kImageHeight = 1080;
const double kVerticalFov = 0.5 * M_PI;
const int kFeatureMatches = 20;
} //\namespace

TEST(EssentialMatrixSolver, TestEssentialMatrixNoiseless) {

  // Create a random number generator.
  math::RandomGenerator rng(0);

  // Create some noiseless feature matches by generating 3D points and
  // projecting them into two cameras.
  Camera camera1;
  Camera camera2;

  // Give the two cameras the same intrinsics.
  CameraIntrinsics intrinsics;
  intrinsics.SetImageLeft(0);
  intrinsics.SetImageTop(0);
  intrinsics.SetImageWidth(kImageWidth);
  intrinsics.SetImageHeight(kImageHeight);
  intrinsics.SetVerticalFOV(kVerticalFov);
  intrinsics.SetFU(intrinsics.f_v());
  intrinsics.SetCU(0.5 * kImageWidth);
  intrinsics.SetCV(0.5 * kImageHeight);

  camera1.SetIntrinsics(intrinsics);
  camera2.SetIntrinsics(intrinsics);

  // Rotate and translate the second camera w.r.t. the first.
  CameraExtrinsics extrinsics1, extrinsics2;
  camera1.SetExtrinsics(extrinsics1);

  extrinsics2.Translate(-2.0, 1.0, 1.0);
  Vector3d euler_angles(Vector3d::Random()*D2R(20.0));
  extrinsics2.Rotate(EulerAnglesToMatrix(euler_angles));
  camera2.SetExtrinsics(extrinsics2);

  // Create a bunch of points in 3D, project, and match.
  FeatureMatchList feature_matches;
  while (feature_matches.size() < kFeatureMatches) {
    // Make some 3D points in front of both cameras.
    const double x = rng.DoubleUniform(-5.0, 5.0);
    const double y = rng.DoubleUniform(-5.0, 5.0);
    const double z = rng.DoubleUniform(4.0, 10.0);

    // Project the 3D point into each camera;
    double u1 = 0.0, v1 = 0.0;
    double u2 = 0.0, v2 = 0.0;
    const bool in_camera1 = camera1.WorldToImage(x, y, z, &u1, &v1);
    const bool in_camera2 = camera2.WorldToImage(x, y, z, &u2, &v2);

    // Make sure the point is visible to both cameras.
    if (!(in_camera1 && in_camera2)) {
      continue;
    }

    // Store this as a feature match.
    FeatureMatch match;
    match.feature1_.u_ = u1;
    match.feature1_.v_ = v1;
    match.feature2_.u_ = u2;
    match.feature2_.v_ = v2;
    feature_matches.push_back(match);
  }

  // Use the eight point algorithm to find the fundamental matrix for the set of
  // points.
  EightPointAlgorithmSolver ep_solver;
  FundamentalMatrixSolverOptions options;
  ep_solver.SetOptions(options);

  Matrix3d F;
  ASSERT_TRUE(ep_solver.ComputeFundamentalMatrix(feature_matches, F));

  // Make sure the computed fundamental matrix is valid.
  for (size_t ii = 0; ii < feature_matches.size(); ++ii) {
    const double u1 = feature_matches[ii].feature1_.u_;
    const double v1 = feature_matches[ii].feature1_.v_;
    const double u2 = feature_matches[ii].feature2_.u_;
    const double v2 = feature_matches[ii].feature2_.v_;
    Vector3d x1(u1, v1, 1);
    Vector3d x2(u2, v2, 1);
    EXPECT_NEAR(0.0, x2.transpose() * F * x1, 1e-8);
  }

  // Matches are noiseless so we shouldn't need RANSAC.
  // Try to get the essential matrix from the fundamental matrix.
  EssentialMatrixSolver e_solver;
  Matrix3d E = e_solver.ComputeEssentialMatrix(F, camera1.Intrinsics(),
                                               camera2.Intrinsics());

  // Extract the pose of camera 2 from E. This pose will be relative to the pose
  // of camera 1, so to test we will need the delta between cameras 1 and 2.
  Pose relative_pose;
  ASSERT_TRUE(e_solver.ComputeExtrinsics(E, feature_matches,
                                         camera1.Intrinsics(),
                                         camera2.Intrinsics(), relative_pose));

  // Both 'delta' and 'relative_pose' are in world frame.
  Pose c1(camera1.Rt());
  Pose c2(camera2.Rt());
  Pose delta = c1.Delta(c2);

  // Delta's translation is not normalized, but the essential matrix can only
  // give us normalized translations. Normalize for comparison.
  delta.SetTranslation(delta.Translation().normalized());

  EXPECT_TRUE(delta.IsApprox(relative_pose));
}

}  //\namespace bsfm
