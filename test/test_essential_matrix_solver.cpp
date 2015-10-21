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
  intrinsics.SetFU(1.0);
  intrinsics.SetFV(1.0);
  intrinsics.SetCU(0.5 * kImageWidth);
  intrinsics.SetCV(0.5 * kImageHeight);

  camera1.SetIntrinsics(intrinsics);
  camera2.SetIntrinsics(intrinsics);

  // Set extrinsics for both cameras to identity pose.
  CameraExtrinsics extrinsics1, extrinsics2;
  extrinsics1.SetWorldToCamera(Pose());
  extrinsics2.SetWorldToCamera(Pose());
  camera1.SetExtrinsics(extrinsics1);
  camera2.SetExtrinsics(extrinsics2);
    
  // Translate the second camera along its X axis. Camera 2 will be 200.0 pixels
  // to the right of camera 1.
  camera2.MutableExtrinsics().TranslateX(1.0);

  // Create a bunch of points in 3D, project, and match.
  FeatureMatchList feature_matches;
  while (feature_matches.size() < kFeatureMatches) {
    // We have set the world to camera transform in camera 1 as the identity, so
    // just set the xy bounds to be approximately the image bounds and the
    // z bounds to be in front of camera 1.
    const double x = rng.DoubleUniform(-2000.0, 2000.0);
    const double y = rng.DoubleUniform(-2000.0, 2000.0);
    const double z = rng.DoubleUniform(2000.0, 3000.0);

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
    match.feature2_.u_ = u1;
    match.feature2_.v_ = v1;
    feature_matches.push_back(match);
  }

  // Use the eight point algorithm to find the fundamental matrix for the set of
  // points.
  EightPointAlgorithmSolver ep_solver;
  FundamentalMatrixSolverOptions options;
  ep_solver.SetOptions(options);

  Eigen::Matrix3d F;
  ASSERT_TRUE(ep_solver.ComputeFundamentalMatrix(feature_matches, F));

  // Matches are noiseless so we shouldn't need RANSAC.
  // Try to get the essential matrix from the fundamental matrix.
  EssentialMatrixSolver e_solver;
  Eigen::Matrix3d E = e_solver.ComputeEssentialMatrix(F, camera1.Intrinsics(),
                                                      camera2.Intrinsics());

  // Extract the pose of camera 2 from E.
  CameraExtrinsics computed_extrinsics;
  ASSERT_TRUE(
      e_solver.ComputeExtrinsics(E, feature_matches, camera1.Intrinsics(),
                                 camera2.Intrinsics(), computed_extrinsics));

  // The true and computed camera pose should be identical.
  std::cout << "Expected: " << std::endl << camera2.Extrinsics().ExtrinsicsMatrix() << std::endl;
  std::cout << "Actual: " << std::endl << computed_extrinsics.ExtrinsicsMatrix() << std::endl;
  EXPECT_TRUE(camera2.Extrinsics().ExtrinsicsMatrix().isApprox(
      computed_extrinsics.ExtrinsicsMatrix(), 1e-4));
}

#if 0
TEST(EssentialMatrixSolver, TestEssentialMatrixSolver) {

  // Start out by running ransac to get the fundamental matrix.

  // Get some noisy feature matches.
  math::RandomGenerator rng(0);

  Image image1(test_image1.c_str());
  Image image2(test_image2.c_str());

  image1.Resize(0.25);
  image2.Resize(0.25);

  KeypointDetector detector;
  detector.SetDetector("SIFT");

  KeypointList keypoints1;
  KeypointList keypoints2;
  detector.DetectKeypoints(image1, keypoints1);
  detector.DetectKeypoints(image2, keypoints2);

  typedef typename ScaledL2Distance::Descriptor Descriptor;
  DescriptorExtractor<float> extractor;
  extractor.SetDescriptor("SIFT");

  std::vector<Feature> features1;
  std::vector<Feature> features2;
  std::vector<Descriptor> descriptors1;
  std::vector<Descriptor> descriptors2;
  extractor.DescribeFeatures(image1, keypoints1, features1, descriptors1);
  extractor.DescribeFeatures(image2, keypoints2, features2, descriptors2);

  FeatureMatcherOptions matcher_options;
  NaiveFeatureMatcher<ScaledL2Distance> feature_matcher;
  feature_matcher.AddImageFeatures(features1, descriptors1);
  feature_matcher.AddImageFeatures(features2, descriptors2);
  PairwiseImageMatchList image_matches;
  feature_matcher.MatchImages(matcher_options, image_matches);

  ASSERT_TRUE(image_matches.size() == 1);

  // RANSAC the feature matches to get inliers.
  FundamentalMatrixRansacProblem problem;
  problem.SetData(image_matches[0].feature_matches_);

  // Create the ransac solver, set options, and run RANSAC on the problem.
  Ransac<FeatureMatch, FundamentalMatrixRansacModel> solver;
  RansacOptions ransac_options;

  ransac_options.iterations = 50;
  ransac_options.acceptable_error = 1e-2;
  ransac_options.minimum_num_inliers = 20;
  ransac_options.num_samples = 8;

  solver.SetOptions(ransac_options);
  solver.Run(problem);

  ASSERT_TRUE(problem.SolutionFound());
  const FeatureMatchList& inliers = problem.Inliers();
  const Eigen::Matrix3d F = problem.Model().F_;

  // Now, create camera intrinsics and calculate the essential matrix.
  Camera cam;
  CameraIntrinsics intrinsics;
  intrinsics.SetImageLeft(0);
  intrinsics.SetImageTop(0);
  intrinsics.SetImageWidth(0.25 * kImageWidth);
  intrinsics.SetImageHeight(0.25 * kImageHeight);
  intrinsics.SetVerticalFOV(0.5 * kVerticalFov);

  // Set f_u, f_v with iPhone 5 specs.
  intrinsics.SetFU(0.25 * 4100.0 / 1.4);
  intrinsics.SetFV(0.25 * 4100.0 / 1.4);
  intrinsics.SetCU(0.25 * 0.5 * kImageWidth);
  intrinsics.SetCV(0.25 * 0.5 * kImageHeight);

  cam.SetIntrinsics(intrinsics);

  // Now, given the fundamental matrix, calculate the essential matrix.
  EssentialMatrixSolver essential_solver;
  const Eigen::Matrix3d E = essential_solver.ComputeEssentialMatrix(
      F, cam.Intrinsics(), cam.Intrinsics());

  EXPECT_TRUE(
      F.isApprox(cam.Intrinsics().IntrinsicsMatrix().transpose().inverse() * E *
                 cam.Intrinsics().IntrinsicsMatrix().inverse()));

  // Check that we can calculate extrinsics from the essential matrix.
  CameraExtrinsics estimated_extrinsics;
  EXPECT_TRUE(essential_solver.ComputeExtrinsics(
      E, inliers, cam.Intrinsics(), cam.Intrinsics(), estimated_extrinsics));
}
#endif

}  //\namespace bsfm
