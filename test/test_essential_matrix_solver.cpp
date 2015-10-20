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

#include <camera/camera.h>
#include <camera/camera_extrinsics.h>
#include <camera/camera_intrinsics.h>
#include <geometry/eight_point_algorithm_solver.h>
#include <image/drawing_utils.h>
#include <image/image.h>
#include <matching/feature_match.h>
#include <matching/distance_metric.h>
#include <matching/naive_feature_matcher.h>
#include <matching/pairwise_image_match.h>
#include <math/random_generator.h>
#include <ransac/fundamental_matrix_ransac_problem.h>
#include <ransac/ransac.h>
#include <ransac/ransac_options.h>
#include <strings/join_filepath.h>

#include <geometry/essential_matrix_solver.h>

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <gflags/gflags.h>

DEFINE_string(essential_matrix_image1, "campanile_view1.jpg",
              "Name of the first image used to test RANSAC feature matching.");
DEFINE_string(essential_matrix_image2, "campanile_view2.jpg",
              "Name of the second image used to test RANSAC feature matching.");

namespace bsfm {

namespace {
const int kImageWidth = 3264.0;
const int kImageHeight = 2448.0;
const double kVerticalFov = 90.0 * M_PI / 180.0;
const int kFeatureMatches = 500;
const std::string test_image1 = strings::JoinFilepath(
      BSFM_TEST_DATA_DIR, FLAGS_essential_matrix_image1.c_str());
const std::string test_image2 = strings::JoinFilepath(
      BSFM_TEST_DATA_DIR, FLAGS_essential_matrix_image2.c_str());
} //\namespace

TEST(EssentialMatrixSolver, TestEssentialMatrixSolver) {

  // Start out by running ransac to get the fundamental matrix.

  // Get some noisy feature matches.
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

}  //\namespace bsfm
