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

#include <gtest/gtest.h>

DEFINE_string(ransac_matched_image1, "lion1.jpg",
              "Name of the first image used to test RANSAC feature matching.");
DEFINE_string(ransac_matched_image2, "lion2.jpg",
              "Name of the second image used to test RANSAC feature matching.");
DEFINE_bool(ransac_draw_feature_matches, false,
            "If true, open a window displaying raw (noisy) feature matches and "
            "post-RANSAC inlier matches.");

namespace bsfm {

class TestRansac : public ::testing::Test {
 protected:

  // Make two camera views, and then create features and match them between the
  // views. This function can create matches that are correct, and matches that
  // are intentionally incorrect, and also add Gaussian noise to match locations.
  PairwiseImageMatch CreateFakeMatchedImagePair(int num_good_matches,
                                                int num_bad_matches,
                                                int noise_stddev = 0.0) {
    // Create a random number generator.
    math::RandomGenerator rng(0);

    // Create two cameras. By default they will have identity extrinsics.
    Camera camera1;
    Camera camera2;

    // Give the two cameras the same intrinsics.
    CameraIntrinsics intrinsics;
    intrinsics.SetImageLeft(0);
    intrinsics.SetImageTop(0);
    intrinsics.SetImageWidth(kImageWidth_);
    intrinsics.SetImageHeight(kImageHeight_);
    intrinsics.SetVerticalFOV(kVerticalFov_);
    intrinsics.SetFU(intrinsics.f_v());
    intrinsics.SetCU(0.5 * kImageWidth_);
    intrinsics.SetCV(0.5 * kImageHeight_);

    camera1.SetIntrinsics(intrinsics);
    camera2.SetIntrinsics(intrinsics);

    // Translate the 2nd camera along its X-axis to give it a known baseline.
    // Camera 2 will now be 200.0 pixels to the right of camera 1.
    camera2.MutableExtrinsics().TranslateX(2.0);

    // Create a bunch of good matches between points in 3D.
    PairwiseImageMatch matched_images_out;
    while(matched_images_out.feature_matches_.size() < num_good_matches) {
      // Since the camera's +Z faces down the world's -Y direction, make random
      // points back there somewhere.
      const double x_world = rng.DoubleUniform(-2.0, 4.0);
      const double y_world = rng.DoubleUniform(3.0, 10.0);
      const double z_world = rng.DoubleUniform(-3.0, 3.0);

      // Project each of the 3D points into the two cameras.
      double u1 = 0.0, v1 = 0.0;
      double u2 = 0.0, v2 = 0.0;
      const bool in_camera1 =
          camera1.WorldToImage(x_world, y_world, z_world, &u1, &v1);
      const bool in_camera2 =
          camera2.WorldToImage(x_world, y_world, z_world, &u2, &v2);

      // Make sure that the point is visible to both cameras. We want feature
      // matches.
      if (!(in_camera1 && in_camera2)) {
        continue;
      }

      // Store this as a feature match.
      FeatureMatch match;
      match.feature1_.u_ = u1 + rng.DoubleGaussian(0.0, noise_stddev);
      match.feature1_.v_ = v1 + rng.DoubleGaussian(0.0, noise_stddev);
      match.feature2_.u_ = u2 + rng.DoubleGaussian(0.0, noise_stddev);
      match.feature2_.v_ = v2 + rng.DoubleGaussian(0.0, noise_stddev);
      matched_images_out.feature_matches_.push_back(match);
    }

    // Create a bunch of bad matches between 3D points to see if RANSAC can
    // filter outliers. Do this by projecting different 3D points into the
    // left and right cameras. Make sure the points are pretty different in 3D,
    // otherwise we might unintentionally make good matches.
    while (matched_images_out.feature_matches_.size() <
           num_good_matches + num_bad_matches) {
      // We can generate points in a larger box now.
      Eigen::Vector3d x_w1, x_w2;
      x_w1.x() = rng.DoubleUniform(-2.0, 4.0);
      x_w1.y() = rng.DoubleUniform(3.0, 10.0);
      x_w1.z() = rng.DoubleUniform(-3.0, 3.0);
      x_w2.x() = rng.DoubleUniform(-2.0, 4.0);
      x_w2.y() = rng.DoubleUniform(3.0, 10.0);
      x_w2.z() = rng.DoubleUniform(-3.0, 3.0);

      // Make sure the points are different enough in 3D.
      if ((x_w1 - x_w2).squaredNorm() < 5.0) {
        continue;
      }

      // Project each of the 3D points into the two cameras.
      double u1 = 0.0, v1 = 0.0;
      double u2 = 0.0, v2 = 0.0;
      const bool in_camera1 =
          camera1.WorldToImage(x_w1.x(), x_w1.y(), x_w1.z(), &u1, &v1);
      const bool in_camera2 =
          camera2.WorldToImage(x_w2.x(), x_w2.y(), x_w2.z(), &u2, &v2);

      // Make sure that the point is visible to both cameras. We want feature
      // matches.
      if (!(in_camera1 && in_camera2)) {
        continue;
      }

      // Store the 2 different projected 3D points as a match. This is a
      // purposely incorrect feature match to test RANSAC's ability to filter
      // outliers.
      FeatureMatch match;
      match.feature1_.u_ = u1 + rng.DoubleGaussian(0.0, noise_stddev);
      match.feature1_.v_ = v1 + rng.DoubleGaussian(0.0, noise_stddev);
      match.feature2_.u_ = u2 + rng.DoubleGaussian(0.0, noise_stddev);
      match.feature2_.v_ = v2 + rng.DoubleGaussian(0.0, noise_stddev);
      matched_images_out.feature_matches_.push_back(match);
    }

    return matched_images_out;
  }

  const std::string test_image1 = strings::JoinFilepath(
      BSFM_TEST_DATA_DIR, FLAGS_ransac_matched_image1.c_str());
  const std::string test_image2 = strings::JoinFilepath(
      BSFM_TEST_DATA_DIR, FLAGS_ransac_matched_image2.c_str());

 private:
  const int kImageWidth_ = 1920;
  const int kImageHeight_ = 1080;
  const double kVerticalFov_ = 0.5 * M_PI;
};  //\class TestRansac


TEST_F(TestRansac, TestNoiselessMatches) {
  // Test whether we can get a good estimate of the fundamental matrix when
  // there is no noise.

  // Get some data. As input, we want a set of features that are matched across
  // two images.
  const int kNumGoodMatches = 8;
  const int kNumBadMatches = 0;

  PairwiseImageMatch data =
      CreateFakeMatchedImagePair(kNumGoodMatches, kNumBadMatches);

  // Define the RANSAC problem - we are attempting to determine the fundamental
  // matrix for a set of noiseless feature correspondences in images.
  FundamentalMatrixRansacProblem problem;
  problem.SetData(data.feature_matches_);

  // Create the ransac solver, set options, and run RANSAC on the problem.
  Ransac<FeatureMatch, FundamentalMatrixRansacModel> solver;
  RansacOptions options;

  // It should only take one iteration to find the right model.
  // Every feature match should be considered an inlier with such a huge error.
  options.iterations = 1;
  options.acceptable_error = 100.0;
  options.minimum_num_inliers = 8;
  options.num_samples = 8;

  solver.SetOptions(options);
  solver.Run(problem);

  // Get the solution from the problem object.
  ASSERT_TRUE(problem.SolutionFound());

  // If check not needed, but useful to show how to implement this.
  Eigen::Matrix3d fundamental_matrix_ransac(Eigen::Matrix3d::Identity());
  if (problem.SolutionFound()) {
    const FundamentalMatrixRansacModel& model = problem.Model();
    fundamental_matrix_ransac = model.F_;

    // Make sure this model has low error on every feature
    for (const auto& feature_match : data.feature_matches_) {
      const double error = model.EvaluateEpipolarCondition(feature_match);
      EXPECT_NEAR(0.0, error, 1e-8);
    }
  }

  // There was no noise in the matches, so this should be the same thing we
  // would have gotten if we were to run the fundamental matrix solver without
  // RANSAC.
  EightPointAlgorithmSolver solver2;
  FundamentalMatrixSolverOptions options2;
  solver2.SetOptions(options2);

  Eigen::Matrix3d fundamental_matrix_ep(Eigen::Matrix3d::Identity());
  ASSERT_TRUE(solver2.ComputeFundamentalMatrix(data.feature_matches_,
                                               fundamental_matrix_ep));

  // Make sure RANSAC comes up with the same solution.
  EXPECT_TRUE(fundamental_matrix_ep.isApprox(fundamental_matrix_ransac, 1e-8));

  // This is redundant, but just as a check - the fundamental matrix computed
  // with the eight-point algorithm should also have very low error.
  for (const auto& feature_match : data.feature_matches_) {
    Eigen::Vector3d x1, x2;
    x1 << feature_match.feature1_.u_, feature_match.feature1_.v_, 1;
    x2 << feature_match.feature2_.u_, feature_match.feature2_.v_, 1;
    EXPECT_NEAR(0.0, x2.transpose() * fundamental_matrix_ep * x1, 1e-8);
  }
}


TEST_F(TestRansac, TestBadMatches) {
  // See if we get a good fundamental matrix in the presence of bad matches.
  // Make sure that the recovered fundamental matrix has low error on the
  // noiseless matches.

  // Get some data. As input, we want a set of features that are matched across
  // two images.
  const int kNumGoodMatches = 100;
  const int kNumBadMatches = 100;

  PairwiseImageMatch data =
      CreateFakeMatchedImagePair(kNumGoodMatches, kNumBadMatches);

  // Define the RANSAC problem - we are attempting to determine the fundamental
  // matrix for a set of noiseless feature correspondences in images.
  FundamentalMatrixRansacProblem problem;
  problem.SetData(data.feature_matches_);

  // Create the ransac solver, set options, and run RANSAC on the problem.
  Ransac<FeatureMatch, FundamentalMatrixRansacModel> solver;
  RansacOptions options;

  // Run RANSAC for a bunch of iterations. It is very likely that in at least 1
  // iteration, all samples will be from the set of good matches and will
  // therefore result in an error of < 1e-8.
  options.iterations = 10000;
  options.acceptable_error = 1e-8;
  options.num_samples = 8;
  options.minimum_num_inliers = kNumGoodMatches;

  solver.SetOptions(options);
  solver.Run(problem);

  // Get the solution from the problem object.
  ASSERT_TRUE(problem.SolutionFound());

  // If check not needed, but useful to show how to implement this.
  if (problem.SolutionFound()) {
    // Check that the fundamental matrix satisfies x1' * F * x2 = 0 for all of
    // the good matches (x1, x2).
    const FundamentalMatrixRansacModel& model = problem.Model();
    for (int ii = 0; ii < kNumGoodMatches; ++ii) {
      const double error =
          model.EvaluateEpipolarCondition(data.feature_matches_[ii]);
      EXPECT_NEAR(0.0, error, 1e-8);
    }
  }
}

TEST_F(TestRansac, TestNeedAtLeastEight) {
  // Make sure that the fundamental matrix ransac solver fails when we don't
  // have a sufficient number of input matches. Make sure the solver fails
  // if we have >= 8 noiseless input matches.
  const int kNumBadMatches = 0;
  for (int num_good_matches = 0; num_good_matches <= 9; ++num_good_matches) {
    PairwiseImageMatch data =
        CreateFakeMatchedImagePair(num_good_matches, kNumBadMatches);

    // Define the RANSAC problem and store data for processing.
    FundamentalMatrixRansacProblem problem;
    problem.SetData(data.feature_matches_);

    // Create the ransac solver, set options, and run RANSAC on the problem.
    Ransac<FeatureMatch, FundamentalMatrixRansacModel> solver;
    RansacOptions options;

    // RANSAC should fail if it does not find 8 inliers.
    options.iterations = 10;
    options.acceptable_error = 1e-5;
    options.minimum_num_inliers = 8;

    solver.SetOptions(options);
    solver.Run(problem);

    // Make sure we don't get a solution until we have at least 8 matches.
    if (num_good_matches < 8) {
      ASSERT_FALSE(problem.SolutionFound());
    } else {
      ASSERT_TRUE(problem.SolutionFound());
    }
  }
}

TEST_F(TestRansac, TestDrawInliers) {

  // Load two images and compute feature matches. Draw matches with and without
  // RANSAC.
  if (!FLAGS_ransac_draw_feature_matches) {
    return;
  }

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

  ransac_options.iterations = 5000;
  ransac_options.acceptable_error = 1e-3;
  ransac_options.minimum_num_inliers = 100;
  ransac_options.num_samples = 8;

  solver.SetOptions(ransac_options);
  solver.Run(problem);

  ASSERT_TRUE(problem.SolutionFound());

  drawing::DrawImageFeatureMatches(image1, image2,
                                   image_matches[0].feature_matches_,
                                   "Noisy Matched Features");

  const FeatureMatchList& inliers = problem.Inliers();
  drawing::DrawImageFeatureMatches(image1, image2, inliers,
                                   "Inlier Matched Features");
}

TEST_F(TestRansac, TestNoisyAndBadMatches) {
  // See if we get a good fundamental matrix in the presence of noisy and bad matches.
  // Make sure that the recovered fundamental matrix has low error on the
  // correct (but noisy) matches.

  // Get some data. As input, we want a set of features that are matched across
  // two images.
  const int kNumGoodMatches = 100;
  const int kNumBadMatches = 100;

  // We must also set the noise standard deviation. This is the amount of noise that
  // is added to the image coordinates of each feature.
  const double noise_stddev = 1.0;

  PairwiseImageMatch data =
      CreateFakeMatchedImagePair(kNumGoodMatches, kNumBadMatches, noise_stddev);

  // Define the RANSAC problem - we are attempting to determine the fundamental
  // matrix for a set of noiseless feature correspondences in images.
  FundamentalMatrixRansacProblem problem;
  problem.SetData(data.feature_matches_);

  // Create the ransac solver, set options, and run RANSAC on the problem.
  Ransac<FeatureMatch, FundamentalMatrixRansacModel> solver;
  RansacOptions options;

  // Run RANSAC for a bunch of iterations. It is very likely that in at least 1
  // iteration, all samples will be from the set of good matches and will
  // therefore result in an error of < 1e-2.
  options.iterations = 10000;
  options.acceptable_error = 1e-2;
  options.num_samples = 8;
  options.minimum_num_inliers = kNumGoodMatches;

  solver.SetOptions(options);
  solver.Run(problem);

  // Get the solution from the problem object.
  ASSERT_TRUE(problem.SolutionFound());

  // If check not needed, but useful to show how to implement this.
  if (problem.SolutionFound()) {
    // Check that the fundamental matrix satisfies x1' * F * x2 = 0 for all of
    // the good matches (x1, x2).
    const FundamentalMatrixRansacModel& model = problem.Model();
    for (int ii = 0; ii < kNumGoodMatches; ++ii) {
      const double error =
          model.EvaluateEpipolarCondition(data.feature_matches_[ii]);
      EXPECT_NEAR(0.0, error, 0.1);
    }
  }
}

}  //\namespace bsfm
