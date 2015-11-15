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
#include <matching/feature_matcher_options.h>
#include <matching/naive_matcher_2d3d.h>
#include <geometry/point_3d.h>
#include <geometry/rotation.h>
#include <matching/feature.h>
#include <math/random_generator.h>
#include <sfm/view.h>
#include <slam/landmark.h>
#include <slam/observation.h>
#include <ransac/pnp_ransac_problem.h>
#include <ransac/ransac.h>

#include <gtest/gtest.h>
#include <gflags/gflags.h>

DEFINE_double(noise_stddev, 1.0,
	      "Additive Gaussian noise on feature coordinates.");

namespace bsfm {

using Eigen::Matrix3d;
using Eigen::Vector3d;

namespace {
// Minimum number of points to constrain the problem. See H&Z pg. 179.
const int kNumLandmarks = 1000;
const int kImageWidth = 1920;
const int kImageHeight = 1080;
const double kVerticalFov = D2R(90.0);

// Bounding volume for 3D points.
const double kMinX = -2000.0;
const double kMinY = -2000.0;
const double kMinZ = -2000.0;
const double kMaxX = 2000.0;
const double kMaxY = 2000.0;
const double kMaxZ = 2000.0;

const unsigned int kDescriptorLength = 64;

// Makes a default set of camera intrinsic parameters.
CameraIntrinsics DefaultIntrinsics() {
  CameraIntrinsics intrinsics;
  intrinsics.SetImageLeft(0);
  intrinsics.SetImageTop(0);
  intrinsics.SetImageWidth(kImageWidth);
  intrinsics.SetImageHeight(kImageHeight);
  intrinsics.SetVerticalFOV(kVerticalFov);
  intrinsics.SetFU(intrinsics.f_v());
  intrinsics.SetCU(0.5 * kImageWidth);
  intrinsics.SetCV(0.5 * kImageHeight);

  return intrinsics;
}

// Makes a random 3D point.
Point3D RandomPoint() {
  CHECK(kMinX <= kMaxX);
  CHECK(kMinY <= kMaxY);
  CHECK(kMinZ <= kMaxZ);

  static math::RandomGenerator rng(0);
  const double x = rng.DoubleUniform(kMinX, kMaxX);
  const double y = rng.DoubleUniform(kMinY, kMaxY);
  const double z = rng.DoubleUniform(kMinZ, kMaxZ);
  return Point3D(x, y, z);
}

// Creates one observation for each landmark in the scene, and adds them to the
// view. Also creates 'num_bad_matches' bad observations (of random 3D points).
// Returns indices of landmarks that were successfully projected.
std::vector<LandmarkIndex> CreateObservations(
    const std::vector<LandmarkIndex>& landmark_indices,
    ViewIndex view_index,
    unsigned int num_bad_matches,
    double noise_stddev = 0.0) {
  View::Ptr view = View::GetView(view_index);
  CHECK_NOTNULL(view.get());

  // For each landmark that projects into the view, create an observation and
  // add it to the view.
  std::vector<LandmarkIndex> projected_landmarks;
  static math::RandomGenerator rng(0);
  for (const auto& landmark_index : landmark_indices) {
    Landmark::Ptr landmark = Landmark::GetLandmark(landmark_index);
    CHECK_NOTNULL(landmark.get());

    double u = 0.0, v = 0.0;
    const Point3D point = landmark->Position();
    if (!view->Camera().WorldToImage(point.X(), point.Y(), point.Z(), &u, &v))
      continue;

    // Creating the observation automatically adds it to the view.
    Observation::Ptr observation = Observation::Create(
        view, Feature(u + rng.DoubleGaussian(0.0, noise_stddev),
                      v + rng.DoubleGaussian(0.0, noise_stddev)),
        landmark->Descriptor());
    observation->SetMatchedLandmark(landmark_index);
    projected_landmarks.push_back(landmark_index);
  }

  // Make some bad observations also.
  for (unsigned int ii = 0; ii < num_bad_matches; ++ii) {
    double u = 0.0, v = 0.0;
    Point3D point = RandomPoint();
    if (!view->Camera().WorldToImage(point.X(), point.Y(), point.Z(), &u, &v))
      continue;

    // Creating the observation automatically adds it to the view.
    Observation::Ptr observation = Observation::Create(
        view, Feature(u + rng.DoubleGaussian(0.0, noise_stddev),
                      v + rng.DoubleGaussian(0.0, noise_stddev)),
        Descriptor::Random(kDescriptorLength));

    // Match to a random landmark.
    size_t random_index =
        static_cast<size_t>(rng.IntegerUniform(0, landmark_indices.size()-1));
    observation->SetMatchedLandmark(landmark_indices[random_index]);
  }

  return projected_landmarks;
}

void TestRansac2D3D(double fraction_bad_matches, double noise_stddev) {
  // Clean up from other tests.
  Landmark::ResetLandmarks();
  View::ResetViews();

  // Make a camera with a random translation and rotation.
  Camera camera;
  CameraExtrinsics extrinsics;
  const Point3D camera_pose = RandomPoint();
  const Vector3d euler_angles(Vector3d::Random() * D2R(180.0));
  extrinsics.Translate(0.1 * camera_pose.X(),
                       0.1 * camera_pose.Y(),
                       0.1 * camera_pose.Z());
  extrinsics.Rotate(EulerAnglesToMatrix(euler_angles));
  camera.SetExtrinsics(extrinsics);
  camera.SetIntrinsics(DefaultIntrinsics());

  // Initialize a view for this camera.
  View::Ptr view = View::Create(camera);

  // Make a bunch of randomly positioned landmarks.
  for (unsigned int ii = 0; ii < kNumLandmarks; ++ii) {
    Point3D point = RandomPoint();
    Landmark::Ptr landmark = Landmark::Create();
    landmark->SetPosition(point);
    landmark->SetDescriptor(Descriptor::Random(kDescriptorLength));
  }
  std::vector<LandmarkIndex> landmark_indices =
      Landmark::ExistingLandmarkIndices();

  // Create observations of the landmarks.
  unsigned int num_bad_matches = static_cast<unsigned int>(fraction_bad_matches *
							   kNumLandmarks);
  std::vector<LandmarkIndex> projected_landmarks = CreateObservations(
      landmark_indices, view->Index(), num_bad_matches, noise_stddev);
  ASSERT_LT(0, projected_landmarks.size());

  // Make sure the distance metric (which is global across tests) is set up
  // correctly.
  DistanceMetric& distance = DistanceMetric::Instance();
  distance.SetMetric(DistanceMetric::Metric::SCALED_L2);
  distance.SetMaximumDistance(std::numeric_limits<double>::max());

  // Set up RANSAC.
  PnPRansacProblem problem;
  CameraIntrinsics intrinsics = DefaultIntrinsics();
  problem.SetIntrinsics(intrinsics);
  problem.SetData(view->Observations());

  // Run RANSAC for a bunch of iterations. It is very likely that in at least 1
  // iteration, all samples will be from the set of good matches and will
  // therefore result in an error of < 1e-8 + 10*noise_variance. We also allow
  // RANSAC to miss some inliers as noise increases.
  Ransac<Observation::Ptr, PnPRansacModel> solver;
  RansacOptions options;
  options.iterations = 200;
  options.acceptable_error = 1e-8 + 10.0 * (noise_stddev * noise_stddev);
  options.num_samples = 6;
  options.minimum_num_inliers =
    std::max(static_cast<size_t>((std::exp(-0.1 * noise_stddev)) * (1.0 - fraction_bad_matches) *
				 static_cast<double>(projected_landmarks.size())),
	     static_cast<size_t>(options.num_samples));

  solver.SetOptions(options);
  solver.Run(problem);

  // Get the solution from the problem object.
  ASSERT_TRUE(problem.SolutionFound());

  // Iterate over all inliers and make sure we have low enough reprojection error.
  Camera estimated_camera = problem.Model().camera_;
  for (auto& observation : problem.Inliers()) {

    // Unpack this observation (extract Feature and Landmark).
    Feature feature = observation->Feature();
    Landmark::Ptr landmark = observation->GetLandmark();
    CHECK_NOTNULL(landmark.get());

    // Extract position of this landmark.
    Point3D point = landmark->Position();

    // Project into this camera.
    double u = 0.0, v = 0.0;
    const bool in_camera =
        estimated_camera.WorldToImage(point.X(), point.Y(), point.Z(), &u, &v);

    // Check that the landmark projects into the image.
    ASSERT_TRUE(in_camera);

    // Compute error and return.
    double delta_u = u - feature.u_;
    double delta_v = v - feature.v_;
    double error = delta_u * delta_u + delta_v * delta_v;

    EXPECT_TRUE(error < 10.0 * options.acceptable_error);
  }

  // Clean up.
  Landmark::ResetLandmarks();
  View::ResetViews();
}

}  //\namespace

//////////////////////////////////////////////////////////////////////////////////
//
//  TESTS. Note that here we specify (in essence) a ratio of good matches to bad
//  matches. The way the bad matches are constructed (i.e. by introducing a new
//  feature and _spuriously_ matching it to an existing and pre-matched landmark)
//  means that we cannot reasonably expect RANSAC to find _all_ the correct
//  matches. When we introduce noise, we expect still fewer, and with a maximum
//  error level somewhat higher than the minimum average model error. We build
//  this into the test by accordingly decreasing the expected number of inliers
//  and incerasing the acceptable error threshold for confirming inliers.
//
//////////////////////////////////////////////////////////////////////////////////

// Test with 1 to 1 correspondence between observations in the view and existing
// landmarks.
TEST(PnPRansac2D3D, TestPnPRansac2D3DNoiseless) {
  TestRansac2D3D(0.0, 0.0);
}

// Test with many to 1 correspondence between observations in the view and existing
// landmarks.
TEST(PnPRansac2D3D, TestPnPRansac2D3DNoisy) {
  TestRansac2D3D(0.25, FLAGS_noise_stddev);
}

// Test with MANY to 1 correspondence between observations in the view and existing
// landmarks.
TEST(PnPRansac2D3D, TestPnPRansac2D3DVeryNoisy) {
  TestRansac2D3D(0.33, FLAGS_noise_stddev);
}

}  //\namespace bsfm
