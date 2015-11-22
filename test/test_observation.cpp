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
#include <geometry/point_3d.h>
#include <geometry/rotation.h>
#include <matching/feature.h>
#include <math/random_generator.h>
#include <sfm/view.h>
#include <slam/observation.h>
#include <slam/landmark.h>
#include <util/types.h>

#include <gflags/gflags.h>
#include <gtest/gtest.h>

namespace bsfm {

using Eigen::Vector3d;

namespace {
const int kImageWidth = 1920;
const int kImageHeight = 1080;
const double kVerticalFov = D2R(90.0);
}  //\namespace

TEST(Observation, TestGetters) {
  // Clear all possibly existing views and landmarks, since all tests are run in
  // the same process.
  Landmark::ResetLandmarks();
  View::ResetViews();


  // Make sure that we get null pointers when asking for bad views or landmarks,
  // and get the correct view and landmark if they are initialized properly.
  Feature feature(0.0, 0.0);
  Descriptor descriptor(Descriptor::Zero(64));

  // Disable console printouts since we expect to get null pointer warnings.
  // Also write a notification into the test log.
  FLAGS_logtostderr = false;
  LOG(INFO) << "Expect to see some null pointer warnings below.";

  // It is impossible to initialize with an invalid view due to nullity checks
  // in the observation constructor, so we can begin by checking valid views
  // with invalid landmarks.
  Camera camera;
  View::Ptr view = View::Create(camera);
  Observation::Ptr observation = Observation::Create(view, feature, descriptor);
  EXPECT_EQ(view, observation->GetView());
  EXPECT_EQ(nullptr, observation->GetLandmark());
  EXPECT_FALSE(observation->IsMatched());

  // Set a valid landmark.
  Landmark::Ptr landmark = Landmark::Create();
  landmark->SetDescriptor(descriptor);

  // Incorporate the observation with the landmark.
  EXPECT_TRUE(landmark->IncorporateObservation(observation));
  EXPECT_EQ(view, observation->GetView());
  EXPECT_EQ(landmark, observation->GetLandmark());
  EXPECT_TRUE(observation->IsMatched());

  // Turn back on print outs
  FLAGS_logtostderr = true;

  // Clean up.
  Landmark::ResetLandmarks();
  View::ResetViews();
}

TEST(Observation, TestTriangulateObservations) {
  // Clear all possibly existing views and landmarks, since all tests are run in
  // the same process.
  Landmark::ResetLandmarks();
  View::ResetViews();

  // Create a 3D point and lots of observations of it from different cameras.
  // Incorporate all of the observations into a landmark to re-triangulate the
  // 3D point's position.
  math::RandomGenerator rng(0);

  CameraIntrinsics intrinsics;
  intrinsics.SetImageLeft(0);
  intrinsics.SetImageTop(0);
  intrinsics.SetImageWidth(kImageWidth);
  intrinsics.SetImageHeight(kImageHeight);
  intrinsics.SetVerticalFOV(kVerticalFov);
  intrinsics.SetFU(intrinsics.f_v());
  intrinsics.SetCU(0.5 * kImageWidth);
  intrinsics.SetCV(0.5 * kImageHeight);
  intrinsics.SetK(0.0, 0.0, 0.0, 0.0, 0.0);

  // Run a bunch of random tests.
  for (int ii = 0; ii < 20; ++ii) {

    // Make a bunch of cameras.
    std::vector<Camera> cameras;
    for (int jj = 0; jj < 50; ++jj) {
      Camera camera;

      // Random position and orientation.
      CameraExtrinsics extrinsics;
      const double cx = rng.DoubleUniform(-2.0, 2.0);
      const double cy = rng.DoubleUniform(-2.0, 2.0);
      const double cz = rng.DoubleUniform(-2.0, 2.0);
      extrinsics.Translate(cx, cy, cz);

      Vector3d euler_angles(Vector3d::Random()*D2R(20.0));
      extrinsics.Rotate(EulerAnglesToMatrix(euler_angles));
      camera.SetExtrinsics(extrinsics);
      camera.SetIntrinsics(intrinsics);
      cameras.push_back(camera);
    }

    // Make a 3D point that can be seen by all cameras.
    bool successfully_projected = false;
    FeatureList features;
    Point3D point;
    while (!successfully_projected) {
      features.clear();
      const double x = rng.DoubleUniform(-10.0, 10.0);
      const double y = rng.DoubleUniform(-10.0, 10.0);
      const double z = rng.DoubleUniform(10.0, 20.0);

      // Project the point into every camera.
      bool in_all_cameras = true;
      for (size_t ii = 0; ii < cameras.size(); ++ii) {
        double u = 0.0, v = 0.0;
        if (!cameras[ii].WorldToImage(x, y, z, &u, &v)) {
          in_all_cameras = false;
          break;
        }

        features.push_back(Feature(u, v));
      }
      if (!in_all_cameras)
        continue;

      point = Point3D(x, y, z);
      successfully_projected = true;
    }

    // Create a view for each camera. For the feature seen in each view, create
    // an observation.
    Descriptor descriptor(Descriptor::Zero(64));
    for (size_t ii = 0; ii < cameras.size(); ++ii) {
      View::Ptr view = View::Create(cameras[ii]);
      Observation::Ptr observation =
          Observation::Create(view, features[ii], descriptor);
    }

    // Create a landmark whose position we don't know yet.
    Landmark::Ptr landmark = Landmark::Create();

    // Incorporate all observations from all views into the landmark.
    for (unsigned int ii = 0; ii < View::NumExistingViews(); ++ii) {
      for (const auto& observation : View::GetView(ii)->Observations()) {
        landmark->IncorporateObservation(observation);
      }
    }

    // Check if our landmark is now positioned on the generated 3D point.
    EXPECT_NEAR(point.X(), landmark->Position().X(), 1e-8);
    EXPECT_NEAR(point.Y(), landmark->Position().Y(), 1e-8);
    EXPECT_NEAR(point.Z(), landmark->Position().Z(), 1e-8);

    // Clean up after each iteration.
    Landmark::ResetLandmarks();
    View::ResetViews();
  }
}

}  //\namespace bsfm
