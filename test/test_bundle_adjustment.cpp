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

#include <geometry/rotation.h>
#include <math/random_generator.h>
#include <sfm/bundle_adjuster.h>
#include <sfm/bundle_adjustment_options.h>
#include <sfm/view.h>
#include <slam/landmark.h>
#include <util/types.h>

#include <Eigen/Core>
#include <gtest/gtest.h>

namespace bsfm {

namespace {
void MakePoints(int num_points, math::RandomGenerator& rng, Point3DList& points) {
  points.clear();

  // Randomly generate cameras will be looking upwards.
  for (int ii = 0; ii < num_points; ++ii) {
    const double x = rng.DoubleUniform(-10.0, 10.0);
    const double y = rng.DoubleUniform(-10.0, 10.0);
    const double z = rng.DoubleUniform(20.0, 30.0);

    points.emplace_back(x, y, z);
  }
}

// Make a camera with a random position that observes all points in 'points'.
Camera RandomCamera(math::RandomGenerator& rng, const Point3DList& points) {

  Camera camera;
  CameraIntrinsics intrinsics;
  intrinsics.SetImageLeft(0);
  intrinsics.SetImageTop(0);
  intrinsics.SetImageWidth(1920);
  intrinsics.SetImageHeight(1080);
  intrinsics.SetVerticalFOV(D2R(90.0));
  intrinsics.SetFU(intrinsics.f_v());
  intrinsics.SetCU(960);
  intrinsics.SetCV(540);
  camera.SetIntrinsics(intrinsics);

  bool sees_all_points = false;
  while (!sees_all_points) {
    // Make a random translation and rotation.
    CameraExtrinsics extrinsics;
    const double x = rng.DoubleUniform(-2.0, 2.0);
    const double y = rng.DoubleUniform(-2.0, 2.0);
    const double z = rng.DoubleUniform(-2.0, 2.0);
    extrinsics.Translate(x, y, z);
    Vector3d euler_angles(Vector3d::Random()*D2R(20.0));
    extrinsics.Rotate(EulerAnglesToMatrix(euler_angles));
    camera.SetExtrinsics(extrinsics);

    // Make sure the camera sees all of the points.
    for (const auto& point : points) {
      double u = 0.0, v = 0.0;
      if (!camera.WorldToImage(point.X(), point.Y(), point.Z(), &u, &v)) {
        sees_all_points = false;
        break;
      }
      sees_all_points = true;
    }
  }

  return camera;
}

}  //\namespace

TEST(BundleAdjuster, TestTwoViewsNoNoise) {
  // Create two views with perfect matches that both view the same set of 3D
  // points. Bundle adjustment shouldn't change a thing.

  // Clean up from other tests.
  Landmark::ResetLandmarks();
  View::ResetViews();

  // Make 3D points.
  math::RandomGenerator rng(0);
  Point3DList points;
  MakePoints(100, rng, points);

  Camera camera1 = RandomCamera(rng, points);
  Camera camera2 = RandomCamera(rng, points);

  // Initialize a view for each camera, a landmark for each point, and an
  // observation for each sighting of each landmark in each camera.
  View::Ptr view1 = View::Create(camera1);
  View::Ptr view2 = View::Create(camera2);

  for (const auto& point : points) {
    Descriptor descriptor(Descriptor::Random(32));

    double u = 0.0, v = 0.0;
    EXPECT_TRUE(camera1.WorldToImage(point.X(), point.Y(), point.Z(), &u, &v));
    Feature feature1(u, v);
    Observation::Ptr observation1 =
        Observation::Create(view1, feature1, descriptor);

    EXPECT_TRUE(camera2.WorldToImage(point.X(), point.Y(), point.Z(), &u, &v));
    Feature feature2(u, v);
    Observation::Ptr observation2 =
        Observation::Create(view2, feature2, descriptor);

    Landmark::Ptr landmark = Landmark::Create();
    view1->AddObservation(observation1);
    view2->AddObservation(observation2);
    EXPECT_TRUE(landmark->IncorporateObservation(observation1));
    EXPECT_TRUE(landmark->IncorporateObservation(observation2));
    EXPECT_NEAR(point.X(), landmark->Position().X(), 1e-8);
    EXPECT_NEAR(point.Y(), landmark->Position().Y(), 1e-8);
    EXPECT_NEAR(point.Z(), landmark->Position().Z(), 1e-8);
  }

  view1->UpdateObservedLandmarks();
  view2->UpdateObservedLandmarks();

  // We now have 100 observations in each view, and 100 corresponding landmarks
  // that are triangulated from the observation positions. Bundle adjustment
  // should have 0 error, and should terminate without altering the positions of
  // any landmarks or cameras.
  BundleAdjuster bundle_adjuster;
  BundleAdjustmentOptions options;

  std::vector<ViewIndex> view_indices = { 0, 1 };
  EXPECT_TRUE(bundle_adjuster.Solve(options, view_indices));

  for (size_t ii = 0; ii < points.size(); ++ii) {
    Landmark::Ptr landmark = Landmark::GetLandmark(ii);
    EXPECT_NEAR(points[ii].X(), landmark->Position().X(), 1e-8);
    EXPECT_NEAR(points[ii].Y(), landmark->Position().Y(), 1e-8);
    EXPECT_NEAR(points[ii].Z(), landmark->Position().Z(), 1e-8);
  }

  EXPECT_TRUE(camera1.Rt().isApprox(view1->Camera().Rt()));
  EXPECT_TRUE(camera2.Rt().isApprox(view2->Camera().Rt()));

  // Clean up.
  Landmark::ResetLandmarks();
  View::ResetViews();
}

TEST(BundleAdjuster, TestManyViewsNoNoise) {
  // Create lots of views with perfect matches all view the same set of 3D
  // points. Bundle adjustment shouldn't change a thing.

  // Clean up from other tests.
  Landmark::ResetLandmarks();
  View::ResetViews();

  // Make 3D points.
  math::RandomGenerator rng(0);
  Point3DList points;
  MakePoints(50, rng, points);

  // Make a bunch of random cameras.
  std::vector<Camera> cameras;
  for (int ii = 0; ii < 50; ++ii) {
    cameras.push_back(RandomCamera(rng, points));
    View::Create(cameras.back());
  }

  for (const auto& p : points) {
    Descriptor descriptor(Descriptor::Random(32));
    Landmark::Ptr landmark = Landmark::Create();

    double u = 0.0, v = 0.0;
    for (size_t ii = 0; ii < cameras.size(); ++ii) {
      EXPECT_TRUE(cameras[ii].WorldToImage(p.X(), p.Y(), p.Z(), &u, &v));
      Feature feature(u, v);

      View::Ptr view = View::GetView(ii);
      Observation::Ptr observation =
          Observation::Create(view, feature, descriptor);
      view->AddObservation(observation);
      EXPECT_TRUE(landmark->IncorporateObservation(observation));
    }
    EXPECT_NEAR(p.X(), landmark->Position().X(), 1e-8);
    EXPECT_NEAR(p.Y(), landmark->Position().Y(), 1e-8);
    EXPECT_NEAR(p.Z(), landmark->Position().Z(), 1e-8);
  }

  // Bundle adjust over the views and points that were just created.
  BundleAdjuster bundle_adjuster;
  BundleAdjustmentOptions options;

  std::vector<ViewIndex> view_indices;
  for (ViewIndex ii = 0; ii < View::NumExistingViews(); ++ii)
    view_indices.push_back(ii);

  EXPECT_TRUE(bundle_adjuster.Solve(options, view_indices));

  for (size_t ii = 0; ii < points.size(); ++ii) {
    Landmark::Ptr landmark = Landmark::GetLandmark(ii);
    EXPECT_NEAR(points[ii].X(), landmark->Position().X(), 1e-8);
    EXPECT_NEAR(points[ii].Y(), landmark->Position().Y(), 1e-8);
    EXPECT_NEAR(points[ii].Z(), landmark->Position().Z(), 1e-8);
  }

  for (const auto& view_index : view_indices) {
    View::Ptr view = View::GetView(view_index);
    EXPECT_TRUE(cameras[view_index].Rt().isApprox(view->Camera().Rt()));
  }

  // Clean up.
  Landmark::ResetLandmarks();
  View::ResetViews();
}

}  //\namespace bsfm
