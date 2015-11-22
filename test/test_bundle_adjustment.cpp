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

  // Create landmarks and observations for each 3D point in each view.
  for (const auto& p : points) {
    Descriptor descriptor(Descriptor::Random(32));
    Landmark::Ptr landmark = Landmark::Create();

    double u = 0.0, v = 0.0;
    for (size_t ii = 0; ii < cameras.size(); ++ii) {
      EXPECT_TRUE(cameras[ii].WorldToImage(p.X(), p.Y(), p.Z(), &u, &v));
      Feature feature(u, v);

      Observation::Ptr observation =
          Observation::Create(View::GetView(ii), feature, descriptor);
      if (!landmark->IncorporateObservation(observation))
        continue;
    }
  }

  // Bundle adjust over the views and points that were created.
  BundleAdjuster bundle_adjuster;
  BundleAdjustmentOptions options;

  std::vector<ViewIndex> view_indices;
  for (ViewIndex ii = 0; ii < View::NumExistingViews(); ++ii)
    view_indices.push_back(ii);

  EXPECT_TRUE(bundle_adjuster.Solve(options, view_indices));

  // Check that landmarks were bundle adjusted to their original positions.
  for (size_t ii = 0; ii < points.size(); ++ii) {
    Landmark::Ptr landmark = Landmark::GetLandmark(ii);
    EXPECT_NEAR(points[ii].X(), landmark->Position().X(), 1e-8);
    EXPECT_NEAR(points[ii].Y(), landmark->Position().Y(), 1e-8);
    EXPECT_NEAR(points[ii].Z(), landmark->Position().Z(), 1e-8);
  }

  // Check that cameras were bundle adjusted to their original positions.
  for (const auto& view_index : view_indices) {
    View::Ptr view = View::GetView(view_index);
    EXPECT_TRUE(cameras[view_index].Rt().isApprox(view->Camera().Rt()));
  }

  // Clean up.
  Landmark::ResetLandmarks();
  View::ResetViews();
}

TEST(BundleAdjuster, TestManyViewsTranslationNoise) {
  // Create lots of views that observe a bunch of points. Project the features
  // into the image, and then add noise to the 3D points. Make sure bundle
  // adjustment preserves distances between all points and cameras. Note that
  // bundle adjustment only will give a unique solution if overconstrained, i.e.
  // we need 2*n*m >= 3*n + 6*m, where n is the number of points and m is the
  // number of cameras. This is satisfied for, e.g. n = m = 20.

  // Clean up from other tests.
  Landmark::ResetLandmarks();
  View::ResetViews();

  // Make 3D points.
  math::RandomGenerator rng(0);
  Point3DList points;
  MakePoints(20, rng, points);

  // Make random cameras.
  std::vector<Camera> cameras;
  for (int ii = 0; ii < 20; ++ii) {
    cameras.push_back(RandomCamera(rng, points));
    View::Create(cameras.back());
  }

  // Create landmarks and observations for each 3D point in each view.
  for (const auto& p : points) {
    Descriptor descriptor(Descriptor::Random(32));
    Landmark::Ptr landmark = Landmark::Create();

    double u = 0.0, v = 0.0;
    for (size_t ii = 0; ii < cameras.size(); ++ii) {
      EXPECT_TRUE(cameras[ii].WorldToImage(p.X(), p.Y(), p.Z(), &u, &v));
      Feature feature(u, v);

      Observation::Ptr observation =
          Observation::Create(View::GetView(ii), feature, descriptor);
      EXPECT_TRUE(landmark->IncorporateObservation(observation));
    }

    // Now that the landmark has been triangulated, corrupt it with some
    // translation noise.
    Vector3d old_position = landmark->Position().Get();
    Vector3d noise(Vector3d::Random()*5.0);
    landmark->SetPosition(Point3D(old_position + noise));
  }

  // Our test invariant will be the distances between points and cameras.
  std::vector<double> distances;
  for (const auto& point : points) {
    for (const auto& camera : cameras) {
      const Vector3d delta = point.Get() - camera.Translation();
      distances.push_back(delta.norm());
    }
  }

  // Bundle adjust over the views and points that were created.
  BundleAdjuster bundle_adjuster;
  BundleAdjustmentOptions options;

  std::vector<ViewIndex> view_indices;
  for (ViewIndex ii = 0; ii < View::NumExistingViews(); ++ii)
    view_indices.push_back(ii);

  // Test bundle adjustment using all solver types.
  const std::vector<std::string> solver_types = {
      "DENSE_QR",    "DENSE_NORMAL_CHOLESKY", "SPARSE_NORMAL_CHOLESKY", "CGNR",
      "DENSE_SCHUR", "SPARSE_SCHUR",          "ITERATIVE_SCHUR",        ""};

  for (const auto& solver_type : solver_types) {
    options.solver_type = solver_type;
    EXPECT_TRUE(bundle_adjuster.Solve(options, view_indices));

    // Check if all distances between points and cameras is the same, up to a
    // scale factor.
    const Vector3d p1 = Landmark::GetLandmark(0)->Position().Get();
    const Vector3d p0 = View::GetView(0)->Camera().Translation();
    const double distance_scale = distances[0] / (p1 - p0).norm();

    int iter = 0;
    for (LandmarkIndex ii = 0; ii < Landmark::NumExistingLandmarks(); ++ii) {
      for (ViewIndex jj = 0; jj < View::NumExistingViews(); ++jj) {
        const Landmark::Ptr landmark = Landmark::GetLandmark(ii);
        const View::Ptr view = View::GetView(jj);
        const Vector3d delta =
            landmark->Position().Get() - view->Camera().Translation();
        EXPECT_NEAR(distances[iter++], distance_scale * delta.norm(), 1e-4);
      }
    }
  }

  // Clean up.
  Landmark::ResetLandmarks();
  View::ResetViews();
}


}  //\namespace bsfm
