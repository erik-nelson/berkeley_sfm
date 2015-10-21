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
#include <camera/camera_intrinsics.h>
#include <camera/camera_extrinsics.h>
#include <geometry/point_3d.h>
#include <geometry/triangulation.h>
#include <matching/feature.h>
#include <matching/feature_match.h>
#include <math/random_generator.h>

#include <gtest/gtest.h>

namespace bsfm {

namespace {
const int kImageWidth = 1920;
const int kImageHeight = 1080;
const double kVerticalFov = 0.5 * M_PI;
}  //\namespace

TEST(Triangulation, TestTriangulateNoiseless) {
  // Repeatedly create a 3D point, project it into a bunch of cameras, and
  // re-triangulate the point.
  math::RandomGenerator rng(0);

  // All cameras will have the same intrinsics.
  CameraIntrinsics intrinsics;
  intrinsics.SetImageLeft(0);
  intrinsics.SetImageTop(0);
  intrinsics.SetImageWidth(kImageWidth);
  intrinsics.SetImageHeight(kImageHeight);
  intrinsics.SetVerticalFOV(kVerticalFov);
  intrinsics.SetFU(intrinsics.f_v());
  intrinsics.SetCU(0.5 * kImageWidth);
  intrinsics.SetCV(0.5 * kImageHeight);

  for (int num_cameras = 2; num_cameras < 20; ++num_cameras) {
    // Create 'num_cameras' cameras.
    std::vector<Camera> cameras;
    for (int ii = 0; ii < num_cameras; ++ii) {
      Camera camera;
      CameraExtrinsics extrinsics;
      extrinsics.TranslateX(2.0 - ii / 10.0);

      camera.SetExtrinsics(extrinsics);
      camera.SetIntrinsics(intrinsics);
      cameras.push_back(camera);
    }

    // Randomly generate a 3D point.
    bool successfully_projected = false;
    FeatureList features;
    Point3D point;
    while (!successfully_projected) {
      features.clear();
      const double x = rng.DoubleUniform(-2.0, 4.0);
      const double y = rng.DoubleUniform(3.0, 10.0);
      const double z = rng.DoubleUniform(-3.0, 3.0);

      // Project the point into every camera.
      for (int ii = 0; ii < num_cameras; ++ii) {
        double u = 0.0, v = 0.0;
        if (!cameras[ii].WorldToImage(x, y, z, &u, &v)) {
          successfully_projected = false;
          break;
        }

        features.push_back(Feature(u, v));
      }

      point = Point3D(x, y, z);
      successfully_projected = true;
    }

    // Now try to triangulate the point and make sure we get the right thing.
    Point3D triangulated;
    ASSERT_TRUE(Triangulate(features, cameras, triangulated));
    EXPECT_NEAR(point.X(), triangulated.X(), 1e-8);
    EXPECT_NEAR(point.Y(), triangulated.Y(), 1e-8);
    EXPECT_NEAR(point.Z(), triangulated.Z(), 1e-8);

  }  // increment number of cameras.
}

}  //\namespace bsfm
