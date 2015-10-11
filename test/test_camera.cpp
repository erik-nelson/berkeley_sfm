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

#include <Eigen/Dense>
#include <camera/camera.h>
#include <math.h>

#include <gflags/gflags.h>
#include <gtest/gtest.h>

namespace bsfm {

TEST(Camera, TestCameraExtrinsics) {
  // Make sure that default constructor makes the extrinsics matrix identity.
  CameraExtrinsics extrinsics;

  double wx = 5.1283, wy = -1282.123, wz = 8713.1237;
  double cx = 0.0, cy = 0.0, cz = 0.0;
  extrinsics.WorldToCamera(wx, wy, wz, &cx, &cy, &cz);
  EXPECT_EQ(wx, cx);
  EXPECT_EQ(wy, cy);
  EXPECT_EQ(wz, cz);

  cx = 2397.123897, cy = -1283.127836, cz = -8129.12387;
  extrinsics.CameraToWorld(cx, cy, cz, &wx, &wy, &wz);
  EXPECT_EQ(cx, wx);
  EXPECT_EQ(cy, wy);
  EXPECT_EQ(cz, wz);

  // Make sure points are translated correctly.
  Eigen::Matrix4d w2b = Eigen::Matrix4d::Identity();
  w2b(0, 3) = 5.0;
  w2b(1, 3) = 8.0;
  w2b(2, 3) = -10.0;

  Eigen::Matrix4d b2c = Eigen::Matrix4d::Identity();
  b2c(0, 3) = -3.0;
  b2c(1, 3) = -10.0;
  b2c(2, 3) = 4.0;

  extrinsics.SetWorldToBody(w2b);
  extrinsics.SetBodyToCamera(b2c);

  cx = 0.0, cy = 0.0, cz = 0.0;
  extrinsics.WorldToCamera(wx, wy, wz, &cx, &cy, &cz);
  EXPECT_EQ(wx + 2.0, cx);
  EXPECT_EQ(wy - 2.0, cy);
  EXPECT_EQ(wz - 6.0, cz);

  wx = 0.0, wy = 0.0, wz = 0.0;
  extrinsics.CameraToWorld(cx, cy, cz, &wx, &wy, &wz);
  EXPECT_EQ(cx - 2.0, wx);
  EXPECT_EQ(cy + 2.0, wy);
  EXPECT_EQ(cz + 6.0, wz);

  Eigen::Matrix<double, 3, 4> expected_extrinsic_matrix;
  expected_extrinsic_matrix.Zero();
  expected_extrinsic_matrix(0, 0) = 1.0;
  expected_extrinsic_matrix(1, 1) = 1.0;
  expected_extrinsic_matrix(2, 2) = 1.0;
  expected_extrinsic_matrix(0, 3) = 2.0;
  expected_extrinsic_matrix(1, 3) = -2.0;
  expected_extrinsic_matrix(2, 3) = -6.0;

  EXPECT_TRUE(expected_extrinsic_matrix.isApprox(extrinsics.ExtrinsicsMatrix()));
}

TEST(Camera, TestCameraIntrinsics) {
  const int kImageWidth = 1920;
  const int kImageHeight = 1080;
  const double kVerticalFov = 90.0 * M_PI / 180.0;

  CameraIntrinsics intrinsics;
  intrinsics.SetImageLeft(0);
  intrinsics.SetImageTop(0);
  intrinsics.SetImageWidth(kImageWidth);
  intrinsics.SetImageHeight(kImageHeight);
  // Setting vertical fov also sets f_v().
  intrinsics.SetVerticalFOV(kVerticalFov);
  // Setting f_u also sets horizontal fov.
  intrinsics.SetFU(intrinsics.f_v());
  intrinsics.SetCU(0.5 * kImageWidth);
  intrinsics.SetCV(0.5 * kImageHeight);

  // No radial distortion.
  intrinsics.SetK(0.0, 0.0, 0.0, 0.0, 0.0);

  Eigen::Matrix3d expected_intrinsic_matrix = Eigen::Matrix3d::Identity();
  expected_intrinsic_matrix(0, 0) = intrinsics.f_u();
  expected_intrinsic_matrix(1, 1) = intrinsics.f_v();
  expected_intrinsic_matrix(0, 2) = intrinsics.c_u();
  expected_intrinsic_matrix(1, 2) = intrinsics.c_v();

  EXPECT_TRUE(expected_intrinsic_matrix.isApprox(intrinsics.IntrinsicsMatrix()));

  // Camera to image on point behind the camera.
  double cx = 0.0, cy = 1.5, cz = -1.0;
  EXPECT_TRUE(!intrinsics.CameraToImage(cx, cy, cz, nullptr, nullptr));
  double u = 0.0, v = 0.0;
  EXPECT_TRUE(!intrinsics.CameraToImage(cx, cy, cz, &u, &v));

  // Camera to image on point in front of camera.
  cz = 3.0;
  EXPECT_TRUE(intrinsics.CameraToImage(cx, cy, cz, &u, &v));
  EXPECT_FLOAT_EQ(0.25 * kImageHeight, v);
  EXPECT_FLOAT_EQ(0.5 * kImageWidth, u);

  cx = -3.0;
  double expected_u = kImageWidth / 2.0 - intrinsics.f_u();
  EXPECT_TRUE(intrinsics.CameraToImage(cx, cy, cz, &u, &v));
  EXPECT_FLOAT_EQ(0.25 * kImageHeight, v);
  EXPECT_FLOAT_EQ(expected_u, u);
}

TEST(Camera, TestCamera) {
  const int kImageWidth = 1920;
  const int kImageHeight = 1080;
  const double kVerticalFov = 90.0 * M_PI / 180.0;

  CameraIntrinsics intrinsics;
  intrinsics.SetImageLeft(0);
  intrinsics.SetImageTop(0);
  intrinsics.SetImageWidth(kImageWidth);
  intrinsics.SetImageHeight(kImageHeight);
  // Setting vertical fov also sets f_v().
  intrinsics.SetVerticalFOV(kVerticalFov);
  // Setting f_u also sets horizontal fov.
  intrinsics.SetFU(intrinsics.f_v());
  intrinsics.SetCU(0.5 * kImageWidth);
  intrinsics.SetCV(0.5 * kImageHeight);

  // No radial distortion.
  intrinsics.SetK(0.0, 0.0, 0.0, 0.0, 0.0);

  CameraExtrinsics extrinsics;

  // Pure translation.
  Eigen::Matrix4d w2c = Eigen::Matrix4d::Identity();
  w2c(0, 3) = 4.0;
  w2c(1, 3) = 2.0;
  w2c(2, 3) = -0.0;

  // Make some points and project them into a second camera.
  Camera camera1, camera2;
  Eigen::Matrix4d eye = Eigen::Matrix4d::Identity();
  camera1.SetIntrinsics(intrinsics);
  camera1.SetExtrinsics(Pose(eye));
  camera2.SetIntrinsics(intrinsics);
  camera2.SetExtrinsics(extrinsics);

  // TODO (eanelson): Write test case.

}

}  //\namespace bsfm
