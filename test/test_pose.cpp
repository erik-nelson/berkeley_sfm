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
#include <gflags/gflags.h>
#include <iostream>

#include <geometry/rotation.h>
#include <math/random_generator.h>
#include <pose/pose.h>

#include <gtest/gtest.h>

namespace bsfm {

TEST(Pose, TestPoseAxisAngle) {
  // Create a simple rotation matrix and random translation vector.
  Eigen::Matrix3d R;
  R << cos(0.5), -sin(0.5), 0,
       sin(0.5),  cos(0.5), 0,
       0,         0,        1;

  const Eigen::Vector3d t = Eigen::Vector3d::Random();
  const Pose p1 = Pose(R, t);
  Pose p2 = p1;

  // Convert to/from axis angle representation and check nothing has changed.
  Eigen::Vector3d aa = p2.ToAxisAngle();
  p2.FromAxisAngle(aa);
  EXPECT_TRUE(p1.IsApprox(p2));
}

TEST(Pose, TestPoseDelta) {

  math::RandomGenerator rng(0);

  for (int ii = 0; ii < 100; ++ii) {
    // Repeatedly make two poses, compose them, and compute the relative
    // transformation between them. Check that this value is correct.
    const double phi1 = rng.DoubleUniform(-M_PI, M_PI);
    const double phi2 = rng.DoubleUniform(-M_PI, M_PI);
    const double theta1 = rng.DoubleUniform(-M_PI, M_PI);
    const double theta2 = rng.DoubleUniform(-M_PI, M_PI);
    const double psi1 = rng.DoubleUniform(-M_PI, M_PI);
    const double psi2 = rng.DoubleUniform(-M_PI, M_PI);

    // Make 2 rotation matrices.
    const Eigen::Matrix3d R1(EulerAnglesToMatrix(phi1, theta1, psi1));
    const Eigen::Matrix3d R2(EulerAnglesToMatrix(phi2, theta2, psi2));
    const Eigen::Vector3d t1(Eigen::Vector3d::Random());
    const Eigen::Vector3d t2(Eigen::Vector3d::Random());

    // Compose the two poses.
    Pose p1(R1, t1);
    Pose p2(R2, t2);
    Pose composed = p1 * p2;

    // Get the deltas between p1 and p2, and between p2 and p1. These should be
    // inverses of one another.
    Pose p12 = p1.Delta(p2);
    Pose p21 = p2.Delta(p1);
    EXPECT_TRUE(p12.IsApprox(p21.Inverse()));

    // Make sure the deltas are what we would expect.
    Eigen::Matrix4d Rt1(Eigen::Matrix4d::Identity());
    Rt1.block(0, 0, 3, 3) = R1;
    Rt1.block(0, 3, 3, 1) = t1;

    Eigen::Matrix4d Rt2(Eigen::Matrix4d::Identity());
    Rt2.block(0, 0, 3, 3) = R2;
    Rt2.block(0, 3, 3, 1) = t2;

    Eigen::Matrix4d expected_delta12 = Rt1.inverse() * Rt2;
    Eigen::Matrix4d expected_delta21 = Rt2.inverse() * Rt1;
    EXPECT_TRUE(expected_delta12.isApprox(p12.Get()));
    EXPECT_TRUE(expected_delta21.isApprox(p21.Get()));
  }

}

} // namespace bsfm
