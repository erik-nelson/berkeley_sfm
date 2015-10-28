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

#include <Eigen/Core>
#include <Eigen/LU>
#include <gflags/gflags.h>

#include <geometry/rotation.h>
#include <math/random_generator.h>

#include <gtest/gtest.h>

namespace bsfm {

using Eigen::Matrix3d;
using Eigen::Vector3d;

TEST(Rotation, TestEulerAnglesAndMatrices) {
  // Randomly generate euler angles, convert to a matrix, then convert back.
  // Check that (1) the rotation matrix has det(R)==1, and (2), that we get the
  // same angles back.
  math::RandomGenerator rng(0);
  for (int ii = 0; ii < 1000; ++ii) {
    Vector3d e1;
    e1.setRandom();

    // Converting from rotation matrices to euler angles is only valid when phi,
    // theta, and psi are all < 0.5*PI. Otherwise the problem has multiple
    // solutions, and we can only return one of them with our function.
    e1 *= 0.5 * M_PI;

    Matrix3d R = EulerAnglesToMatrix(e1);
    EXPECT_NEAR(1.0, R.determinant(), 1e-8);

    Vector3d e2 = MatrixToEulerAngles(R);
    EXPECT_NEAR(0.0, S1Distance(e1(0), e2(0)), 1e-8);
    EXPECT_NEAR(0.0, S1Distance(e1(1), e2(1)), 1e-8);
    EXPECT_NEAR(0.0, S1Distance(e1(2), e2(2)), 1e-8);

    EXPECT_TRUE(R.isApprox(EulerAnglesToMatrix(e2), 1e-4));
  }
}

}  //\namespace bsfm
