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

#include <matching/distance_metric.h>
#include <math/random_generator.h>

#include <Eigen/core>
#include <gtest/gtest.h>

namespace bsfm {

// The L2 distance between two (descriptor) vectors induces an inner product
// space. Test 2 of the properties of inner product spaces (the other 2,
// scalar/vector linearity, won't hold unless we square our feature vectors, so
// we don't need to care too much about that property). Check:
// 1. (Symmetry)              <x, y> = <y, x>
// 2. (Positive definiteness) <x, x> = 0 ==> x = 0
//
// Also test for the correct value.

TEST(ScaledL2Distance, TestSymmetry) {
  // Create a distance metric.
  ScaledL2Distance distance;

  Eigen::VectorXf descriptor1(64);
  Eigen::VectorXf descriptor2(64);
  for (int ii = 0; ii < 1000; ++ii) {
    descriptor1.setRandom().normalize();
    descriptor2.setRandom().normalize();

    EXPECT_NEAR(distance(descriptor1, descriptor2),
                distance(descriptor2, descriptor1), 1e-4);
  }
}

TEST(ScaledL2Distance, TestPositiveDefiniteness) {
  // Create a distance metric.
  ScaledL2Distance distance;

  Eigen::VectorXf descriptor1(64);
  for (int ii = 0; ii < 1000; ++ii) {
    descriptor1.setRandom().normalize();

    Eigen::VectorXf descriptor2 = descriptor1;
    EXPECT_NEAR(0.0, distance(descriptor1, descriptor2), 1e-4);
  }
}

TEST(ScaledL2Distance, TestValue) {
  // Create a distance metric.
  ScaledL2Distance distance;

  Eigen::VectorXf descriptor1(64);
  Eigen::VectorXf descriptor2(64);
  for (int ii = 0; ii < 1000; ++ii) {
    descriptor1.setRandom().normalize();
    descriptor2.setRandom().normalize();

    const double expected_dist = 1.0 - descriptor1.dot(descriptor2);
    EXPECT_NEAR(expected_dist, distance(descriptor1, descriptor2), 1e-4);
  }
}

}  //\namespace bsfm
