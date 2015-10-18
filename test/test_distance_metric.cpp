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

#include <Eigen/Core>
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
  typedef typename ScaledL2Distance::Descriptor Descriptor;

  Descriptor descriptor1(64);
  Descriptor descriptor2(64);
  for (int ii = 0; ii < 1000; ++ii) {
    descriptor1.setRandom().normalize();
    descriptor2.setRandom().normalize();

    // Check symmetry.
    EXPECT_NEAR(distance(descriptor1, descriptor2),
                distance(descriptor2, descriptor1), 1e-4);
  }
}

TEST(ScaledL2Distance, TestPositiveDefiniteness) {
  // Create a distance metric.
  ScaledL2Distance distance;
  typedef typename ScaledL2Distance::Descriptor Descriptor;

  Descriptor descriptor1(64);
  for (int ii = 0; ii < 1000; ++ii) {
    descriptor1.setRandom().normalize();

    Descriptor descriptor2 = descriptor1;
    EXPECT_NEAR(0.0, distance(descriptor1, descriptor2), 1e-4);
  }
}

TEST(ScaledL2Distance, TestValue) {
  // Create a distance metric.
  ScaledL2Distance distance;
  typedef typename ScaledL2Distance::Descriptor Descriptor;

  Descriptor descriptor1(64);
  Descriptor descriptor2(64);
  for (int ii = 0; ii < 1000; ++ii) {
    descriptor1.setRandom().normalize();
    descriptor2.setRandom().normalize();

    const double expected_dist = 1.0 - descriptor1.dot(descriptor2);
    EXPECT_NEAR(expected_dist, distance(descriptor1, descriptor2), 1e-4);
  }
}

// For Hamming distance, test for symmetry, and also check that two binary
// descriptors have a large distance when they have 0 matches, and have a 0
// distance when they match exactly.
TEST(HammingDistance, TestSymmetry) {
  // Create a distance metric.
  HammingDistance distance;
  typedef typename HammingDistance::Descriptor Descriptor;

  Descriptor descriptor1(32);
  Descriptor descriptor2(32);
  for (int ii = 0; ii < 1000; ++ii) {
    descriptor1.setRandom();
    descriptor2.setRandom();

    // Check symmetry.
    EXPECT_EQ(distance(descriptor1, descriptor2),
              distance(descriptor2, descriptor1));
  }
}

TEST(HammingDistance, TestValue) {
  // Create a distance metric.
  HammingDistance distance;
  typedef typename HammingDistance::Descriptor Descriptor;

  Descriptor descriptor1(32);
  Descriptor descriptor2(32);
  for (int ii = 0; ii < 1000; ++ii) {
    descriptor1.setRandom();
    descriptor2.setRandom();

    int expected_dist = 0;
    for (int jj = 0; jj < 32; ++jj) {
      expected_dist += descriptor1(jj) ^ descriptor2(jj);
    }
    EXPECT_EQ(expected_dist, distance(descriptor1, descriptor2));
  }

  // Set the descriptors to exact opposites of one another.
  for (int ii = 0; ii < 32; ++ii) {
    descriptor1(ii) = static_cast<unsigned char>(ii % 2 == 0);
    descriptor2(ii) = static_cast<unsigned char>(ii % 2 != 0);
  }
  // Check that we get a maximum value (32).
  EXPECT_EQ(32, distance(descriptor1, descriptor2));

  // Now set the descriptors to be equal, and check that we get a distance of 0.
  descriptor2 = descriptor1;
  EXPECT_EQ(0, distance(descriptor1, descriptor2));
}

}  //\namespace bsfm
