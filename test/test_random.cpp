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

#include <math/random_generator.h>

#include <gtest/gtest.h>

namespace bsfm {

TEST(RandomGenerator, TestSeed) {
  for (unsigned long ii = 0; ii < 1000; ++ii) {
    // Create two random generators with the same seed.
    math::RandomGenerator rng1(ii);
    int int1 = rng1.Integer();

    math::RandomGenerator rng2(ii);
    int int2 = rng2.Integer();

    // Check that the values they generate are repeatable.
    EXPECT_EQ(int1, int2);

    // Create two random generators with seeds based on time and PID.
    math::RandomGenerator rng3(math::RandomGenerator::Seed());
    int int3 = rng3.Integer();

    // We need to wait a split second in between to ensure different timestamps.
    usleep(10);

    math::RandomGenerator rng4(math::RandomGenerator::Seed());
    int int4 = rng4.Integer();

    // Check that the values they generate are not the same.
    EXPECT_NE(int3, int4);
  }
}

TEST(RandomGenerator, TestUniform) {
  // Create a random generator.
  math::RandomGenerator rng(math::RandomGenerator::Seed());

  for (int ii = 0; ii < 1000; ++ii) {
    // Check that random numbers generated form a uniform distribution lie
    // within the bounds specified.
    int int1 = rng.IntegerUniform(0, ii);
    int int2 = rng.IntegerUniform(ii, 1000);

    EXPECT_LE(0, int1);
    EXPECT_GE(ii, int1);
    EXPECT_LE(ii, int2);
    EXPECT_GE(1000, int2);

    double double1 = rng.DoubleUniform(0.0, static_cast<double>(ii));
    double double2 = rng.DoubleUniform(static_cast<double>(ii), 1000.0);

    EXPECT_LE(0.0, double1);
    EXPECT_GE(static_cast<double>(ii), double1);
    EXPECT_LE(static_cast<double>(ii), double2);
    EXPECT_GE(1000.0, double2);

    // Check that the default bounds are also satisfied (on calls to "Integer"
    // and "Double").
    int int3 = rng.Integer();
    EXPECT_LE(0, int3);
    EXPECT_GE(RAND_MAX, int3);

    double double3 = rng.Double();
    EXPECT_LE(0.0, double3);
    EXPECT_GE(1.0, double3);

    // Do all the same tests for "Integers" and "IntegersUniform".
    std::vector<int> ints1, ints2, ints3;
    rng.Integers(10, &ints1);
    rng.IntegersUniform(10, 0, ii, &ints2);
    rng.IntegersUniform(10, ii, 1000, &ints3);

    for (const auto& integer : ints1) {
      EXPECT_LE(0, integer);
      EXPECT_GE(RAND_MAX, integer);
    }
    for (const auto& integer : ints2) {
      EXPECT_LE(0, integer);
      EXPECT_GE(ii, integer);
    }
    for (const auto& integer : ints3) {
      EXPECT_LE(ii, integer);
      EXPECT_GE(1000, integer);
    }

    // Do all the same tests for "Doubles" and "DoublesUniform".
    std::vector<double> doubles1, doubles2, doubles3;
    rng.Doubles(10, &doubles1);
    rng.DoublesUniform(10, 0.0, static_cast<double>(ii), &doubles2);
    rng.DoublesUniform(10, static_cast<double>(ii), 1000.0, &doubles3);

    for (const auto& d : doubles1) {
      EXPECT_LE(0.0, d);
      EXPECT_GE(1.0, d);
    }
    for (const auto& d : doubles2) {
      EXPECT_LE(0.0, d);
      EXPECT_GE(static_cast<double>(ii), d);
    }
    for (const auto& d : doubles3) {
      EXPECT_LE(static_cast<double>(ii), d);
      EXPECT_GE(1000.0, d);
    }
  }
}

TEST(RandomGenerator, TestGaussian) {
  // We can't really do much to systematically test drawing from a Gaussian
  // distribution, since the distribution is intentionally random... The best we
  // can do (and this is sort of valid w/ Hoeffding's inequality...) is draw a
  // ton of numbers that have an extremely high chance of being near the
  // specified mean. We can sample enough times that only 1 in every bazillion
  // users fails the test.
  math::RandomGenerator rng(math::RandomGenerator::Seed());

  const double mean = 50.0;
  const double stddev = 1.0;

  double sample_mean = 0.0;
  for (int ii = 0; ii < 10000; ++ii) {
    sample_mean += rng.DoubleGaussian(mean, stddev);
  }
  sample_mean /= 10000.0;
  EXPECT_NEAR(mean, sample_mean, 0.1);

  // Repeat, but draw the numbers all at once.
  std::vector<double> samples;
  rng.DoublesGaussian(10000, mean, stddev, &samples);

  sample_mean = 0.0;
  for (size_t ii = 0; ii < samples.size(); ++ii)
    sample_mean += samples[ii];

  sample_mean /= samples.size();
  EXPECT_NEAR(mean, sample_mean, 0.1);
}

}  //\namespace bsfm
