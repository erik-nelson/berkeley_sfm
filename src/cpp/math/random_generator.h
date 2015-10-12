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

#ifndef BSFM_MATH_RANDOM_GENERATOR_H
#define BSFM_MATH_RANDOM_GENERATOR_H

#include <glog/logging.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <vector>

#include "../util/disallow_copy_and_assign.h"

namespace bsfm {
namespace math {

class RandomGenerator {
 public:
  // Construct with "RandomGenerator(RandomGenerator::Seed())".
  explicit RandomGenerator(unsigned long seed);

  // Creates a damn good seed.
  static unsigned long Seed();

  // Generates a random integer in [0, RAND_MAX).
  int Integer();

  // Generates a random integer in [0, 'max').
  int IntegerUniform(int max);

  // Generates a random integer in ['min', 'max').
  int IntegerUniform(int min, int max);

  // Generates 'count' random integers in [0, RAND_MAX).
  void Integers(size_t count, std::vector<int> *integers);

  // Generates 'count' random integers between ['min', 'max').
  void IntegersUniform(size_t count, int min, int max,
                       std::vector<int> *integers);

  // Generates a random double in [0, 1).
  double Double();

  // Generates a random double in ['min' and 'max').
  double DoubleUniform(double min, double max);

  // Samples a double from a Gaussian distribution with parameters 'mean'
  // and 'stddev'.
  double DoubleGaussian(double mean, double stddev);

  // Generates 'count' random doubles in [0, 1).
  void Doubles(size_t count, std::vector<double>* doubles);

  // Generates 'count' random doubles between ['min', 'max').
  void DoublesUniform(size_t count, double min, double max,
                      std::vector<double>* doubles);

  // Samples 'count' doubles from a Gaussian distribution with parameters
  // 'mean' and 'stddev'.
  void DoublesGaussian(size_t count, double mean, double stddev,
                       std::vector<double>* doubles);

 private:
  DISALLOW_COPY_AND_ASSIGN(RandomGenerator)

};  //\class RandomGenerator

}  //\namespace math
}  //\namespace bsfm

#endif
