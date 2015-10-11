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

#include "random_generator.h"

namespace bsfm {
namespace math {

RandomGenerator::RandomGenerator(unsigned long seed) {
  srand(seed);
}

unsigned long RandomGenerator::Seed() {
  // Hash from: http://burtleburtle.net/bob/hash/doobs.html
  // TODO(eanelson): Update this to read a seed from /dev/urandom instead.
  unsigned long a = clock();
  unsigned long b = time(NULL);
  unsigned long c = getpid();
  a-=b; a-=c; a^=(c >> 13); b-=c; b-=a; b^=(a << 8); c-=a; c-=b; c^=(b >> 13);
  a-=b; a-=c; a^=(c >> 12); b-=c; b-=a; b^=(a << 16); c-=a; c-=b; c^=(b >> 5);
  a-=b; a-=c; a^=(c >> 3); b-=c; b-=a; b^=(a << 10); c-=a; c-=b; c^=(b >> 15);
  return c;
}


int RandomGenerator::Integer() {
  // TODO(eanelson: Use a threadsafe rng.
  return rand();
}

int RandomGenerator::IntegerUniform(int min, int max) {
  if (min >= max) {
    LOG(WARNING) << "min >= max. Returning min.";
    // Eat a random number anyways and return min.
    Integer();
    return min;
  }

  return min + (Integer() % static_cast<int>(max - min + 1));
}

void RandomGenerator::Integers(size_t count, std::vector<int> *integers) {
  if (integers == nullptr) {
    return;
  }

  for (size_t i = 0; i < count; ++i) {
    integers->push_back(Integer());
  }
}

void RandomGenerator::IntegersUniform(size_t count, int min, int max,
                                      std::vector<int>* integers) {
  if (integers == nullptr) {
    return;
  }

  for (size_t i = 0; i < count; ++i) {
    integers->push_back(IntegerUniform(min, max));
  }
}

double RandomGenerator::Double() {
  return static_cast<double>(Integer()) / static_cast<double>(RAND_MAX);
}

double RandomGenerator::DoubleUniform(double min, double max) {
    if (min >= max) {
      LOG(WARNING) << "min >= max. Returning min.";
      // Eat a random number anyways and return min.
      Integer();
      return min;
    }

    double coefficient = (max - min) / static_cast<double>(RAND_MAX);
    return min + static_cast<double>(Integer()) * coefficient;
}

double RandomGenerator::DoubleGaussian(double mean, double std) {
  // Use the box-muller transform to approximate a normal distribution:
  // https://en.wikipedia.org/wiki/Box%E2%80%93Muller_transform
  // u must be \in (0, 1], and v must be \in [0, 1).
  double u = 1.0 - Double();
  double v = Double();
  double z = sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v);

  return mean + std * z;
}

void RandomGenerator::Doubles(size_t count, std::vector<double>* doubles) {
  if (doubles == nullptr) {
    return;
  }

  for (size_t i = 0; i < count; ++i) {
    doubles->push_back(Double());
  }
}

void RandomGenerator::DoublesUniform(size_t count, double min, double max,
                                     std::vector<double>* doubles) {
  if (doubles == nullptr) {
    return;
  }

  for (size_t i = 0; i < count; ++i) {
    doubles->push_back(DoubleUniform(min, max));
  }
}

void RandomGenerator::DoublesGaussian(size_t count, double mean, double std,
                                      std::vector<double>* doubles) {
  if (doubles == nullptr) {
    return;
  }

  for (size_t i = 0; i < count; ++i) {
    doubles->push_back(DoubleGaussian(mean, std));
  }
}

}  //\namespaces math
}  //\namespace bsfm
