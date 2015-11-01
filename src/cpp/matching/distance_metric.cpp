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

#include "distance_metric.h"

namespace bsfm {

DistanceMetric& DistanceMetric::Instance() {
  static DistanceMetric instance;
  return instance;
}

void DistanceMetric::SetMetric(const Metric& metric) {
  metric_ = metric;
}

void DistanceMetric::SetMaximumDistance(double maximum_distance) {
  maximum_distance_ = maximum_distance;
}

double DistanceMetric::Max() const {
  return maximum_distance_;
}

double DistanceMetric::operator()(const Descriptor& descriptor1,
                                  const Descriptor& descriptor2) {
  double distance = 0.0;
  switch (metric_) {
    case SCALED_L2:
      distance = GetScaledL2Distance(descriptor1, descriptor2);
      break;
    case HAMMING:
      distance = GetHammingDistance(descriptor1, descriptor2);
      break;
    // No default to catch incompatible types at compile time.
  }
  return distance;
}

bool DistanceMetric::MaybeNormalizeDescriptors(
    std::vector<Descriptor>& descriptors) const {
  bool normalized = false;
  switch (metric_) {
    case SCALED_L2:
      NormalizeDescriptors(descriptors);
      normalized = true;
      break;
    case HAMMING:
      break;
    // No default to catch incompatible types at compile time.
  }

  return normalized;
}

// Hidden constructor.
DistanceMetric::DistanceMetric()
    : maximum_distance_(std::numeric_limits<double>::max()) {
  // Set metric to the default argument of SetMetric(), defined in the header.
  SetMetric();
}

double DistanceMetric::GetScaledL2Distance(
    const Descriptor& descriptor1, const Descriptor& descriptor2) const {
  CHECK_EQ(descriptor1.size(), descriptor2.size());
  return 1.0 - descriptor1.dot(descriptor2);
}

double DistanceMetric::GetHammingDistance(const Descriptor& descriptor1,
                                          const Descriptor& descriptor2) const {
  CHECK_EQ(descriptor1.size(), descriptor2.size());
  int sum = 0;
  for (size_t ii = 0; ii < descriptor1.size(); ++ii) {
    unsigned char d1 = static_cast<unsigned char>(descriptor1(ii));
    unsigned char d2 = static_cast<unsigned char>(descriptor2(ii));
    sum += d1 ^ d2;
  }
  return static_cast<double>(sum);
}

void DistanceMetric::NormalizeDescriptors(
    std::vector<Descriptor>& descriptors) const {
  for (auto& descriptor : descriptors) {
    descriptor.normalize();
  }
}

}  //\namespace bsfm
