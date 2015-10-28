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

///////////////////////////////////////////////////////////////////////////////
//
// The DistanceMetric class defines a singleton that computes distances between
// two descriptors. The descriptor and distance metric have global access via
// the DistanceMetric::Instance() method.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_MATCHING_DISTANCE_METRIC_H
#define BSFM_MATCHING_DISTANCE_METRIC_H

#include <Eigen/Core>
#include <glog/logging.h>
#include <limits>

#include "../util/disallow_copy_and_assign.h"
#include "../util/types.h"

namespace bsfm {

class DistanceMetric {
 public:
  // Possible metrics.
  enum Metric {
    SCALED_L2,
    HAMMING
  };

  // Get the singleton instance of the distance metric.
  static DistanceMetric& Instance();

  // Set distance metric type.
  void SetMetric(const Metric& metric = Metric::SCALED_L2);

  // Set a maximum tolerable distance between two descriptors. This is not
  // required, but is useful for comparisons like:
  //
  // DistanceMetric& distance = DistanceMetric::Instance();
  // if (distance(descriptor1, descriptor2) < DistanceMetric::Max()) {
  //    // This is a good match.
  //  }
  //
  //  By default, a maximum distance is set to infinity, so if this function is
  //  not called, distance(descriptor1, descriptor2) < DistanceMetric::Max()
  //  will always evaluate to true.
  void SetMaximumDistance(double maximum_distance);

  // Returns the maximum tolerable distance between two descriptors. If this
  // value has not been set with 'SetMaximumDistance', returns 0.
  double Max() const;

  // Functor method computes distance between two input descriptors.
  double operator()(const Descriptor& descriptor1,
                    const Descriptor& descriptor2);

  // Depending on the distance metric used, normalize descriptors.
  bool MaybeNormalizeDescriptors(std::vector<Descriptor>& descriptors) const;

 private:
  // Ensure that DistanceMetric is a singleton.
  DISALLOW_COPY_AND_ASSIGN(DistanceMetric)
  DistanceMetric();

  // Compute the L2 norm of the difference between two descriptor vectors. If both
  // descriptors have unit length, the L2 norm is equal to 2*(1-x.y). Since all
  // distances are computed this way, we can drop the leading 2*. The L2 norm
  // induces an inner product space over R^{n}, and we can test as such.
  double GetScaledL2Distance(const Descriptor& descriptor1,
                             const Descriptor& descriptor2) const;

  // Compute the Hamming distance between two binary descriptor vectors. This is
  // the number of bits that are in disagreement (i.e. bit1 ^ bit2 == 1).
  double GetHammingDistance(const Descriptor& descriptor1,
                            const Descriptor& descriptor2) const;

  // Normalize all input descriptor vectors.
  void NormalizeDescriptors(std::vector<Descriptor>& descriptors) const;

  // The distance metric that will be used.
  Metric metric_;

  // A maximum tolerable distance between two descriptors. Defaults to
  // std::numeric_limits<double>::max(), which would imply that all descriptors
  // match to one another.
  double maximum_distance_;

};  //\class DistanceMetric

}  //\namespace bsfm

#endif
