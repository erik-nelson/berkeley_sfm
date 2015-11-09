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

///////////////////////////////////////////////////////////////////////////////
//
// This class provides a naive O(n^2) descriptor matcher for matching 2d
// Features to 3D Landmarks. Right now, this class does not inherit from
// FeatureMatcher because that class assumes we are matching Features between
// two or more images, and here we are not doing that precisely.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_MATCHING_NAIVE_MATCHER_2D3D_H
#define BSFM_MATCHING_NAIVE_MATCHER_2D3D_H

#include <algorithm>
#include <memory>
#include <vector>

#include "distance_metric.h"
#include "feature.h"
#include "feature_match.h"
#include "feature_matcher_options.h"

#include "../sfm/view.h"
#include "../slam/landmark.h"
#include "../slam/observation.h"
#include "../util/disallow_copy_and_assign.h"

namespace bsfm {

class NaiveMatcher2D3D {
 public:
  NaiveMatcher2D3D();
  ~NaiveMatcher2D3D();

  // Match observations stored in the view referred to by 'view_index' with
  // landmarks in 'landmark_indices'. Update all matched observations in the
  // view.
  bool Match(const FeatureMatcherOptions& options,
             const ViewIndex& view_index,
             const std::vector<LandmarkIndex>& landmark_indices);

 private:
  DISALLOW_COPY_AND_ASSIGN(NaiveMatcher2D3D)

  // Compute one-way matches.
  // Note: this is essentially the function
  // NaiveFeatureMatcher::ComputePutativeMatches but it has been adjusted
  // slightly for this 2d-3d matcher.
  void ComputeOneWayMatches(const std::vector<Descriptor>& descriptors1,
                            const std::vector<Descriptor>& descriptors2,
                            std::vector<LightFeatureMatch>& matches);

  // Compute symmetric matches.
  // Note: this is essentially the function FeatureMatcher::SymmetricMatches
  // but it has been adjusted slightly for this 2d-3d matcher.
  void ComputeSymmetricMatches(
      const std::vector<LightFeatureMatch>& feature_matches_lhs,
      std::vector<LightFeatureMatch>& feature_matches_rhs);

  // Feature matching options.
  FeatureMatcherOptions options_;

};  //\class NaiveMatcher2D3D

}  //\namespace bsfm

#endif
