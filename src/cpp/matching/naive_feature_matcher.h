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
// This class defines a naive feature matcher. The matcher takes keypoints and
// descriptors from two images and searches for the best pairwise matches,
// a naive strategy that results in O(n^2) in the number of features.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_MATCHING_NAIVE_FEATURE_MATCHER_H
#define BSFM_MATCHING_NAIVE_FEATURE_MATCHER_H

#include <algorithm>
#include <memory>
#include <vector>

#include "distance_metric.h"
#include "feature.h"
#include "feature_match.h"
#include "feature_matcher.h"
#include "pairwise_image_match.h"

#include "../image/image.h"
#include "../util/disallow_copy_and_assign.h"
#include "../util/types.h"

namespace bsfm {

class NaiveFeatureMatcher : public FeatureMatcher {
 public:
  NaiveFeatureMatcher() { }
  virtual ~NaiveFeatureMatcher() { }

 private:
  DISALLOW_COPY_AND_ASSIGN(NaiveFeatureMatcher)

  // Match two images together by doing a pairwise comparison of all of their
  // individual feature descriptors.
  virtual bool MatchImagePair(int image_index1, int image_index2,
                              PairwiseImageMatch& feature_matches);

  // Compute putative matches between feature descriptors for an image pair.
  // These might be removed later on due to e.g. not being symmetric, etc.
  void ComputePutativeMatches(const std::vector<Descriptor>& descriptors1,
                              const std::vector<Descriptor>& descriptors2,
                              std::vector<LightFeatureMatch>& putative_matches);
};  //\class NaiveFeatureMatcher

}  //\namespace bsfm

#endif
