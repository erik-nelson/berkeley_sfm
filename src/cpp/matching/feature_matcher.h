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
// This class defines a base class for feature matching. A derived class should
// be used to implement the specific feature matching strategy.  All matching
// strategies will attempt to do pairwise matches between all pairs of input
// images. This is slow, with O(n^2) in the number of images.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_MATCHING_FEATURE_MATCHER_H
#define BSFM_MATCHING_FEATURE_MATCHER_H

#include <unordered_map>

#include <glog/logging.h>

#include "distance_metric.h"
#include "feature.h"
#include "feature_matcher_options.h"
#include "pairwise_image_match.h"

#include "../image/image.h"
#include "../util/disallow_copy_and_assign.h"
#include "../util/types.h"

namespace bsfm {

class FeatureMatcher {
 public:
  FeatureMatcher() { }
  virtual ~FeatureMatcher() { }

  // Add a single image's features for matching.
  virtual void AddImageFeatures(
      const std::vector<Feature>& image_features,
      const std::vector<Descriptor>& image_descriptors);

  // Add features from a set of images for matching.
  virtual void AddImageFeatures(
      const std::vector<std::vector<Feature> >& image_features,
      const std::vector<std::vector<Descriptor> >& image_descriptors);

  // Match images together using the input options.
  virtual bool MatchImages(const FeatureMatcherOptions& options,
                           PairwiseImageMatchList& image_matches);

 protected:
  // Abstract method to match a pair of images using the input options. Override
  // this in the derived feature matching strategy class to implement it.
  virtual bool MatchImagePair(int image_index1, int image_index2,
                              PairwiseImageMatch& image_match) = 0;

  // Find the set intersection of the two sets of input feature matches, and
  // store that set in the second argument.
  virtual void SymmetricMatches(
      const std::vector<LightFeatureMatch>& feature_matches_lhs,
      std::vector<LightFeatureMatch>& feature_matches_rhs);

  // Each image has a list of features and descriptors (one per feature).
  std::vector<std::vector<Feature> > image_features_;
  std::vector<std::vector<Descriptor> > image_descriptors_;

  // A set of options used for matching features and images.
  FeatureMatcherOptions options_;

 private:
  DISALLOW_COPY_AND_ASSIGN(FeatureMatcher)

};  //\class FeatureMatcher

}  //\namespace bsfm
#endif
