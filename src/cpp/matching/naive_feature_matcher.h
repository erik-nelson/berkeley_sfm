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

#include <memory>
#include <vector>

#include "../image/image.h"
#include "../util/disallow_copy_and_assign.h"

#include "distance_metric.h"
#include "feature.h"
#include "feature_match.h"

namespace bsfm {

template <typename DistanceMetric>
class NaiveFeatureMatcher : public FeatureMatcher<DistanceMetric> {
 public:
  NaiveFeatureMatcher() { }
  virtual ~NaiveFeatureMatcher() { }

 private:
  // Match two images together by doing a pairwise comparison of all of their
  // individual feature descriptors.
  bool MatchImagePair(int image_index1, int image_index2,
                      FeatureMatchList& feature_matches) override;

  // Compute putative matches between features for an image pair. These might be
  // removed later on due to e.g. not being symmetric, etc.
  void ComputePutativeMatches(
      const FeatureList& features1,
      const FeaturesList& features2,
      LightFeatureMatchList& putative_matches) {

  DISALLOW_COPY_AND_ASSIGN(NaiveFeatureMatcher)

};  //\class NaiveFeatureMatcher

// ------------------- Implementation ------------------- //

template <typename DistanceMetric>
bool NaiveFeatureMatcher<DistanceMetric>::MatchImagePair(
    int image_index1,
    int image_index2,
    FeatureMatchList& feature_matches) {
  feature_matches.clear();

  // Get the features corresponding to these two images.
  FeatureList& features1 = image_features_[image_index1];
  FeatureList& features2 = image_features_[image_index1];

  // Normalize descriptors if required.
  if (DistanceMetric::RequiresNormalizedDescriptors()) {
    for (auto& feature : features1) {
      feature.NormalizeDescriptor();
    }
    for (auto& feature : features2) {
      feature.NormalizeDescriptor();
    }
  }

  // Compute forward (and reverse, if applicable) matches.
  LightFeatureMatchList light_feature_matches;
  ComputePutativeMatches(features1, features2, light_feature_matches);

  // Check that we got enough matches here. If we didn't, reverse matches won't
  // help us.
  if (light_feature_matches.size() < options_.min_num_feature_matches) {
    return false;
  }

  if (options_.compute_symmetric_matches) {
    LightFeatureMatchList reverse_light_feature_matches;
    ComputePutativeMatches(features2, features1, reverse_light_feature_matches);
    SymmetricMatches(reverse_light_feature_matches, &light_feature_matches);
  }

  if (light_feature_matches.size() < options_.min_num_feature_matches) {
    return false;
  }

  // Convert from LightFeatureMatchList to FeatureMatchList for the output.
  for (const auto& light_feature_match : light_feature_matches) {
    const Feature& matched_feature1 =
        image_features_[image_index1][light_feature_match.feature_index1];
    const Feature& matched_feature2 =
        image_features_[image_index2][light_feature_match.feature_index2];
    feature_matches.emplace_back(matched_feature1, matched_feature2);
  }

  return true;
}

template <typename DistanceMetric>
void NaiveFeatureMatcher<DistanceMetric>::ComputePutativeMatches(
    const FeatureList& features1,
    const FeaturesList& features2,
    LightFeatureMatchList& putative_matches) {
  putative_matches.clear();

  // Create a distance metric for descriptor comparison.
  DistanceMetric distance;

  // Store all matches and their distances.
  for (size_t ii = 0; ii < features1.size(); ++ii) {
    LightFeatureMatchList features2_matches(features2.size());
    for (size_t jj = 0; jj < features2.size(); ++jj)
      features2_matches[jj] =
          LightFeatureMatch(ii, jj, distance(features1[ii].descriptor_,
                                             features2[ii].descriptor_));
    }

  // Store the best match for this element of features2.
  if (options_.use_lowes_ratio) {
    // Sort by distance. We only care about the distances between the best 2
    // matches for the Lowes ratio test.
    std::partial_sort(features2_matches.begin(),
                      features2_matches.begin() + 1,
                      features2_matches.end(),
                      SortByDistance);

    double lowes_ratio_squared = options_.lowes_ratio * options_.lowes_ratio;
    if (features2_matches[0].distance <
        lowes_ratio_squared * features2_matches[1].distance) {
      putative_matches.emplace_back(features2_matches[0]);
    }
  } else {
    putative_matches.emplace_back(features2_matches[0]);
  }
}

}  //\namespace bsfm

#endif
