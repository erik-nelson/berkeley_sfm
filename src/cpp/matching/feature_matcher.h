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
// be used to implement the specific feature matching strategy. The feature
// matcher is templated by a distance metric that will be used to compare
// distances between descriptor vectors. All matching strategies will attempt to
// do pairwise matches between all pairs of images. This is slow, with O(n^2) in
// the number of images.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_MATCHING_FEATURE_MATCHER_H
#define BSFM_MATCHING_FEATURE_MATCHER_H

#include <unordered_map>

#include <glog/logging.h>

#include "../image/image.h"
#include "../util/disallow_copy_and_assign.h"

#include "distance_metric.h"
#include "feature.h"
#include "feature_matcher_options.h"
#include "pairwise_image_match.h"

namespace bsfm {

// Template by the type of distance metric that will be used to compare
// descriptors.
template <typename DistanceMetric>
class FeatureMatcher {
 public:
  FeatureMatcher() { }
  virtual ~FeatureMatcher() { }

  // Add a single image's features for matching.
  virtual inline void AddImageFeatures(const FeatureList& image_features);

  // Add features from a set of images for matching.
  virtual inline void AddImageFeatures(
      const std::vector<FeatureList>& image_features);

  // Match images together using the input options.
  virtual inline bool MatchImages(const FeatureMatcherOptions& options,
                                  PairwiseImageMatchList& image_matches);

 protected:
  // Abstract method to match a pair of images using the input options. Override
  // this in the derived feature matching strategy class to implement it.
  virtual bool MatchImagePair(int image_index1, int image_index2,
                              FeatureMatchList& feature_matches) = 0;

  // Find the set intersection of the two sets of input feature matches, and
  // store that set in the second argument.
  virtual inline void SymmetricMatches(
      const LightFeatureMatchList& feature_matches_lhs,
      LightFeatureMatchList& feature_matches_rhs);

  // Summarize each image by its list of features. This list contains a list of
  // features for each image.
  std::vector<FeatureList> image_features_;

  // A set of options used for matching features and images.
  FeatureMatcherOptions options_;

 private:
  DISALLOW_COPY_AND_ASSIGN(FeatureMatcher)
};  //\class FeatureMatcher


// ------------------- Implementation ------------------- //

// Append features from a single image to the list of all image features.
template <typename DistanceMetric>
void FeatureMatcher<DistanceMetric>::
AddImageFeatures(const FeatureList& image_features) {
  image_features_.push_back(image_features);
}

// Append features from a set of images to the list of all image features.
template <typename DistanceMetric>
void FeatureMatcher<DistanceMetric>::
AddImageFeatures(const std::vector<FeatureList>& image_features) {
  image_features_.insert(image_features_.end(),
                         image_features.begin(),
                         image_features.end());
}

template <typename DistanceMetric>
bool FeatureMatcher<DistanceMetric>::
MatchImages(const FeatureMatcherOptions& options,
            PairwiseImageMatchList& image_matches) {
  // Store the matching options locally.
  options_ = options;

  // Iterate over all pairs of images and attempt to match them together by
  // comparing their features.
  for (size_t ii = 0; ii < image_features_.size(); ++ii) {
    // Make sure this image has features.
    if (image_features_[ii].size() == 0) {
      continue;
    }
    for (size_t jj = ii+1; jj < image_features_.size(); ++jj) {
      // Make sure this image has features.
      if (image_features_[jj].size() == 0) {
        continue;
      }

      // Create an image match object and attempt to match image ii to image jj.
      PairwiseImageMatch image_match;
      if (!MatchImagePair(ii, jj, image_match.feature_matches_)) {
        VLOG(1) << "Could not match image " << ii << " to image " << jj << ".";
        continue;
      }

      // If the image match was successful, store it.
      image_match.image1_index_ = ii;
      image_match.image2_index_ = jj;
      image_matches.push_back(image_match);
    }
  }
  // Return whether or not we found matches between any of the images.
  return image_matches.size() > 0;
}

template <typename DistanceMetric>
void FeatureMatcher<DistanceMetric>::
SymmetricMatches(const LightFeatureMatchList& feature_matches_lhs,
                 LightFeatureMatchList& feature_matches_rhs) {

  std::unordered_map<int, int> feature_indices;
  feature_indices.reserve(feature_matches_lhs.size());

  // Add all LHS matches to the map.
  for (const auto& feature_match : feature_matches_lhs) {
    feature_indices.insert(std::make_pair(feature_match.feature_index1_,
                                          feature_match.feature_index2_));
  }

  // For each match in the RHS set, search for the same match in the LHS set.
  // If the match is not symmetric, remove it from the RHS set.
  auto rhs_iter = feature_matches_rhs.begin();
  while (rhs_iter != feature_matches_rhs.end()) {
    const auto& lhs_matched_iter =
        feature_indices.find(rhs_iter->feature_index2_);

    // If a symmetric match is found, keep it in the RHS set.
    if (lhs_matched_iter != feature_indices.end()) {
      if (lhs_matched_iter->second == rhs_iter->feature_index1_) {
        ++rhs_iter;
        continue;
      }
    }

    // Remove the non-symmetric match and continue on.
    feature_matches_rhs.erase(rhs_iter);
  }
}

}  //\namespace bsfm
#endif
