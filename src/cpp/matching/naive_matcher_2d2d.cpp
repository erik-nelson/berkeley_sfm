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

#include "naive_matcher_2d2d.h"

namespace bsfm {

bool NaiveMatcher2D2D::MatchImagePair(
    int image_index1, int image_index2,
    PairwiseImageMatch& image_match) {
  image_match.feature_matches_.clear();
  image_match.descriptor_indices1_.clear();
  image_match.descriptor_indices2_.clear();

  // Get the descriptors corresponding to these two images.
  std::vector<Descriptor>& descriptors1 = image_descriptors_[image_index1];
  std::vector<Descriptor>& descriptors2 = image_descriptors_[image_index2];

  // Normalize descriptors if required by the distance metric.
  DistanceMetric::Instance().SetMetric(options_.distance_metric);
  DistanceMetric::Instance().MaybeNormalizeDescriptors(descriptors1);
  DistanceMetric::Instance().MaybeNormalizeDescriptors(descriptors2);

  // Compute forward (and reverse, if applicable) matches.
  LightFeatureMatchList light_feature_matches;
  ComputePutativeMatches(descriptors1,
                         descriptors2,
                         image_index1,
                         image_index2,
                         light_feature_matches);

  // Check that we got enough matches here. If we didn't, reverse matches won't
  // help us.
  if (light_feature_matches.size() < options_.min_num_feature_matches) {
    return false;
  }

  if (options_.require_symmetric_matches) {
    LightFeatureMatchList reverse_light_feature_matches;
    ComputePutativeMatches(descriptors2,
                           descriptors1,
                           image_index2,
                           image_index1,
                           reverse_light_feature_matches);
    SymmetricMatches(reverse_light_feature_matches, light_feature_matches);
  }

  if (light_feature_matches.size() < options_.min_num_feature_matches) {
    return false;
  }

  // Check how many features the user wants.
  unsigned int num_features_out = light_feature_matches.size();
  if (options_.only_keep_best_matches) {
    num_features_out =
        std::min(num_features_out, options_.num_best_matches);

    // Return relevant matches in sorted order.
    std::partial_sort(light_feature_matches.begin(),
                      light_feature_matches.begin() + num_features_out + 1,
                      light_feature_matches.end(),
                      LightFeatureMatch::SortByDistance);
  }

  // Convert from LightFeatureMatchList to FeatureMatchList for the output.
  for (int ii = 0; ii < num_features_out; ++ii) {
    const auto& match = light_feature_matches[ii];

    const Feature& matched_feature1 =
        image_features_[image_index1][match.feature_index1_];
    const Feature& matched_feature2 =
        image_features_[image_index2][match.feature_index2_];

#if 0
    // Check that the features are close in image space.
    if (options_.threshold_image_distance) {
      const double du = matched_feature1.u_ - matched_feature2.u_;
      const double dv = matched_feature1.v_ - matched_feature2.v_;
      if (std::sqrt(du*du + dv*dv) > options_.maximum_image_distance) {
        continue;
      }
    }
#endif
    image_match.feature_matches_.emplace_back(matched_feature1,
                                              matched_feature2);

    // Also store the index of each descriptor used for this match.
    image_match.descriptor_indices1_.push_back(match.feature_index1_);
    image_match.descriptor_indices2_.push_back(match.feature_index2_);
  }

  return true;
}

void NaiveMatcher2D2D::ComputePutativeMatches(
    const std::vector<Descriptor>& descriptors1,
    const std::vector<Descriptor>& descriptors2, int image_index1,
    int image_index2, std::vector<LightFeatureMatch>& putative_matches) {
  putative_matches.clear();

  // Get the singletone distance metric for descriptor comparison.
  DistanceMetric& distance = DistanceMetric::Instance();

  // Set the maximum tolerable distance between descriptors, if applicable.
  if (options_.enforce_maximum_descriptor_distance) {
    distance.SetMaximumDistance(options_.maximum_descriptor_distance);
  }

  // Store all matches and their distances.
  for (size_t ii = 0; ii < descriptors1.size(); ++ii) {
    LightFeatureMatchList one_way_matches;
    for (size_t jj = 0; jj < descriptors2.size(); ++jj) {
      double dist = distance(descriptors1[ii], descriptors2[jj]);

      // Check that the features are close in image space.
      bool close_in_image_space = true;
      const Feature& feature1 = image_features_[image_index1][ii];
      const Feature& feature2 = image_features_[image_index2][jj];
      if (options_.threshold_image_distance) {
        const double du = feature1.u_ - feature2.u_;
        const double dv = feature1.v_ - feature2.v_;
        if (std::sqrt(du * du + dv * dv) > options_.maximum_image_distance) {
          close_in_image_space = false;
        }
      }

      // If max distance was not set above, distance.Max() will be infinity and
      // the first check will always be true (dist < distance.Max()).
      if (dist < distance.Max() && close_in_image_space) {
        one_way_matches.emplace_back(ii, jj, dist);
      }
    }

    if (one_way_matches.empty()) {
      continue;
    }

    if (one_way_matches.size() == 1) {
      putative_matches.emplace_back(one_way_matches[0]);
      continue;
    }

    // Sort by distance. We only care about the distances between the best 2
    // matches for the Lowes ratio test, and about the best match if we are not
    // using Lowes ratio.
    std::partial_sort(one_way_matches.begin(),
                      one_way_matches.begin() + 2,
                      one_way_matches.end(),
                      LightFeatureMatch::SortByDistance);

    // Store the best match for this element of features2.
    if (options_.use_lowes_ratio && one_way_matches.size() > 1) {
      // The second best match must be within the lowes ratio of the best match.
      if (one_way_matches[0].distance_ <
          options_.lowes_ratio * one_way_matches[1].distance_) {
        putative_matches.emplace_back(one_way_matches[0]);
      }
    } else {
      putative_matches.emplace_back(one_way_matches[0]);
    }
  }
}

}  //\namespace bsfm
