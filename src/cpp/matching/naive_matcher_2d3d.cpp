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

#include "naive_matcher_2d3d.h"

namespace bsfm {

NaiveMatcher2D3D::NaiveMatcher2D3D() {}

NaiveMatcher2D3D::~NaiveMatcher2D3D() {}

bool NaiveMatcher2D3D::Match(
    const FeatureMatcherOptions& options, const ViewIndex& view_index,
    const std::vector<LandmarkIndex>& landmark_indices) {
  // Copy options.
  options_ = options;

  // Make sure the view is valid.
  View::Ptr view = View::GetView(view_index);
  if (view == nullptr) {
    LOG(WARNING) << "Cannot perform 2D<-->3D matching. View must not be null.";
    return false;
  }

  // Get descriptors from unincorporated observations in the view.
  std::vector<Feature> features;
  std::vector<Descriptor> descriptors_2d;
  std::vector<size_t> observation_indices;
  std::vector<Observation::Ptr> observations = view->Observations();
  for (size_t ii = 0; ii < observations.size(); ++ii) {
    CHECK_NOTNULL(observations[ii].get());
    if (!observations[ii]->IsIncorporated()) {
      descriptors_2d.push_back(observations[ii]->Descriptor());
      features.push_back(observations[ii]->Feature());
      observation_indices.push_back(ii);
    }
  }

  // Get descriptors from landmarks.
  std::vector<Feature> projected_features;
  std::vector<Descriptor> descriptors_3d;
  descriptors_3d.reserve(landmark_indices.size());
  for (size_t ii = 0; ii < landmark_indices.size(); ii++) {
    const Landmark::Ptr landmark = Landmark::GetLandmark(landmark_indices[ii]);
    CHECK_NOTNULL(landmark.get());
    descriptors_3d.push_back(landmark->Descriptor());

    // Get the image space position of the landmark to threshold matches based
    // on image space distance. If the landmark has not been observed at all,
    // set a feature position to be at infinity such that the distance check will
    // fail on this landmark.
    if (!landmark->Observations().empty()) {
      const Observation::Ptr observation = landmark->Observations().back();
      CHECK_NOTNULL(observation.get());
      projected_features.push_back(observation->Feature());
    } else {
      projected_features.push_back(Feature(std::numeric_limits<double>::max(),
                                           std::numeric_limits<double>::max()));
    }
  }

  // Normalize descriptors if required by the distance metric.
  DistanceMetric::Instance().SetMetric(options_.distance_metric);
  DistanceMetric::Instance().MaybeNormalizeDescriptors(descriptors_2d);
  DistanceMetric::Instance().MaybeNormalizeDescriptors(descriptors_3d);

  // Compute forward matches.
  std::vector<LightFeatureMatch> forward_matches;
  ComputeOneWayMatches(descriptors_2d,
                       descriptors_3d,
                       features,
                       projected_features,
                       forward_matches);

  if (forward_matches.size() < options_.min_num_feature_matches)
    return false;

  // Compute reverse matches if needed.
  if (options_.require_symmetric_matches) {
    std::vector<LightFeatureMatch> reverse_matches;
    ComputeOneWayMatches(descriptors_3d,
                         descriptors_2d,
                         projected_features,
                         features,
                         reverse_matches);
    ComputeSymmetricMatches(reverse_matches, forward_matches);
  }

  // Symmetric matches are now stored in 'forward_matches'.
  if (forward_matches.size() < options_.min_num_feature_matches)
    return false;

  // Check how many features the user wants.
  size_t num_features_out = forward_matches.size();
  if (options_.only_keep_best_matches) {
    num_features_out =
      std::min(num_features_out, static_cast<size_t>(options_.num_best_matches));

    // Return relevant matches in sorted order.
    std::partial_sort(forward_matches.begin(),
                      forward_matches.begin() + num_features_out + 1,
                      forward_matches.end(),
                      LightFeatureMatch::SortByDistance);
  }

  // Set all matched observations as matched.
  for (size_t ii = 0; ii < num_features_out; ii++) {
    const size_t observation_index =
        observation_indices[forward_matches[ii].feature_index1_];
    const LandmarkIndex landmark_index =
        landmark_indices[forward_matches[ii].feature_index2_];

    observations[observation_index]->SetMatchedLandmark(landmark_index);
  }

  return true;
}

// Compute one-way matches.
// Note: this is essentially the function
// NaiveFeatureMatcher::ComputePutativeMatches but it has been adjusted slightly
// for this 2d-3d matcher.
void NaiveMatcher2D3D::ComputeOneWayMatches(
     const std::vector<Descriptor>& descriptors1,
     const std::vector<Descriptor>& descriptors2,
     const std::vector<Feature>& features1,
     const std::vector<Feature>& features2,
     std::vector<LightFeatureMatch>& matches) {
  matches.clear();

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
      const double dist = distance(descriptors1[ii], descriptors2[jj]);
      bool close_in_image_space = true;

      // Check if the feature match is close enough in image space to be
      // considered a match.
      if (options_.threshold_image_distance) {
        const double du = features1[ii].u_ - features2[jj].u_;
        const double dv = features1[ii].v_ - features2[jj].v_;
        if (std::sqrt(du*du + dv*dv) > options_.maximum_image_distance) {
          close_in_image_space = false;
        }
      }

      // If max distance was not set above, distance.Max() will be infinity and
      // this first check (dist < distance.Max()) will always be true.
      if (dist < distance.Max() && close_in_image_space) {
        one_way_matches.emplace_back(ii, jj, dist);
      }
    }

    if (one_way_matches.empty()) {
      continue;
    }

    if (one_way_matches.size() == 1) {
      matches.emplace_back(one_way_matches[0]);
      continue;
    }

    // Sort by distance. We only care about the distances between the best 2
    // matches for the Lowes ratio test, and only care about the lowest distance
    // match if we are not using Lowes ratio.
    std::partial_sort(one_way_matches.begin(),
                      one_way_matches.begin() + 2,
                      one_way_matches.end(),
                      LightFeatureMatch::SortByDistance);

    // Store the best match for this element of features2.
    if (options_.use_lowes_ratio) {
      // The second best match must be within the lowes ratio of the best match.
      if (one_way_matches[0].distance_ <
          options_.lowes_ratio * one_way_matches[1].distance_) {
        matches.emplace_back(one_way_matches[0]);
      }
    } else {
      matches.emplace_back(one_way_matches[0]);
    }
  }
}

// Compute symmetric matches.
// Note: this is essentially the function FeatureMatcher::SymmetricMatches
// but it has been adjusted slightly for this 2d-3d matcher.
void NaiveMatcher2D3D::ComputeSymmetricMatches(
    const std::vector<LightFeatureMatch>& feature_matches_lhs,
    std::vector<LightFeatureMatch>& feature_matches_rhs) {
  // Keep track of symmetric matches in a hash table.
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
