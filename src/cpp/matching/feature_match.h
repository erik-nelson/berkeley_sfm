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
// These classes define a match between two image features. The FeatureMatch
// class contains the features themselves, while the LightFeatureMatch contains
// only the indices corresponding to the features in their respective images, as
// well as the distance computed between the two descriptors. The
// LightFeatureMatch class is cheaper to sort/store.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_MATCHING_FEATURE_MATCH_H
#define BSFM_MATCHING_FEATURE_MATCH_H

#include <memory>
#include <vector>

#include "../image/image.h"

namespace bsfm {

struct FeatureMatch {
  typedef std::shared_ptr<FeatureMatch> Ptr;
  typedef std::shared_ptr<const FeatureMatch> ConstPtr;

  // Basic constructor
  FeatureMatch(const Feature& feature1, const Feature& feature2)
      : feature1_(feature1), feature2_(feature2) {}

  Feature feature1_;
  Feature feature2_;
};  //\struct FeatureMatch

struct LightFeatureMatch {
  typedef std::shared_ptr<LightFeatureMatch> Ptr;
  typedef std::shared_ptr<const LightFeatureMatch> ConstPtr;

  // Basic constructor.
  LightFeatureMatch(int feature_index1, int feature_index2, double distance)
      : feature_index1_(feature_index1),
        feature_index2_(feature_index2),
        distance_(distance) {}

  // Computed distance between the two features.
  double distance_;

  // Index of the feature in each image.
  int feature_index1_;
  int feature_index2_;

  // A custom sorting function to find the match with the smallest distance.
  static bool SortByDistance(const LightFeatureMatch& lhs,
                             const LightFeatureMatch& rhs) {
    return lhs.distance_ < rhs.distance_;
  }
};  //\struct LightFeatureMatch

typedef std::vector<FeatureMatch> FeatureMatchList;
typedef std::vector<LightFeatureMatch> LightFeatureMatchList;

}  //\namespace bsfm

#endif
