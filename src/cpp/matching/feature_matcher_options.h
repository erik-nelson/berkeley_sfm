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
// This struct defines a set of options that are used during feature matching.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_MATCHING_FEATURE_MATCHER_OPTIONS_H
#define BSFM_MATCHING_FEATURE_MATCHER_OPTIONS_H

#include <string>

namespace bsfm {

struct FeatureMatcherOptions {

  // Use the lowes ratio test for feature matching. Given n potential matches
  // between a feature from this image and features in another image, we will
  // only consider this feature matched if the best 2 matches differ by at most
  // the lowes ratio.
  // i.e. store the match if:
  //   distance(best_match) < lowes_ratio^2 * distance(second_best_match)
  bool use_lowes_ratio = true;
  double lowes_ratio = 0.75;

  // The minimum number of feature matches between two images required to
  // consider the image pair a match.
  unsigned int min_num_feature_matches = 20;

  // Only store matches that are the best feature match in both directions.
  bool require_symmetric_matches = true;

  // Only return the best matches.
  bool only_keep_best_matches = false;

  // If 'only_keep_best_matches' is true, this is the number of best matches to
  // keep. Otherwise this option will not do anything.
  unsigned int num_best_matches = 100;

  // Decide whether or not to enforce a maximum distance between descriptors.
  // This is not usually necessary, as the lowes ratio test is generally
  // sufficient.
  bool enforce_maximum_descriptor_distance = false;

  // The maximum tolerable distance between two descriptors to classify them as
  // a match. This value will only be used if
  // 'enforce_maximum_descriptor_distance' is true.
  double maximum_descriptor_distance = 1.0;

  // Decide whether or not to discard matches if they are too far away in image
  // space. For 2D<-->3D matching, this will compare the image-space location of
  // the feature in the most recent view that it was observed from to its
  // current image-space position.
  bool threshold_image_distance = true;

  // The maximum tolerable image space distance (pixels) that two features can
  // be from
  // each other to consider them a match. This option does nothing if
  // 'threshold_image_distance' is set to false.
  double maximum_image_distance = 60.0;

  // The distance metric to be used for matching. Options can be found under
  // matching/distance_metric.h.
  std::string distance_metric = "SCALED_L2";

};  //\struct FeatureMatcherOptions

}  //\namespace bsfm

#endif
