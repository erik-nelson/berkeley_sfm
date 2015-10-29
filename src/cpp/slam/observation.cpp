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

#include "observation.h"

#include "landmark.h"
#include "../sfm/view.h"

namespace bsfm {

// Initialize an observation with the view that it came from, an image-space
// feature coordinate pair, and an associated descriptor. Implicitly initializes
// the landmark to be invalid, since the observation has not been matched with a
// 3D landmark yet.
Observation::Observation(
    ViewIndex view_index, const Feature::Ptr& feature_ptr,
    const std::shared_ptr<::bsfm::Descriptor>& descriptor_ptr)
    : view_index_(view_index),
      landmark_index_(kInvalidLandmark),
      is_matched_(false),
      feature_ptr_(feature_ptr),
      descriptor_ptr_(descriptor_ptr) {}

Observation::~Observation() {}

// Attempts to match a landmark to this observation. Returns false if the
// landmark's descriptor does not match this observation.
bool Observation::Match(LandmarkIndex landmark_index) {

  // Get the landmark's descriptor.
  Landmark::Ptr landmark = Landmark::GetLandmark(landmark_index);

  // Also get this observation's descriptor. Make sure both are not null, then
  // copy and normalize, if required by the distance metric.
  std::vector<::bsfm::Descriptor> descriptors;
  ::bsfm::Descriptor d1 = *(CHECK_NOTNULL(descriptor_ptr_.get()));
  ::bsfm::Descriptor d2 = *(CHECK_NOTNULL(landmark->Descriptor().get()));
  descriptors.push_back(d1);
  descriptors.push_back(d2);
  DistanceMetric& distance = DistanceMetric::Instance();
  distance.MaybeNormalizeDescriptors(descriptors);

  // Is this a good 2D<-->3D match?
  if (distance(descriptors[0], descriptors[1]) > distance.Max()) {
    VLOG(1) << "Observation was not matched to landmark " << landmark_index;
    return false;
  }

  // If the observation and landmark match, store the landmark's index so that
  // we can access it with Landmark::GetLandmark() later.
  landmark_index_ = landmark_index;
  is_matched_ = true;
  return true;
}

// Get the view that this observation was seen from.
std::shared_ptr<View> Observation::GetView() const {
  if (view_index_ == kInvalidView) {
    LOG(WARNING) << "View index is invalid. Returning a null pointer.";
    return View::Ptr();
  }

  View::Ptr view = View::GetView(view_index_);
  CHECK_NOTNULL(view.get());
  return view;
}

// Get the landmark that this observation corresponds to. Returns a null
// pointer if the observation has not been matched with a landmark.
Landmark::Ptr Observation::GetLandmark() const {
  if (landmark_index_ == kInvalidLandmark) {
    LOG(WARNING) << "Landmark index is invalid. Returning a null pointer.";
    return Landmark::Ptr();
  }

  Landmark::Ptr landmark = Landmark::GetLandmark(landmark_index_);
  CHECK_NOTNULL(landmark.get());
  return landmark;
}

// Returns whether or not the observation has been matched with a landmark. If
// this returns false, 'GetLandmark()' will return a null pointer.
bool Observation::IsMatched() const {
  return is_matched_;
}

// Get this observation's feature.
Feature::Ptr Observation::Feature() {
  CHECK_NOTNULL(feature_ptr_.get());
  return feature_ptr_;
}

// Get the descriptor corresponding to this observation's feature.
std::shared_ptr<::bsfm::Descriptor> Observation::Descriptor() {
  CHECK_NOTNULL(descriptor_ptr_.get());
  return descriptor_ptr_;
}

}  //\namespace bsfm
