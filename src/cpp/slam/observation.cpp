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

// Factory method.
Observation::Ptr Observation::Create(const std::shared_ptr<View>& view_ptr,
                                     const ::bsfm::Feature& feature,
                                     const ::bsfm::Descriptor& descriptor) {
  Observation::Ptr observation(new Observation(view_ptr, feature, descriptor));
  view_ptr->AddObservation(observation);
  return observation;
}

Observation::~Observation() {}

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

// Sets the landmark as a potential match for this observation. The observation
// still has not been incorporated into the landmark.
void Observation::SetMatchedLandmark(LandmarkIndex landmark_index) {
  CHECK_NE(kInvalidLandmark, landmark_index);
  landmark_index_ = landmark_index;
  is_matched_ = true;
}

// Returns whether or not the observation has been matched with a landmark. If
// this returns false, 'GetLandmark()' will return a null pointer.
bool Observation::IsMatched() const {
  return is_matched_;
}

// Associates a landmark with this observation. This is called by
// Landmark::IncorporateObservation().
void Observation::SetIncorporatedLandmark(LandmarkIndex landmark_index) {
  CHECK_NE(kInvalidLandmark, landmark_index);
  landmark_index_ = landmark_index;
  is_matched_ = true;
  is_incorporated_ = true;
}

// Returns whether or not the observation has been incorporated into a landmark.
bool Observation::IsIncorporated() const {
  return is_incorporated_;
}

// Get this observation's feature.
const ::bsfm::Feature& Observation::Feature() const {
  return feature_;
}

// Get the descriptor corresponding to this observation's feature.
const ::bsfm::Descriptor& Observation::Descriptor() const {
  return descriptor_;
}

// Constructor is private to enforce usage of factory method. Initialize an
// observation with the view that it came from, an image-space feature
// coordinate pair, and an associated descriptor. Implicitly initializes the
// landmark to be invalid, since the observation has not been matched with a 3D
// landmark yet.
Observation::Observation(const View::Ptr& view_ptr,
                         const ::bsfm::Feature& feature,
                         const ::bsfm::Descriptor& descriptor)
    : landmark_index_(kInvalidLandmark),
      is_matched_(false),
      is_incorporated_(false),
      feature_(feature),
      descriptor_(descriptor) {
  // Store the view's index to access it later.
  CHECK_NOTNULL(view_ptr.get());
  view_index_ = view_ptr->Index();
}

}  //\namespace bsfm
