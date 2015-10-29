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

#include "view.h"
#include "../slam/landmark.h"
#include "../slam/observation.h"

namespace bsfm {

// Declaration of static member variable.
std::unordered_map<ViewIndex, View::Ptr> View::view_registry_;
ViewIndex View::current_view_index_ = 0;

// Factory method. Registers the view and index in the view registry so
// that they can be accessed from the static GetView() method. This guarantees
// that all views will have uniqueindices.
View::Ptr View::Create(const ::bsfm::Camera& camera) {
  // Create a new View, implicitly assigning a unique index.
  View::Ptr view(new View(camera));
  view_registry_.insert(std::make_pair(view->Index(), view));
  return view;
}

// Gets the view corresponding to the input view index. If the view has not been
// created yet, this method returns a null pointer.
View::Ptr View::GetView(ViewIndex view_index) {
  auto registry_element = view_registry_.find(view_index);
  if (registry_element == view_registry_.end()) {
    LOG(WARNING)
        << "View does not exist in registry. Returning a null pointer.";
    return View::Ptr();
  }

  return registry_element->second;
}

// Returns the total number of existing views.
ViewIndex View::NumExistingViews() {
  return view_registry_.size();
}

// Returns whether the view index corresponds to a view that has been created.
bool View::IsValidView(ViewIndex view_index) {
  return view_index <= current_view_index_;
}

// Resets all views and clears the view registry. This should rarely be called,
// except when completely resetting the program or reconstruction.
void View::ResetViews() {
  current_view_index_ = 0;
  view_registry_.clear();
}

void View::SetCamera(const ::bsfm::Camera& camera) {
  camera_ = camera;
}

::bsfm::Camera& View::MutableCamera() {
  return camera_;
}

const ::bsfm::Camera& View::Camera() const {
  return camera_;
}

ViewIndex View::Index() const {
  return view_index_;
}

void View::AddObservation(const Observation::Ptr& observation) {
  CHECK_NOTNULL(observation.get());
  observations_.push_back(observation);

  // If the observation is matched to a landmark, add that landmark to our
  // registry.
  if (observation->IsMatched()) {
    landmarks_.insert(observation->GetLandmark()->Index());
  }
}

void View::AddObservations(const std::vector<Observation::Ptr>& observations) {
  for (const auto& observation : observations) {
    AddObservation(observation);
  }
}

const std::vector<Observation::Ptr>& View::Observations() const {
  return observations_;
}

bool View::HasObservedLandmark(LandmarkIndex landmark_index) const {
  return landmarks_.count(landmark_index);
}

void View::UpdateObservedLandmarks() {
  for (const auto& observation : observations_) {
    CHECK_NOTNULL(observation.get());
    if (!observation->IsMatched()) continue;

    // Calls to std::set::insert() will overwrite old key value pairs.
    landmarks_.insert(observation->GetLandmark()->Index());
  }
}

bool View::SortByIndex(const View::Ptr& lhs, const View::Ptr& rhs) {
  return lhs->Index()< rhs->Index();
}

// Hidden constructor. This will be called from the factory method.
View::View(const ::bsfm::Camera& camera)
    : view_index_(NextViewIndex()), camera_(camera) {}

// Static method for determining the next index across all Views
// constructed so far. This is called in the View constructor.
ViewIndex View::NextViewIndex() {
  return current_view_index_++;
}

}  //\namespace bsfm
