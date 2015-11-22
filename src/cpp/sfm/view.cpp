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

// Deletes the most recently created view, such that the next view that is
// created will have the deleted view's index. This has the potential to cause
// issues if the caller holds onto a pointer to the deleted view.
void View::DeleteMostRecentView() {
  current_view_index_--;

  // Remove any observations from landmarks that might point back to this view.
  View::Ptr most_recent_view = View::GetView(current_view_index_);
  CHECK_NOTNULL(most_recent_view.get());
  most_recent_view->UpdateObservedLandmarks();
  for (auto& landmark_index : most_recent_view->ObservedLandmarks()) {
    Landmark::Ptr landmark = Landmark::GetLandmark(landmark_index);
    CHECK_NOTNULL(landmark.get());
    landmark->RemoveObservationFromView(current_view_index_);
  }

  view_registry_.erase(current_view_index_);
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
}

const std::vector<Observation::Ptr>& View::Observations() const {
  return observations_;
}

void View::MatchedObservations(
    std::vector<Observation::Ptr>* matched_observations) const {
  CHECK_NOTNULL(matched_observations)->clear();

  for (const auto& observation : observations_) {
    CHECK_NOTNULL(observation.get());
    if (observation->IsMatched()) {
      matched_observations->push_back(observation);
    }
  }
}

void View::IncorporatedObservations(
    std::vector<Observation::Ptr>* incorporated_observations) const {
  CHECK_NOTNULL(incorporated_observations)->clear();

  for (const auto& observation : observations_) {
    CHECK_NOTNULL(observation.get());
    if (observation->IsIncorporated()) {
      incorporated_observations->push_back(observation);
    }
  }
}

bool View::CreateAndAddObservations(
    const std::vector<Feature>& features,
    const std::vector<Descriptor>& descriptors) {
  if (features.size() != descriptors.size()) {
    LOG(WARNING) << "Number of features and descriptors does not match.";
    return false;
  }

  // Get a shared pointer to this view.
  View::Ptr this_view = GetView(this->Index());

  // Add observations to this view.
  for (size_t ii = 0; ii < features.size(); ++ii)
    Observation::Create(this_view, features[ii], descriptors[ii]);

  return true;
}

void View::GetFeaturesAndDescriptors(
    std::vector<Feature>* features,
    std::vector<Descriptor>* descriptors) const {
  CHECK_NOTNULL(features);
  CHECK_NOTNULL(descriptors);
  features->clear();
  descriptors->clear();

  // Iterate over observations and return a list of features and descriptors
  // owned by them.
  for (const auto& observation : observations_) {
    features->emplace_back(observation->Feature());
    descriptors->emplace_back(observation->Descriptor());
  }
}

bool View::HasObservedLandmark(LandmarkIndex landmark_index) const {
  return landmarks_.count(landmark_index);
}

bool View::CanSeeLandmark(LandmarkIndex landmark_index) const {
  const Landmark::Ptr landmark = Landmark::GetLandmark(landmark_index);
  CHECK_NOTNULL(landmark.get());

  const Point3D point = landmark->Position();
  double u = 0.0, v = 0.0;  // unused
  return camera_.WorldToImage(point.X(), point.Y(), point.Z(), &u, &v);
}

void View::UpdateObservedLandmarks() {
  for (const auto& observation : observations_) {
    CHECK_NOTNULL(observation.get());
    if (!observation->IsIncorporated()) continue;

    // Calls to std::set::insert() will overwrite old key value pairs.
    landmarks_.insert(observation->GetLandmark()->Index());
  }
}

void View::GetSlidingWindowLandmarks(
    unsigned int sliding_window_length,
    std::vector<LandmarkIndex>* landmark_indices) {
  CHECK_NOTNULL(landmark_indices)->clear();

  // Create a list of view indices in the sliding window.
  std::vector<ViewIndex> view_indices;
  ViewIndex end_view = current_view_index_;

  ViewIndex start_view = 0;
  if (sliding_window_length <= end_view) {
    start_view = end_view - sliding_window_length;
  }

  for (ViewIndex ii = start_view; ii < end_view; ++ii) {
    view_indices.push_back(ii);
  }

  // Get a set of all landmarks seen by all of these views.
  std::unordered_set<LandmarkIndex> landmark_set;
  for (const auto& view_index : view_indices) {
    View::Ptr view = View::GetView(view_index);
    CHECK_NOTNULL(view.get());

    view->UpdateObservedLandmarks();
    landmark_set.insert(view->ObservedLandmarks().begin(),
                        view->ObservedLandmarks().end());
  }

  // Copy the set of unique landmark indices into the output vector.
  landmark_indices->reserve(landmark_set.size());
  landmark_indices->assign(landmark_set.begin(), landmark_set.end());
}

const std::unordered_set<LandmarkIndex>& View::ObservedLandmarks() const {
  return landmarks_;
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
