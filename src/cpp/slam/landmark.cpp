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

#include "landmark.h"

#include <numeric>

#include "../geometry/rotation.h"
#include "../sfm/view.h"

namespace bsfm {

namespace {
const double kMinTriangulationAngle = D2R(2.0);
}  //\namespace

// Declaration of static member variables.
std::unordered_map<LandmarkIndex, Landmark::Ptr> Landmark::landmark_registry_;
LandmarkIndex Landmark::current_landmark_index_ = 0;
unsigned int Landmark::required_observations_ = 2;

// Factory method. Registers the landmark and newly created index in the
// landmark registry so that they can be accessed from the static GetLandmark()
// method. This guarantees that all landmarks will have unique indices.
Landmark::Ptr Landmark::Create() {
  // Create a new Landmark, implicitly assigning a unique index.
  Landmark::Ptr landmark(new Landmark());
  landmark_registry_.insert(std::make_pair(landmark->Index(), landmark));
  return landmark;
}

// Gets the landmark corresponding to the input index. If the landmark has not
// been created yet, this method returns a null pointer.
Landmark::Ptr Landmark::GetLandmark(LandmarkIndex landmark_index) {
  auto registry_element = landmark_registry_.find(landmark_index);
  if (registry_element == landmark_registry_.end()) {
    LOG(WARNING)
        << "Landmark does not exist in registry. Returning a null pointer.";
    return Landmark::Ptr();
  }

  return registry_element->second;
}

// Returns the total number of existing landmarks.
LandmarkIndex Landmark::NumExistingLandmarks() {
  return landmark_registry_.size();
}

// Returns a vector of all existing landmark indices.
std::vector<LandmarkIndex> Landmark::ExistingLandmarkIndices() {
  std::vector<LandmarkIndex> indices(NumExistingLandmarks());
  std::iota(indices.begin(), indices.end(), 0);
  return indices;
}

// Returns whether the landmark index corresponds to a landmark that has been
// created.
bool Landmark::IsValidLandmark(LandmarkIndex landmark_index) {
  return landmark_index <= current_landmark_index_;
}

// Resets all landmarks and clears the landmark registry. This should rarely be
// called, except when completely resetting the program or reconstruction.
void Landmark::ResetLandmarks() {
  current_landmark_index_ = 0;
  landmark_registry_.clear();
}

// Deletes the most recently created landmark, such that the next landmark that
// is created will have the deleted landmark's index. This has the potential to
// cause issues if the caller holds onto a pointer to the deleted landmark.
void Landmark::DeleteMostRecentLandmark() {
  current_landmark_index_--;

  Landmark::Ptr most_recent_landmark =
      Landmark::GetLandmark(current_landmark_index_);

  CHECK_NOTNULL(most_recent_landmark.get());
  for (auto& observation : most_recent_landmark->Observations())
    observation->RemoveLandmarkAssociation();

  landmark_registry_.erase(current_landmark_index_);
}

// Returns the unique index of this landmark.
LandmarkIndex Landmark::Index() const {
  return landmark_index_;
}

// Set the landmark's position.
void Landmark::SetPosition(const Point3D& position) {
  position_ = position;
}

// Set the landmark's descriptor.
void Landmark::SetDescriptor(const ::bsfm::Descriptor& descriptor) {
  descriptor_ = descriptor;
}

// Remove all existing observations of the landmark.
void Landmark::ClearObservations() {
  observations_.clear();
}

// Get position.
const Point3D& Landmark::Position() const {
  return position_;
}

// Get descriptor.
const ::bsfm::Descriptor& Landmark::Descriptor() const {
  return descriptor_;
}

// Get observations.
std::vector<Observation::Ptr>& Landmark::Observations() {
  return observations_;
}

const std::vector<Observation::Ptr>& Landmark::Observations() const {
  return observations_;
}

bool Landmark::IsEstimated() const {
  return is_estimated_;
}

// Removes the observation originating from the provided view.
void Landmark::RemoveObservationFromView(ViewIndex view_index) {
  View::Ptr view = View::GetView(view_index);
  CHECK_NOTNULL(view.get());

  for (int ii = observations_.size() - 1; ii >= 0; --ii) {
    CHECK_NOTNULL(observations_[ii].get());
    if (observations_[ii]->GetViewIndex() == view_index) {
      observations_.erase(observations_.begin() + ii);
    }
  }
}

// Returns a raw pointer to the data elements of the position of the landamark.
// This is useful for optimization on landmark positions (e.g. during bundle
// adjustment).
double* Landmark::PositionData() {
  return position_.Get().data();
}

// Add a new observation of the landmark. The landmark's position will be
// retriangulated from all observations of it.
bool Landmark::IncorporateObservation(const Observation::Ptr& observation) {
  CHECK_NOTNULL(observation.get());

  // Does the landmark descriptor match with the observation's descriptor?
  if (!observations_.empty()) {
    DistanceMetric& distance = DistanceMetric::Instance();
    if (distance.Max() != std::numeric_limits<double>::max()) {
      std::vector<::bsfm::Descriptor> descriptors;
      descriptors.push_back(descriptor_);
      descriptors.push_back(observation->Descriptor());
      distance.MaybeNormalizeDescriptors(descriptors);

      if (distance(descriptors[0], descriptors[1]) > distance.Max()) {
        VLOG(1) << "Observation was not matched to landmark "
            << this->Index();
        return false;
      }
    }
  }

  // If we don't have enough observations to triangulate the landmark yet,
  // continually store them until we do.
  if (observations_.size() < RequiredObservations() - 1) {
    observation->SetIncorporatedLandmark(this->Index());
    observations_.push_back(observation);
    descriptor_ = observation->Descriptor();
    return true;
  }

  // Triangulate the landmark's putative position if we were to incorporate the
  // new observation.
  std::vector<Camera> cameras;
  std::vector<Feature> features;
  for (const auto& obs : observations_) {
    cameras.push_back(obs->GetView()->Camera());
    features.push_back(obs->Feature());
  }
  cameras.push_back(observation->GetView()->Camera());
  features.push_back(observation->Feature());

  // If triangulation fails, we don't have a match and won't update position.
  double uncertainty = 0.0;
  Point3D new_position;
  if (!Triangulate(features, cameras, new_position, uncertainty)) {
    return false;
  }

  // Make sure we aren't triangulating something collinear with our position.
  if (1.0 / uncertainty < kMinTriangulationAngle) {
    // This observation is still fine, it's just collinear. We can store it, but
    // shouldn't update our position estimate.
    observation->SetIncorporatedLandmark(this->Index());
    observations_.push_back(observation);
    descriptor_ = observation->Descriptor();

    return false;
  }

  // We got a position, so this landmark is now estimated!
  is_estimated_ = true;
  position_ = new_position;

  // Successfully triangulated the landmark. Update its position and store this
  // observation. Also tell the observation that it has been matched with us.
  observation->SetIncorporatedLandmark(this->Index());
  observations_.push_back(observation);
  descriptor_ = observation->Descriptor();

  return is_estimated_;
}

// Return the first view to observe this landmark.
View::Ptr Landmark::SourceView() const {
  if (observations_.empty()) {
    LOG(WARNING) << "Landmark has not been associated with any observations, "
                    "so source view is undefined. Returning null pointer.";
    return View::Ptr();
  }

  return observations_.front()->GetView();
}

// Given a set of views, return whether or not this landmark has been seen by at
// least N of them.
bool Landmark::SeenByAtLeastNViews(const std::vector<ViewIndex>& view_indices,
                                   unsigned int N) {
  unsigned int count = 0;
  for (const auto& view_index : view_indices) {
    // If they gave us a bad input, ignore it and continue.
    if (!View::IsValidView(view_index))
      continue;

    // Make sure this view knows all landmarks its observations have seen.
    View::Ptr view = View::GetView(view_index);
    view->UpdateObservedLandmarks();
    if (view->HasObservedLandmark(this->Index())) {
      count++;
    }

    if (count == N) {
      return true;
    }
  }

  return false;
}

// Return the minimum number of observations necessary to triangulate teh
// landmark.
unsigned int Landmark::RequiredObservations() {
  return required_observations_;
}

// Set the minimum number of observations necessary to triangulate the landmark.
void Landmark::SetRequiredObservations(unsigned int required_observations) {
  required_observations_ = required_observations;
}

// Private constructor enforces creation via factory method. This will be called
// from the factory method.
Landmark::Landmark()
    : position_(Point3D(0.0, 0.0, 0.0)),
      landmark_index_(NextLandmarkIndex()),
      is_estimated_(false) {}

// Static method for determining the next index across all Landmarks constructed
// so far. This is called in the Landmark constructor.
LandmarkIndex Landmark::NextLandmarkIndex() {
  return current_landmark_index_++;
}

}  //\namespace bsfm
