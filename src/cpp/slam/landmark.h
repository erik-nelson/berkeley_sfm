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
// The Landmark class defines a 3D point and associated descriptor that can be
// used for 2D-3D matching. Each landmark also has a list of observations that
// have been associated with it over time.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_SLAM_LANDMARK_H
#define BSFM_SLAM_LANDMARK_H

#include <Eigen/Core>
#include <memory>
#include <vector>

#include "observation.h"
#include "../geometry/point_3d.h"

namespace bsfm {

template <typename Descriptor>
class Landmark {
 public:
  Landmark();
  ~Landmark() { }

  // Setters.
  void SetDescriptor(const Descriptor& descriptor);
  void SetDescriptor(const std::shared_ptr<Descriptor>& descriptor_ptr);
  void ClearObservations();

  // Accessors.
  const Point3D& Position() const;
  const std::vector<Observation::Ptr>& Observations() const;
  const std::shared_ptr<Descriptor>& Descriptor() const;

  // Adding a new observation will update the estimated position by
  // re-triangulating the feature.
  bool IncorporateObservation(const Observation::Ptr& observation);

 private:
  Point3D position_;
  std::vector<Observation::Ptr> observations_;
  std::shared_ptr<Descriptor> descriptor_ptr_;

};  //\class Landmark


// -------------------- Implementation -------------------- //

// Constructore initializes position and covariance to zero and identity.
template <typename Descriptor>
Landmark<Descriptor>::Landmark()
    : position_(Point3D(0.0, 0.0, 0.0)),
      covariance_(Covariance3D::Identity()) {}

// Create a new descriptor pointer, assuming the templated type has a copy ctor.
template <typename Descriptor>
void Landmark<Descriptor>::SetDescriptor(const Descriptor& descriptor) {
  descriptor_ptr_.reset(new Descriptor(descriptor));
}

// Copy an existing descriptor pointer.
template <typename Descriptor>
void Landmark<Descriptor>::SetDescriptor(
    const std::shared_ptr<Descriptor>& descriptor_ptr) {
  descriptor_ptr_ = descriptor_ptr;
}

// Remove all existing observations of the landmark.
template <typename Descriptor>
void Landmark<Descriptor>::ClearObservations() {
  observations_.clear();
}

// Get position.
template <typename Descriptor>
const Landmark<Descriptor>::Point3D& Position() const {
  return position_;
}

// Get covariance.
template <typename Descriptor>
const Landmark<Descriptor>::Covariance3D& Covariance() const {
  return covariance_;
}

// Get observations.
template <typename Descriptor>
const std::vector<Observation::Ptr>& Landmark<Descriptor>::Observations()
    const {
  return observations_;
}

// Get descriptor.
template <typename Descriptor>
const std::shared_ptr<Descriptor>& Landmark<Descriptor>::Descriptor() const {
  return descriptor_;
}

// Adding a new observation will update the estimated position by
// re-triangulating the feature.
template <typename Descriptor>
bool Landmark<Descriptor>::IncorporateObservation(
    const Observation::Ptr& observation) {
  CHECK(observation != nullptr);

  // Triangulate the landmark's position after incorporating the new observation.
  std::vector<Camera> cameras;
  std::vector<Feature> features;
  for (const auto& old_observation : observations_) {
    cameras.push_back(old_observation->camera_);
    features.push_back(old_observation->feature_);
  }
  cameras.push_back(observation->camera_);
  features.push_back(observation->feature_);

  Point3D new_position;
  if (!Triangulate(features, cameras, new_position)) {
    // Don't update the landmark's position.
    return false;
  }

  // Successfully triangulated the landmark. Update its position and store this
  // observation.
  position_ = new_position;
  observations_.push_back(observation);
  return true;
}

}  //\namespace bsfm

#endif
