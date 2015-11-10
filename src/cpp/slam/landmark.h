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
#include <unordered_map>
#include <utility>
#include <vector>

#include "observation.h"
#include "../geometry/point_3d.h"
#include "../geometry/triangulation.h"
#include "../util/types.h"

namespace bsfm {

class View;

class Landmark {
 public:
  typedef std::shared_ptr<Landmark> Ptr;
  typedef std::shared_ptr<const Landmark> ConstPtr;

  // Factory method. Registers the landmark and newly created index in the
  // landmark registry so that they can be accessed from the static
  // GetLandmark() method. This guarantees that all landmarks will have unique
  // indices.
  static Landmark::Ptr Create();
  ~Landmark() {}

  // Gets this landmark's index.
  LandmarkIndex Index() const;

  // Gets the landmark corresponding to the input index. If the landmark has not
  // been created yet, this method returns a null pointer.
  static Landmark::Ptr GetLandmark(LandmarkIndex landmark_index);

  // Returns the number of existing landmarks.
  static LandmarkIndex NumExistingLandmarks();

  // Returns a vector of all existing landmark indices.
  static std::vector<LandmarkIndex> ExistingLandmarkIndices();

  // Returns whether the landmark index corresponds to a landmark that has been
  // created.
  static bool IsValidLandmark(LandmarkIndex landmark_index);

  // Resets all landmarks and clears the landmark registry. If somebody else is
  // holding onto a shared pointer to a landmark, that landmark will still be
  // valid and may now have an index that conflicts with landmarks that are
  // subsequently added to the landmark registry. Therefore this function can
  // cause some chaos if not used properly. This should rarley need to be
  // called, except when completely resetting the program or reconstruction.
  static void ResetLandmarks();

  // Setters.
  void SetPosition(const Point3D& position);
  void SetDescriptor(const ::bsfm::Descriptor& descriptor);
  void ClearObservations();

  // Accessors.
  const Point3D& Position() const;
  const ::bsfm::Descriptor& Descriptor() const;
  const std::vector<Observation::Ptr>& Observations() const;

  // Returns a raw pointer to the data elements of the position of the landmark.
  // This is useful for optimization on landmark positions (e.g. during bundle
  // adjustment.)
  double* PositionData();

  // Add a new observation of the landmark. If 'retriangulate' is true, the
  // landmark's position will be retriangulated from all observations of it.
  // This will return false if the observation's descriptor does not match with
  // our own descriptor, or if we fail to retriangulate the landmark after
  // incorporating the new observation.
  bool IncorporateObservation(const Observation::Ptr& observation,
                              bool retriangulate = true);

  // Get the view that first saw this landmark.
  std::shared_ptr<View> SourceView() const;

  // Given a set of views, return whether or not this landmark has been seen by
  // at least N of them.
  bool SeenByAtLeastNViews(const std::vector<ViewIndex>& view_indices, unsigned int N);

 private:
  DISALLOW_COPY_AND_ASSIGN(Landmark)

  // Private constructor to enforce creation via factory method.
  Landmark();

  // Static method for determining the next index across all Landmarks
  // constructed so far. This is called in the Landmark constructor.
  static LandmarkIndex NextLandmarkIndex();

  // The landmark's 3D position.
  Point3D position_;

  // An index which uniquely defines this landmark.
  LandmarkIndex landmark_index_;

  // A registry of all landmarks constructed so far. These can be queried with
  // the static method GetLandmark();
  static std::unordered_map<LandmarkIndex, Landmark::Ptr> landmark_registry_;

  // Observations that were triangulated to find the 3D position of this
  // landmark.
  std::vector<Observation::Ptr> observations_;

  // The descriptor associated with this 3D point. This is assigned based on the
  // first observation added to the landmark.
  ::bsfm::Descriptor descriptor_;

  // The maximum index assigned to any landmark created so far.
  static LandmarkIndex current_landmark_index_;
};  //\class Landmark

}  //\namespace bsfm

#endif
