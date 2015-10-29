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
// This View class models a camera at a specific frame index. Views must be
// created with the Create() factory method (construction is disabled). On
// creation, a view will be given a unique index which cannot be changed. The
// view and index will be registered so that views can be accessed from the
// static function GetView().
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_SFM_VIEW_H
#define BSFM_SFM_VIEW_H

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "../camera/camera.h"
#include "../slam/landmark.h"
#include "../slam/observation.h"
#include "../util/disallow_copy_and_assign.h"
#include "../util/types.h"

namespace bsfm {

class View {
 public:
  typedef std::shared_ptr<View> Ptr;
  typedef std::shared_ptr<const View> ConstPtr;

  // Factory method. Registers the view and newly created index in the view
  // registry so that they can be accessed from the static GetView() method.
  // This guarantees that all views will have unique indices.
  static View::Ptr Create(const ::bsfm::Camera& camera);
  ~View() {}

  // Gets the view corresponding to the input index. If the view has not
  // been created yet, this method returns a null pointer.
  static View::Ptr GetView(ViewIndex view_index);

  // Returns the total number of existing landmarks.
  static ViewIndex NumExistingViews();

  // Resets all views and clears the view registry. If somebody else is holding
  // onto a shared pointer to a view, that view will still be valid and may now
  // have an index that conflicts with views that are subsequently added to the
  // view registry. Therefore this function can cause some chaos if not used
  // properly. This should rarely need to be called, except when completely
  // resetting the program or reconstruction.
  static void ResetViews();

  // Get and set the camera.
  void SetCamera(const ::bsfm::Camera& camera);
  ::bsfm::Camera& MutableCamera();
  const ::bsfm::Camera& Camera() const;

  // Get this view's index.
  ViewIndex Index() const;

  // Add observations to this view.
  void AddObservation(const Observation::Ptr& observation);
  void AddObservations(const std::vector<Observation::Ptr>& observations);

  // Get observations.
  const std::vector<Observation::Ptr>& Observations() const;

  // Update the landmark registry by looping over all observations and seeing
  // which landmarks they have observed.
  void UpdateObservedLandmarks();

  // For sorting a list of views by their indices.
  static bool SortByIndex(const View::Ptr& lhs, const View::Ptr& rhs);

 private:
  DISALLOW_COPY_AND_ASSIGN(View)

  // Private constructor to enfore creation via factory method.
  View(const ::bsfm::Camera& camera);

  // Static method for determining the next index across all Views
  // constructed so far. This is called in the View constructor.
  static ViewIndex NextViewIndex();

  // Includes intrinsics and extrinsics.
  ::bsfm::Camera camera_;

  // An index which uniquely defines this view.
  ViewIndex view_index_;

  // A registry of all views constructed so far. These can be queried with the
  // static method GetView().
  static std::unordered_map<ViewIndex, View::Ptr> view_registry_;

  // A list of all 2D features and landmarks that they correspond to, as seen by
  // this view.
  std::vector<Observation::Ptr> observations_;

  // A registry of all landmarks associated with this view. These are determined
  // by iterating over observations and getting each observations' corresponding
  // landmark, if it has been matched.
  std::unordered_set<LandmarkIndex> landmarks_;

  // The maximum index assigned to any view created so far.
  static ViewIndex current_view_index_;
};  //\class View

}  //\namespace bsfm

#endif
