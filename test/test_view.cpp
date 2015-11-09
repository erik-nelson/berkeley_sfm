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
 * Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 *          Erik Nelson            ( eanelson@eecs.berkeley.edu )
 */

#include <algorithm>

#include <gflags/gflags.h>
#include <camera/camera.h>
#include <camera/camera_intrinsics.h>
#include <geometry/rotation.h>
#include <matching/feature.h>
#include <slam/landmark.h>
#include <slam/observation.h>
#include <sfm/view.h>
#include <util/types.h>

#include <gtest/gtest.h>

namespace bsfm {

namespace {
const int kImageWidth = 1920;
const int kImageHeight = 1080;
const double kVerticalFov = D2R(90.0);

CameraIntrinsics DefaultIntrinsics() {
  CameraIntrinsics intrinsics;
  intrinsics.SetImageLeft(0);
  intrinsics.SetImageTop(0);
  intrinsics.SetImageWidth(kImageWidth);
  intrinsics.SetImageHeight(kImageHeight);
  intrinsics.SetVerticalFOV(kVerticalFov);
  intrinsics.SetFU(intrinsics.f_v());
  intrinsics.SetCU(0.5 * kImageWidth);
  intrinsics.SetCV(0.5 * kImageHeight);

  return intrinsics;
}
}  //\namespace

TEST(View, TestUniqueViewIndices) {
  // Since all tests are run in a single process, views may have been created
  // prior to this in other tests (since they are statically stored in a
  // registry until program termination). We need to begin with the maximum view
  // number.
  ViewIndex start_index = View::NumExistingViews();

  // Make sure frame indices are assigned on construction.
  Camera camera;
  for (ViewIndex ii = start_index; ii < start_index+3; ++ii) {
    View::Ptr view = View::Create(camera);
    CHECK_EQ(ii, view->Index());

    // Also make sure the static getter works.
    CHECK_EQ(ii, View::GetView(ii)->Index());
  }

  // Sort views by frame index.
  std::vector<View::Ptr> views_earlier;
  for (ViewIndex ii = 0; ii < 10; ++ii) {
    views_earlier.push_back(View::Create(camera));
  }

  std::vector<View::Ptr> views_later;
  for (ViewIndex ii = 0; ii < 10; ++ii) {
    views_later.push_back(View::Create(camera));
  }

  std::vector<View::Ptr> views;
  views.insert(views.end(), views_later.begin(), views_later.end());
  views.insert(views.end(), views_earlier.begin(), views_earlier.end());
  std::sort(views.begin(), views.end(), View::SortByIndex);

  // Are the views in the right order after sorting?
  for (ViewIndex ii = 0; ii < views.size() - 1; ++ii) {
    CHECK_LE(views[ii]->Index(), views[ii+1]->Index());

    // Make sure the static getter also works here.
    CHECK_LE(View::GetView(ii)->Index(),
             View::GetView(ii + 1)->Index());
  }
}

TEST(View, TestPersistentViews) {
  // We already created some views that should be stored in the static view
  // registry in a previous test, even after leaving that test's scope. Check
  // that views are persistent until program exit.
  for (ViewIndex ii = 0; ii < 3; ++ii) {
    CHECK_EQ(ii, View::GetView(ii)->Index());
  }

  ViewIndex start_index = View::NumExistingViews();
  EXPECT_NE(0, start_index);
  for (ViewIndex ii = start_index; ii < start_index + 3; ++ii) {
    View::Ptr view = View::Create(Camera());
    CHECK_EQ(ii, view->Index());
  }

  // Check if we can reset views.
  View::ResetViews();
  CHECK_EQ(0, View::NumExistingViews());
  for (ViewIndex ii = 0; ii < 3; ++ii) {
    View::Ptr view = View::Create(Camera());
    CHECK_EQ(ii, view->Index());
  }

  // Reset so the next test doesn't have issues.
  View::ResetViews();
}

TEST(View, TestCanSeeLandmark) {
  // Create a landmark in front of the camera (camera faces world +Z by
  // default).
  Landmark::Ptr landmark = Landmark::Create();
  landmark->SetPosition(Point3D(0.0, 0.0, 10.0));

  View::Ptr view = View::Create(Camera());
  view->MutableCamera().SetIntrinsics(DefaultIntrinsics());

  // Make sure the view can see the landmark, but has not yet observed it.
  EXPECT_TRUE(view->CanSeeLandmark(landmark->Index()));
  EXPECT_FALSE(view->HasObservedLandmark(landmark->Index()));

  // Force the view to observe the landmark.
  const Feature feature(kImageWidth / 2, kImageHeight / 2);
  Observation::Ptr observation =
      Observation::Create(view, feature, Descriptor::Random(64));
  EXPECT_TRUE(landmark->IncorporateObservation(observation));
  view->UpdateObservedLandmarks();

  // Make sure the view has now observed the landmark
  EXPECT_TRUE(view->CanSeeLandmark(landmark->Index()));
  EXPECT_TRUE(view->HasObservedLandmark(landmark->Index()));
  EXPECT_TRUE(observation->IsMatched());
  EXPECT_TRUE(observation->IsIncorporated());
}

}  //\namespace bsfm
