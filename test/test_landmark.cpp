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

#include <gflags/gflags.h>
#include <matching/feature.h>
#include <sfm/view.h>
#include <slam/landmark.h>
#include <util/types.h>

#include <gtest/gtest.h>

namespace bsfm {

// Since all tests are run in a single process, landmarks may have been
// created to this in other tests (since they are statically stored in a
// registry until program termination). Therefore we need to begin with the
// maximum view number in each test.

TEST(Landmark, TestUniqueLandmarkIndices) {
  LandmarkIndex start_index = Landmark::NumExistingLandmarks();

  // Make sure frame indices are assigned on construction.
  for (LandmarkIndex ii = start_index; ii < start_index+3; ++ii) {
    Landmark::Ptr landmark = Landmark::Create();
    CHECK_EQ(ii, landmark->Index());

    // Also make sure the static getter works.
    CHECK_EQ(ii, Landmark::GetLandmark(ii)->Index());
  }
}

TEST(Landmark, TestPersistentLandmarks) {
  // We already created 3 landmarks that should be stored in the static landmark
  // registry in a previous test, even after leaving that test's scope. Check
  // that landmarks persist until program exit.
  for (LandmarkIndex ii = 0; ii < 3; ++ii) {
    CHECK_EQ(ii, Landmark::GetLandmark(ii)->Index());
  }

  LandmarkIndex start_index = Landmark::NumExistingLandmarks();
  EXPECT_NE(0, start_index);
  for (LandmarkIndex ii = start_index; ii < start_index + 3; ++ii) {
    Landmark::Ptr landmark = Landmark::Create();
    CHECK_EQ(ii, landmark->Index());
  }

  // Check if we can reset landmarks.
  Landmark::ResetLandmarks();
  CHECK_EQ(0, Landmark::NumExistingLandmarks());
  for (LandmarkIndex ii = 0; ii < 3; ++ii) {
    Landmark::Ptr landmark = Landmark::Create();
    CHECK_EQ(ii, landmark->Index());
  }

  // Reset so the next test doesn't have issues.
  Landmark::ResetLandmarks();
}

TEST(Landmark, TestSeenByAtLeastTwoViews) {
  // Clean up from any other tests that may have initialized landmarks or views.
  Landmark::ResetLandmarks();
  View::ResetViews();

  // The landmark should not have been seen by at least 2 views when there are
  // no views.
  Landmark::Ptr landmark = Landmark::Create();
  std::vector<ViewIndex> view_indices;
  EXPECT_FALSE(landmark->SeenByAtLeastNViews(view_indices, 2));

  // Create two views with no observations.
  View::Ptr view1 = View::Create(Camera());
  View::Ptr view2 = View::Create(Camera());
  view_indices.push_back(view1->Index());
  view_indices.push_back(view2->Index());

  // The landmark still should not have been observed, since neither view
  // contains observations.
  EXPECT_FALSE(landmark->SeenByAtLeastNViews(view_indices, 2));

  // Add some observations to the views.
  Feature feature(0.0, 0.0);
  Descriptor descriptor(Descriptor::Zero(64));
  Observation::Ptr observation1 =
      Observation::Create(view1, feature, descriptor);
  Observation::Ptr observation2 =
      Observation::Create(view2, feature, descriptor);

  // The landmark still should not have been observed, since neither view
  // contains observations that have specifically been matched with this
  // landmark.
  EXPECT_FALSE(landmark->SeenByAtLeastNViews(view_indices, 2));

  // Match the observations in the views with the landmark. After adding both,
  // the landmark should report that it has been seen by both views. Since
  // observations don't have valid positions, we won't be able to triangulate,
  // so just ask the landmark not to retriangulate its 3D position.
  EXPECT_TRUE(landmark->IncorporateObservation(observation1));
  EXPECT_FALSE(landmark->SeenByAtLeastNViews(view_indices, 2));
  EXPECT_TRUE(landmark->IncorporateObservation(observation2));
  EXPECT_TRUE(landmark->SeenByAtLeastNViews(view_indices, 2));

  // Finally, make sure that incorporating additional views that observe the
  // landmark doesn't alter behavior.
  for (int ii = 0; ii < 20; ++ii) {
    View::Ptr view = View::Create(Camera());
    Observation::Ptr observation =
        Observation::Create(view, feature, descriptor);

    view_indices.push_back(view->Index());
    EXPECT_TRUE(landmark->IncorporateObservation(observation));
    EXPECT_TRUE(landmark->SeenByAtLeastNViews(view_indices, 2));
  }

  // Clean up.
  Landmark::ResetLandmarks();
  View::ResetViews();
}

}  //\namespace bsfm
