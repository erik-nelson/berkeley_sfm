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
#include <slam/landmark.h>

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
  for (LandmarkIndex ii = start_index; ii < start_index+3; ++ii) {
    Landmark::Ptr landmark = Landmark::Create();
    CHECK_EQ(ii, landmark->Index());
  }
}

}  //\namespace bsfm
