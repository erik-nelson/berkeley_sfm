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

#include <camera/camera.h>
#include <geometry/point_3d.h>
#include <matching/feature.h>
#include <math/random_generator.h>
#include <sfm/view.h>
#include <slam/observation.h>
#include <slam/landmark.h>
#include <util/types.h>

#include <gflags/gflags.h>
#include <gtest/gtest.h>

namespace bsfm {

TEST(Observation, TestGetters) {
  // Make sure that we get null pointers when asking for bad views or landmarks,
  // and get the correct view and landmark if they are initialized properly.
  Feature::Ptr feature(new Feature(0.0, 0.0));
  std::shared_ptr<Descriptor> descriptor(new Descriptor(Descriptor::Zero(64)));

  // Disable console printouts since we expect to get null pointer warnings.
  // Also write a notification into the test log.
  FLAGS_logtostderr = false;
  LOG(INFO) << "Expect to see some null pointer warnings below.";

  // It is impossible to initialize with an invalid view due to nullity checks
  // in the observation constructor, so we can begin by checking valid views
  // with invalid landmarks.
  Camera camera;
  View::Ptr view = View::Create(camera);
  Observation::Ptr observation(new Observation(view, feature, descriptor));
  EXPECT_EQ(view, observation->GetView());
  EXPECT_EQ(nullptr, observation->GetLandmark());
  EXPECT_FALSE(observation->IsMatched());

  // Set a valid landmark.
  Landmark::Ptr landmark = Landmark::Create();
  landmark->SetDescriptor(descriptor);

  // Incorporate the observation with the landmark.
  EXPECT_TRUE(landmark->IncorporateObservation(observation));
  EXPECT_EQ(view, observation->GetView());
  EXPECT_EQ(landmark, observation->GetLandmark());
  EXPECT_TRUE(observation->IsMatched());

  // Turn back on print outs
  FLAGS_logtostderr = true;
}

TEST(Observation, TestTriangulateObservations) {
  // Create a 3D point, and lots of observations of it from different cameras.
  // Incorporate all of the observations into a landmark to re-triangulate the
  // 3D point's position.
  math::RandomGenerator rng(0);

  for (int ii = 0; ii < 100; ++ii) {
    const double x = rng.DoubleUniform(-2.0, 2.0);
    const double y = rng.DoubleUniform(-2.0, 2.0);
    const double z = rng.DoubleUniform(-2.0, 2.0);
  }
}

}  //\namespace bsfm
