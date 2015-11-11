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

///////////////////////////////////////////////////////////////////////////////
//
// This file defines methods that can be used to compute the reprojection error
// from multiple keypoint matches.
//
///////////////////////////////////////////////////////////////////////////////

#include "reprojection_error.h"

#include "point_3d.h"
#include "../matching/feature.h"
#include "../slam/landmark.h"

namespace bsfm {

// Evaluate the reprojection error on the given Observation.
double ReprojectionError(const Observation::Ptr& observation,
                         const Camera& camera) {
  CHECK_NOTNULL(observation.get());

  // Unpack this observation (extract Feature and Landmark).
  Feature feature = observation->Feature();
  Landmark::Ptr landmark = observation->GetLandmark();
  CHECK_NOTNULL(landmark.get());

  // Extract position of this landmark.
  Point3D point = landmark->Position();

  // Project into this camera.
  double u = 0.0, v = 0.0;
  const bool in_camera =
      camera.WorldToImage(point.X(), point.Y(), point.Z(), &u, &v);

  // Check that the landmark projects into the image.
  if (!in_camera) return std::numeric_limits<double>::infinity();

  // Compute error and return.
  double delta_u = u - feature.u_;
  double delta_v = v - feature.v_;
  double error = delta_u * delta_u + delta_v * delta_v;

  return error;
}

// Repeats the ReprojectionError() function on a list of obserations, returning
// the sum of squared errors.
double ReprojectionError(const std::vector<Observation::Ptr>& observations,
                         const Camera& camera) {
  double total_error = 0.0;
  for (const auto& observation : observations)
    total_error += ReprojectionError(observation, camera);

  return total_error;
}

}  //\namespace bsfm
