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

#include "pnp_ransac_problem.h"
#include <geometry/pose_estimator_2d3d.h>
#include <camera/camera_extrinsics.h>
#include <camera/camera_intrinsics.h>
#include <camera/camera.h>

#include <iostream>

namespace bsfm {

// ------------ PnPRansacModel methods ------------ //

// Default constructor. Initialize to empty matches and default camera.
// Note: this should really never be used -- instead use the constructor below.
PnPRansacModel::PnPRansacModel()
  : camera_(Camera()),
    matches_(std::vector<Observation::Ptr>()),
    error_(0.0) {}

// Constructor. Set parameters as given.
// Use this constructor instead of the default.
PnPRansacModel::PnPRansacModel(const Camera& camera,
			       const std::vector<Observation::Ptr>& matches)
  : camera_(camera),
    matches_(matches),
    error_(0.0) {}

// Destructor.
PnPRansacModel::~PnPRansacModel() {}

// Return model error.
double PnPRansacModel::Error() const {
  return error_;
}

// Evaluate model on a single data element and update error.
bool PnPRansacModel::IsGoodFit(const Observation::Ptr& observation,
			       double error_tolerance) {
  // Get error.
  double error = PnPRansacModel::EvaluateReprojectionError(observation);

  // Did not project into the image.
  if (error < 0.0)
    return false;

  // Check tolerance.
  if (error <= error_tolerance)
    return true;
  return false;
}

// Compute reprojection error of this landmark. Return infinity if point
// does not project into the image.
double PnPRansacModel::EvaluateReprojectionError(
   const Observation::Ptr& observation) const {
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
    camera_.WorldToImage(point.X(), point.Y(), point.Z(), &u, &v);

  // Check that the landmark projects into the image.
  if (!in_camera)
    return std::numeric_limits<double>::infinity();

  // Compute error and return.
  double delta_u = u - feature.u_;
  double delta_v = v - feature.v_;
  double error = delta_u*delta_u + delta_v*delta_v;

  return error;
}

// ------------ PnPRansacProblem methods ------------ //

// Default constructor/destructor.
PnPRansacProblem::PnPRansacProblem() {}
PnPRansacProblem::~PnPRansacProblem() {}

// Set camera intrinsics.
void PnPRansacProblem::SetIntrinsics(CameraIntrinsics& intrinsics) {
  intrinsics_ = intrinsics;
}

// Subsample the data.
std::vector<Observation::Ptr> PnPRansacProblem::SampleData(
   unsigned int num_samples) {

  // Randomly shuffle the entire dataset and take the first elements.
  std::random_shuffle(data_.begin(), data_.end());

  // Make sure we don't over step.
  if (static_cast<size_t>(num_samples) > data_.size()) {
    VLOG(1) << "Requested more RANSAC data samples than are available. "
	    << "Returning all data.";
    num_samples = data_.size();
  }

  // Get samples.
  std::vector<Observation::Ptr> samples(
    data_.begin(), data_.begin() + static_cast<size_t>(num_samples));

  return samples;
}

// Return the data that was not sampled.
std::vector<Observation::Ptr> PnPRansacProblem::RemainingData(
    unsigned int num_sampled_previously) const {
  // In Sample(), the data was shuffled and we took the first
  // 'num_sampled_previously' elements. Here, take the remaining elements.
  if (num_sampled_previously >= data_.size()) {
    VLOG(1) << "No remaining RANSAC data to sample.";
    return std::vector<Observation::Ptr>();
  }

  return std::vector<Observation::Ptr>(
    data_.begin() + num_sampled_previously, data_.end());
}

// Fit a model to the provided data using PoseEstimatorPnP.
PnPRansacModel PnPRansacProblem::FitModel(
    const std::vector<Observation::Ptr>& input_data) const {

  // Extract a FeatureList and a Point3DList from input_data.
  FeatureList points_2d;
  Point3DList points_3d;
  points_2d.reserve(input_data.size());
  points_3d.reserve(input_data.size());

  for (const auto& observation : input_data) {
    CHECK_NOTNULL(observation.get());

    // Extract feature and append.
    points_2d.push_back(observation->Feature());

    // Extract landmark position and append.
    Landmark::Ptr landmark = observation->GetLandmark();
    CHECK_NOTNULL(landmark.get());
    points_3d.push_back(landmark->Position());
  }

  // Set up solver.
  PoseEstimator2D3D solver;
  solver.Initialize(points_2d, points_3d, intrinsics_);

  // Solve.
  Pose calculated_pose;
  if (!solver.Solve(calculated_pose)) {
    VLOG(1) << "Could not estimate a pose using the PnP solver. "
	    << "Assuming identity pose.";
  }

  // Generate a model from this Pose.
  CameraExtrinsics extrinsics(calculated_pose);
  Camera camera(extrinsics, intrinsics_);

  PnPRansacModel model(camera, input_data);
  return model;
}

} //\namespace bsfm
