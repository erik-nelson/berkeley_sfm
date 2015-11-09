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

#include "visual_odometry.h"

#include "../camera/camera_extrinsics.h"
#include "../geometry/essential_matrix_solver.h"
#include "../geometry/eight_point_algorithm_solver.h"
#include "../matching/distance_metric.h"
#include "../matching/feature.h"
#include "../matching/pairwise_image_match.h"
#include "../matching/naive_matcher_2d2d.h"
#include "../slam/landmark.h"
#include "../slam/observation.h"
#include "../sfm/view.h"

namespace bsfm {

VisualOdometry::VisualOdometry(const VisualOdometryOptions& options,
                               const Camera& camera)
    : options_(options) {
  // Use input options to specify member variable settings.
  keypoint_detector_.SetDetector(options_.feature_type);
  if (options_.adaptive_features) {
    keypoint_detector_.SetAdaptiveOn(options_.adaptive_min,
                                     options_.adaptive_max,
                                     options_.adaptive_iters);
  }

  descriptor_extractor_.SetDescriptor(options_.descriptor_type);

  DistanceMetric& distance = DistanceMetric::Instance();
  distance.SetMetric(options_.matcher_options.distance_metric);
  if (options_.matcher_options.enforce_maximum_descriptor_distance) {
    distance.SetMaximumDistance(
        options_.matcher_options.maximum_descriptor_distance);
  }

  // Create an initial view using the provided camera. Store camera intrinsics.
  View::Ptr view = View::Create(camera);
  view_indices_.push_back(view->Index());
  intrinsics_ = camera.Intrinsics();
}

VisualOdometry::~VisualOdometry() {}

Status VisualOdometry::Update(const Image& image) {

  // Handle the first two images separately for initializiation.
  if (view_indices_.empty()) {
    return InitializeFirstView(image);
  } else if (view_indices_.size() == 1) {
    return InitializeSecondView(image);
  }

  return ProcessImage(image);
}

Status VisualOdometry::InitializeFirstView(const Image& image) {
  // Extract features and descriptors.
  std::vector<Feature> features;
  std::vector<Descriptor> descriptors;
  Status status = GetFeaturesAndDescriptors(image, &features, &descriptors);
  if (!status.ok()) return status;

  // We created the first view in the constructor. Add observations to it.
  View::Ptr first_view = View::GetView(view_indices_.front());
  CHECK_NOTNULL(first_view.get());

  first_view->CreateAndAddObservations(features, descriptors);
  return Status::Ok();
}

Status VisualOdometry::InitializeSecondView(const Image& image) {
  // Extract features and descriptors for the second view.
  std::vector<Feature> features2;
  std::vector<Descriptor> descriptors2;
  Status status = GetFeaturesAndDescriptors(image, &features2, &descriptors2);
  if (!status.ok()) return status;

  // Try to match features and descriptors from the first image against those
  // from this image.
  View::Ptr first_view = View::GetView(view_indices_.front());
  CHECK_NOTNULL(first_view.get());

  std::vector<Feature> features1;
  std::vector<Descriptor> descriptors1;
  first_view->GetFeaturesAndDescriptors(&features1, &descriptors1);

  NaiveMatcher2D2D feature_matcher;
  feature_matcher.AddImageFeatures(features1, descriptors1);
  feature_matcher.AddImageFeatures(features2, descriptors2);

  PairwiseImageMatchList image_matches;
  if (!feature_matcher.MatchImages(options_.matcher_options, image_matches)) {
    return Status::Cancelled("Failed to match first and second images.");
  }
  CHECK(image_matches.size() == 1);
  PairwiseImageMatch image_match = image_matches[0];
  FeatureMatchList feature_matches = image_match.feature_matches_;

  // Check for sufficient matches.
  if (feature_matches.size() < 8) {
    return Status::Cancelled(
        "Not enough matched points to compute a fundamental matrix.");
  }

  // Get the fundamental matrix between the first two cameras.
  Matrix3d F;
  EightPointAlgorithmSolver f_solver;
  f_solver.SetOptions(options_.f_solver_options);
  if (!f_solver.ComputeFundamentalMatrix(feature_matches, F)) {
    return Status::Cancelled("Failed to compute fundamental matrix.");
  }

  // Get the essential matrix between the first two cameras.
  Matrix3d E;
  EssentialMatrixSolver e_solver;
  E = e_solver.ComputeEssentialMatrix(E, intrinsics_, intrinsics_);

  // Extract a relative pose from the essential matrix.
  Pose relative_pose;
  if (!e_solver.ComputeExtrinsics(E,
                                  feature_matches,
                                  intrinsics_,
                                  intrinsics_,
                                  relative_pose)) {
    return Status::Cancelled("Failed to decompose essential matrix.");
  }

  // If we got to here, we can initialize a second view.
  const Pose initial_pose = first_view->Camera().Extrinsics().WorldToCamera();
  CameraExtrinsics extrinsics(relative_pose * initial_pose);

  Camera camera2;
  camera2.SetExtrinsics(extrinsics);
  camera2.SetIntrinsics(intrinsics_);
  View::Ptr second_view = View::Create(camera2);
  view_indices_.push_back(second_view->Index());

  second_view->CreateAndAddObservations(features2, descriptors2);

  // Use all matched features to triangulate an initial set of landmarks.
  for (size_t ii = 0; ii < feature_matches.size(); ++ii) {
    Landmark::Ptr landmark = Landmark::Create();

    // Find the observation corresponding to this match in each view. Add those
    // 2 observations to the landmark.
    Feature feature1 = feature_matches[ii].feature1_;
    for (const auto& observation1 : first_view->Observations()) {
      if (feature1 == observation1->Feature()) {
        landmark->IncorporateObservation(observation1);
        break;
      }
    }

    Feature feature2 = feature_matches[ii].feature2_;
    for (const auto& observation2 : second_view->Observations()) {
      if (feature2 == observation2->Feature()) {
        landmark->IncorporateObservation(observation2);
        break;
      }
    }
  }

  // We now have 2 views, a bunch of triangulated landmarks, and observations of
  // those landmarks in each view!
  return Status::Ok();
}

Status VisualOdometry::ProcessImage(const Image& image) {
  // Extract features and descriptors.
  std::vector<Feature> features;
  std::vector<Descriptor> descriptors;
  Status status = GetFeaturesAndDescriptors(image, &features, &descriptors);
  if (!status.ok()) return status;

  return Status::Ok();
}

Status VisualOdometry::GetFeaturesAndDescriptors(
    const Image& image, std::vector<Feature>* features,
    std::vector<Descriptor>* descriptors) {
  CHECK_NOTNULL(features);
  CHECK_NOTNULL(descriptors);
  features->clear();
  descriptors->clear();

  // Detect keypoints in the image.
  KeypointList keypoints;
  if (!keypoint_detector_.DetectKeypoints(image, keypoints)) {
    return Status::Cancelled("Failed to detect keypoints.");
  }

  // Get descriptors for each keypoint.
  if (!descriptor_extractor_.DescribeFeatures(image,
                                              keypoints,
                                              *features,
                                              *descriptors)) {
    return Status::Cancelled("Failed to describe features.");
  }

  return Status::Ok();
}

}  //\namespace bsfm
