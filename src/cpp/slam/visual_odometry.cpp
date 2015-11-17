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

#include <string>

#include "visual_odometry.h"

#include "../camera/camera_extrinsics.h"
#include "../geometry/essential_matrix_solver.h"
#include "../geometry/reprojection_error.h"
#include "../geometry/triangulation.h"
#include "../matching/distance_metric.h"
#include "../matching/feature.h"
#include "../matching/pairwise_image_match.h"
#include "../matching/naive_matcher_2d2d.h"
#include "../matching/naive_matcher_2d3d.h"
#include "../ransac/fundamental_matrix_ransac_problem.h"
#include "../ransac/pnp_ransac_problem.h"
#include "../ransac/ransac.h"
#include "../ransac/ransac_options.h"
#include "../slam/landmark.h"
#include "../slam/observation.h"
#include "../sfm/bundle_adjuster.h"

namespace bsfm {

VisualOdometry::VisualOdometry(const VisualOdometryOptions& options,
                               const Camera& camera)
    : has_first_view_(false), options_(options) {
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

  // Update the annotator's image.
  if (options_.draw_features ||
      options_.draw_landmarks ||
      options_.draw_inlier_observations ||
      options_.draw_tracks) {
    annotator_.SetImage(image);
  }

  // Handle the first two images separately for initializiation.
  if (!has_first_view_) {
    return InitializeFirstView(image);
  } else if (view_indices_.size() == 1) {
    return InitializeSecondView(image);
  }

  Status s = ProcessImage(image);
  if (!s.ok())
    return s;

#if 0
  // Bundle adjust views in the sliding window.
  BundleAdjuster bundle_adjuster;
  if (!bundle_adjuster.Solve(options_.bundle_adjustment_options,
                             SlidingWindowViewIndices()))
    return Status::Cancelled("Failed to perform bundle adjustment.");
#endif

  return Status::Ok();
}

void VisualOdometry::GetAnnotatedImage(Image* image) const {
  annotator_.GetImageCopy(image);
}

const std::vector<ViewIndex>& VisualOdometry::ViewIndices() const {
  return view_indices_;
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
  has_first_view_ = true;

  first_view->CreateAndAddObservations(features, descriptors);

  // Annotate features only in the first frame.
  if (options_.draw_features) {
    annotator_.AnnotateFeatures(features);
    annotator_.Draw();
  }

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

  // Use all matches for 2D to 2D feature matching (not just the best n).
  const bool kTempOption = options_.matcher_options.only_keep_best_matches;
  options_.matcher_options.only_keep_best_matches = false;

  PairwiseImageMatchList image_matches;
  if (!feature_matcher.MatchImages(options_.matcher_options, image_matches)) {
    options_.matcher_options.only_keep_best_matches = kTempOption;
    return Status::Cancelled("Failed to match first and second images.");
  }
  options_.matcher_options.only_keep_best_matches = kTempOption;

  CHECK(image_matches.size() == 1);
  PairwiseImageMatch image_match = image_matches[0];
  FeatureMatchList feature_matches = image_match.feature_matches_;

  // Check for sufficient matches.
  if (feature_matches.size() < 8) {
    return Status::Cancelled(
        "Not enough matched points to compute a fundamental matrix.");
  }

  // Get the fundamental matrix between the first two cameras using RANSAC.
  FundamentalMatrixRansacProblem f_problem;
  f_problem.SetData(feature_matches);

  Ransac<FeatureMatch, FundamentalMatrixRansacModel> f_solver;
  f_solver.SetOptions(options_.fundamental_matrix_ransac_options);
  f_solver.Run(f_problem);

  if (!f_problem.SolutionFound()) {
    return Status::Cancelled(
        "Failed to compute a fundamental matrix with RANSAC.");
  }
  Matrix3d F = f_problem.Model().F_;

  // Get the essential matrix between the first two cameras.
  EssentialMatrixSolver e_solver;
  Matrix3d E = e_solver.ComputeEssentialMatrix(F, intrinsics_, intrinsics_);

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

  // Use all RANSAC inlier features to triangulate an initial set of landmarks.
 const FeatureMatchList ransac_inliers = f_problem.Inliers();
 std::vector<LandmarkIndex> new_landmark_indices;
 for (size_t ii = 0; ii < ransac_inliers.size(); ++ii) {
   Landmark::Ptr landmark = Landmark::Create();
   new_landmark_indices.push_back(landmark->Index());

   // Find the observation corresponding to this match in each view. Add those
   // 2 observations to the landmark.
   const Feature& feature1 = ransac_inliers[ii].feature1_;
   const Feature& feature2 = ransac_inliers[ii].feature2_;

   for (const auto& observation1 : first_view->Observations()) {
     if (feature1 == observation1->Feature()) {
       landmark->IncorporateObservation(observation1);
       break;
     }
   }

   for (const auto& observation2 : second_view->Observations()) {
     if (feature2 == observation2->Feature()) {
       landmark->IncorporateObservation(observation2);
       break;
     }
   }
 }

  // Annotate features and landmarks in the second frame.
  if (options_.draw_features)
    annotator_.AnnotateFeatures(features2);
  if (options_.draw_landmarks)
    annotator_.AnnotateLandmarks(new_landmark_indices, second_view->Camera());
  if (options_.draw_features || options_.draw_landmarks)
    annotator_.Draw();

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

  // Create a new view with identity camera extrinsics.
  Camera new_camera(CameraExtrinsics(), intrinsics_);
  View::Ptr new_view = View::Create(new_camera);

  // Add features and descriptors as observations to the view.
  new_view->CreateAndAddObservations(features, descriptors);

  // Get a list of all landmarks seen by the views in the sliding window.
  std::vector<LandmarkIndex> landmark_indices;
  View::GetSlidingWindowLandmarks(options_.sliding_window_length,
                                  &landmark_indices);

  // Match features seen by this view with existing landmarks.
  NaiveMatcher2D3D feature_matcher;
  if (!feature_matcher.Match(options_.matcher_options,
                             new_view->Index(),
                             landmark_indices)) {
    View::DeleteMostRecentView();
    return Status::Cancelled(
        "Failed to match 2D descriptors with existing landmarks.");
  }

  // Use PnP RANSAC to find the pose of this camera using the 2D<-->3D matches.
  PnPRansacProblem pnp_problem;
  pnp_problem.SetIntrinsics(intrinsics_);

  std::vector<Observation::Ptr> matched_observations;
  new_view->MatchedObservations(&matched_observations);
  pnp_problem.SetData(matched_observations);

  Ransac<Observation::Ptr, PnPRansacModel> pnp_solver;
  pnp_solver.SetOptions(options_.pnp_ransac_options);
  pnp_solver.Run(pnp_problem);

  // If RANSAC fails, delete the view and return.
  if (!pnp_problem.SolutionFound()) {
    View::DeleteMostRecentView();
    return Status::Cancelled(
        "Failed to compute new camera pose with PnP RANSAC.");
  }

  // Get the camera pose from RANSAC.
  const CameraExtrinsics& computed_extrinsics =
      pnp_problem.Model().camera_.Extrinsics();
  new_view->MutableCamera().SetExtrinsics(computed_extrinsics);

  // Loop over RANSAC inliers and incorporate observations into the landmarks
  // they point to.
  for (auto& observation : pnp_problem.Inliers()) {
    Landmark::Ptr landmark = observation->GetLandmark();
    CHECK_NOTNULL(landmark.get());

    landmark->IncorporateObservation(observation);

    // Update the landmark's descriptor using this new observation. This makes
    // it easier to find the landmark as we change angles, etc.
    landmark->SetDescriptor(observation->Descriptor());
  }

  // Go through all unincorporated observations and try to match them against
  // previous images in the sliding window to generate new landmarks.
  InitializeNewLandmarks(new_view);

  // Add the new view to the trajectory.
  view_indices_.push_back(new_view->Index());

  // Annotate everything in the N'th frame.
  std::vector<LandmarkIndex> new_view_landmarks;
  if (options_.draw_landmarks || options_.draw_tracks) {
    View::GetSlidingWindowLandmarks(1, &new_view_landmarks);
    landmark_indices.insert(landmark_indices.end(),
                            new_view_landmarks.begin(),
                            new_view_landmarks.end());
  }

  if (options_.draw_features)
    annotator_.AnnotateFeatures(features);
  if (options_.draw_landmarks)
    annotator_.AnnotateLandmarks(landmark_indices, new_view->Camera());
  if (options_.draw_inlier_observations)
    annotator_.AnnotateObservations(new_view->Index(), new_view->Observations());
  if (options_.draw_tracks)
    annotator_.AnnotateTracks(new_view_landmarks, SlidingWindowViewIndices());
  if (options_.draw_features || options_.draw_landmarks ||
      options_.draw_inlier_observations || options_.draw_tracks) {
    annotator_.Draw();
  }

  return Status::Ok();
}

void VisualOdometry::InitializeNewLandmarks(const View::Ptr& new_view) {
  CHECK_NOTNULL(new_view.get());

  // Get a list of all features and descriptors that have not been incorporated
  // into landmarks.
  std::vector<Feature> unused_features1;
  std::vector<Descriptor> unused_descriptors1;
  GetUnusedFeatures(new_view->Index(), &unused_features1, &unused_descriptors1);

  // Use all matches for 2D to 2D feature matching (not just the best n).
  const bool kTempOption = options_.matcher_options.only_keep_best_matches;
  options_.matcher_options.only_keep_best_matches = false;

  // Loop over other images in the sliding window. Try to match against their
  // unused features and triangulate new landmarks.
  const Camera new_camera = new_view->Camera();
  for (const auto& view_index : SlidingWindowViewIndices()) {
    std::vector<Feature> unused_features2;
    std::vector<Descriptor> unused_descriptors2;
    GetUnusedFeatures(view_index, &unused_features2, &unused_descriptors2);

    // Match features.
    NaiveMatcher2D2D feature_matcher;
    feature_matcher.AddImageFeatures(unused_features1, unused_descriptors1);
    feature_matcher.AddImageFeatures(unused_features2, unused_descriptors2);

    PairwiseImageMatchList image_matches;
    if (!feature_matcher.MatchImages(options_.matcher_options, image_matches))
      continue;

    CHECK(image_matches.size() == 1);
    PairwiseImageMatch image_match = image_matches[0];
    FeatureMatchList feature_matches = image_match.feature_matches_;

    // Attempt to triangulate matches between these two images. Evaluate the
    // reprojection error of the resulting points.
    View::Ptr old_view = View::GetView(view_index);
    CHECK_NOTNULL(old_view.get());

    const Camera old_camera = old_view->Camera();
    for (const auto& feature_match : feature_matches) {
      // Triangulate the match.
      Point3D point;
      if (!Triangulate(feature_match, new_camera, old_camera, point)) continue;

      // Check reprojection error of the match.
      const Feature& feature1 = feature_match.feature1_;
      const Feature& feature2 = feature_match.feature2_;
      const double max_error = options_.pnp_ransac_options.acceptable_error;
      if (ReprojectionError(feature1, point, new_camera) > max_error) {
        continue;
      }
      if (ReprojectionError(feature2, point, old_camera) > max_error) {
        continue;
      }

      // Initialize a new landmark.
      Landmark::Ptr landmark = Landmark::Create();

      // Find the observation corresponding to this match in each view. Add
      // those 2 observations to the landmark.
      for (const auto& new_observation : new_view->Observations()) {
        if (feature1 == new_observation->Feature()) {
          landmark->IncorporateObservation(new_observation);
          break;
        }
      }

      for (const auto& old_observation : old_view->Observations()) {
        if (feature2 == old_observation->Feature()) {
          landmark->IncorporateObservation(old_observation);
          break;
        }
      }
    }
  }

  // Put back matcher options for next round of 2D<-->3D matching.
  options_.matcher_options.only_keep_best_matches = kTempOption;
}

Status VisualOdometry::GetFeaturesAndDescriptors(
    const Image& image, std::vector<Feature>* features,
    std::vector<Descriptor>* descriptors) {
  CHECK_NOTNULL(features)->clear();
  CHECK_NOTNULL(descriptors)->clear();

  // Detect keypoints in the image.
  KeypointList keypoints;
  if (!keypoint_detector_.DetectKeypoints(image, keypoints)) {
    return Status::Cancelled("Failed to detect keypoints.");
  }

  // Get descriptors for each keypoint.
  if (!descriptor_extractor_.DescribeFeatures(image, keypoints, *features,
                                              *descriptors)) {
    return Status::Cancelled("Failed to describe features.");
  }

  return Status::Ok();
}

void VisualOdometry::GetUnusedFeatures(ViewIndex view_index,
                                       std::vector<Feature>* features,
                                       std::vector<Descriptor>* descriptors) {
  CHECK_NOTNULL(features)->clear();
  CHECK_NOTNULL(descriptors)->clear();

  View::Ptr view = View::GetView(view_index);
  CHECK_NOTNULL(view.get());

  for (auto& observation : view->Observations()) {
    if (!observation->IsIncorporated()) {
      features->emplace_back(observation->Feature());
      descriptors->emplace_back(observation->Descriptor());
    }
  }
}

// Return view indices in the sliding window.
std::vector<ViewIndex> VisualOdometry::SlidingWindowViewIndices() {
  std::vector<ViewIndex> sw_view_indices;
  int start = std::max(static_cast<int>(view_indices_.size()) -
                       static_cast<int>(options_.sliding_window_length), 0);

  for (size_t ii = start; ii < view_indices_.size(); ++ii) {
    sw_view_indices.push_back(view_indices_[ii]);
  }

  return sw_view_indices;
}

}  //\namespace bsfm
