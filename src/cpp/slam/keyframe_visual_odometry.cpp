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

#include <set>
#include <string>
#include <unordered_set>

#include "keyframe_visual_odometry.h"
#include "../file/csv_writer.h"
#include "../geometry/essential_matrix_solver.h"
#include "../matching/feature_match.h"
#include "../matching/naive_matcher_2d2d.h"
#include "../matching/naive_matcher_2d3d.h"
#include "../matching/pairwise_image_match.h"
#include "../ransac/fundamental_matrix_ransac_problem.h"
#include "../ransac/pnp_ransac_problem.h"
#include "../ransac/ransac.h"
#include "../ransac/ransac_options.h"
#include "../sfm/bundle_adjuster.h"
#include "../util/progress_bar.h"

namespace bsfm {

KeyframeVisualOdometry::KeyframeVisualOdometry(
    const VisualOdometryOptions& options, const CameraIntrinsics& intrinsics)
    : initialize_new_keyframe_(false),
      options_(options),
      current_keyframe_(kInvalidView) {
  // Use input options to specify member variable settings.
  keypoint_detector_.SetDetector(options_.feature_type);
  if (options_.use_grid_filter) {
    keypoint_detector_.SetGridOn(options_.grid_rows, options_.grid_cols,
                                 options_.grid_min_num_features);
  }

  descriptor_extractor_.SetDescriptor(options_.descriptor_type);

  DistanceMetric& distance = DistanceMetric::Instance();
  distance.SetMetric(options_.matcher_options.distance_metric);
  if (options_.matcher_options.enforce_maximum_descriptor_distance) {
    distance.SetMaximumDistance(
        options_.matcher_options.maximum_descriptor_distance);
  }

  // Store camera intrinsics.
  intrinsics_ = intrinsics;
}

KeyframeVisualOdometry::~KeyframeVisualOdometry() {}

Status KeyframeVisualOdometry::Update(const Image& image) {
  // Extract keypoints from the frame.
  std::vector<Keypoint> keypoints;
  Status status = GetKeypoints(image, &keypoints);
  if (!status.ok()) return status;

  // Extract features and descriptors from the keypoints.
  std::vector<Feature> features;
  std::vector<Descriptor> descriptors;
  status = GetDescriptors(image, &keypoints, &features, &descriptors);
  if (!status.ok()) return status;

  // Set the annotator's image.
  if (options_.draw_tracks || options_.draw_features) {
    annotator_.SetImage(image);
  }

  // Initialize the very first view if we don't have one yet.
  if (current_keyframe_ == kInvalidView) {
    Landmark::SetRequiredObservations(2);
    InitializeFirstView(features, descriptors);
    return Status::Ok();
  }

  // Initialize the second view if we don't have one yet using 2D<-->2D
  // matching.
  if (view_indices_.size() == 1) {
    status = InitializeSecondView(features, descriptors);
    if (status.ok()) {
      Landmark::SetRequiredObservations(options_.num_observations_to_triangulate);
    }
    return status;
  }

  // Create a camera with an unknown pose.
  Camera camera;
  camera.SetIntrinsics(intrinsics_);
  View::Ptr new_view = View::Create(camera);

  // Is this new image going to be a keyframe?
  const bool is_keyframe =
      initialize_new_keyframe_ ||
      NumEstimatedTracks() < options_.min_num_feature_tracks;
  if (is_keyframe) {
    initialize_new_keyframe_ = false;
  }

  // Update feature tracks and add matched features to the view.
  status = UpdateFeatureTracks(features,
                               descriptors,
                               new_view->Index(),
                               is_keyframe);
  if (!status.ok()) {
    View::DeleteMostRecentView();
    return status;
  }

  // Compute the new camera pose.
  status = EstimatePose(new_view->Index());
  if (!status.ok()) {
    View::DeleteMostRecentView();
  } else {
    view_indices_.push_back(new_view->Index());
  }

  if (is_keyframe && options_.perform_bundle_adjustment) {
    // Bundle adjust views in the sliding window.
    BundleAdjuster bundle_adjuster;
    if (!bundle_adjuster.Solve(options_.bundle_adjustment_options,
                               SlidingWindowViewIndices()))
      return Status::Cancelled("Failed to perform bundle adjustment.");
  }

  // Annotate tracks and features.
  if (options_.draw_features) {
    annotator_.AnnotateFeatures(features);
  }
  if (options_.draw_tracks) {
    annotator_.AnnotateTracks(tracks_);
  }
  if (options_.draw_tracks || options_.draw_features) {
    annotator_.Draw();
  }

  return status;
}

void KeyframeVisualOdometry::GetAnnotatedImage(Image* image) const {
  annotator_.GetImageCopy(image);
}

const std::vector<ViewIndex>& KeyframeVisualOdometry::ViewIndices() const {
  return view_indices_;
}

Status KeyframeVisualOdometry::WriteTrajectoryToFile(
    const std::string& filename) const {
  file::CsvWriter csv_writer(filename);
  if (!csv_writer.IsOpen())
    return Status::FailedPrecondition("Invalid filename.");

  // Iterate over all views, storing camera poses to the file. Output format
  // on each line will be:
  // view index, t.x, t.y, t.z, r.x, r.y, r.z
  // where rotations are expressed in axis-angle format.
  util::ProgressBar progress("Writing trajectory", view_indices_.size());
  for (size_t ii = 0; ii < view_indices_.size(); ++ii) {
    progress.Update(ii);

    const View::Ptr view = View::GetView(view_indices_[ii]);
    CHECK_NOTNULL(view.get());
    const Vector3d t = view->Camera().Translation();
    const Vector3d r = view->Camera().AxisAngleRotation();

    std::vector<double> camera_info = {static_cast<double>(view_indices_[ii])};
    camera_info.push_back(t(0));
    camera_info.push_back(t(1));
    camera_info.push_back(t(2));
    camera_info.push_back(r(0));
    camera_info.push_back(r(1));
    camera_info.push_back(r(2));
    csv_writer.WriteLine(camera_info);
  }
  progress.Update(view_indices_.size());

  if (!csv_writer.Close()) {
    return Status::Unknown("Failed to close csv file.");
  }

  return Status::Ok();
}

Status KeyframeVisualOdometry::WriteMapToFile(
    const std::string& filename) const {
  if (tracks_.empty() && frozen_landmarks_.empty())
    return Status::Cancelled("No tracks or landmarks to write.");

  file::CsvWriter csv_writer(filename);
  if (!csv_writer.IsOpen())
    return Status::FailedPrecondition("Invalid filename.");

  // Iterate over all current tracks and frozen landmarks, storing:
  // p.x, p.y, p.z
  // where p is the 3D point.

  // First get all triangulated features that are currently being tracked.
  util::ProgressBar progress("Writing map",
                             tracks_.size() + frozen_landmarks_.size());
  for (size_t ii = 0; ii < tracks_.size(); ++ii) {
    progress.Update(ii);
    Landmark::Ptr track = Landmark::GetLandmark(tracks_[ii]);
    CHECK_NOTNULL(track.get());

    if (!track->IsEstimated())
      continue;

    std::vector<double> point_info = {track->Position().X(),
                                      track->Position().Y(),
                                      track->Position().Z()};
    csv_writer.WriteLine(point_info);
  }
  progress.Update(tracks_.size());

  // Now repeat for all frozen landmarks.
  for (size_t ii = 0; ii < frozen_landmarks_.size(); ++ii) {
    progress.Update(tracks_.size() + ii);
    Landmark::Ptr landmark = Landmark::GetLandmark(frozen_landmarks_[ii]);
    CHECK_NOTNULL(landmark.get());

    if (!landmark->IsEstimated())
      continue;

    std::vector<double> point_info = {landmark->Position().X(),
                                      landmark->Position().Y(),
                                      landmark->Position().Z()};
    csv_writer.WriteLine(point_info);
  }
  progress.Update(tracks_.size() + frozen_landmarks_.size());

  if (!csv_writer.Close()) {
    return Status::Unknown("Failed to close csv file.");
  }

  return Status::Ok();
}

void KeyframeVisualOdometry::InitializeFirstView(
    const std::vector<Feature>& features,
    const std::vector<Descriptor>& descriptors) {
  CHECK_EQ(features.size(), descriptors.size());

  // Create the first view at identity.
  Camera first_camera;
  first_camera.SetIntrinsics(intrinsics_);
  View::Ptr first_view = View::Create(first_camera);
  first_view->CreateAndAddObservations(features, descriptors);

  // Annotate tracks only in the first frame.
  if (options_.draw_tracks) {
    annotator_.AnnotateFeatures(features);
    annotator_.Draw();
  }

  current_keyframe_ = first_view->Index();
  view_indices_.push_back(first_view->Index());
  return;
}

Status KeyframeVisualOdometry::InitializeSecondView(
    const std::vector<Feature>& features,
    const std::vector<Descriptor>& descriptors) {
  CHECK_EQ(features.size(), descriptors.size());

  // Try to match features and descriptors from the first image against those
  // from this image.
  View::Ptr first_view = View::GetView(current_keyframe_);
  CHECK_NOTNULL(first_view.get());

  std::vector<Feature> old_features;
  std::vector<Descriptor> old_descriptors;
  first_view->GetFeaturesAndDescriptors(&old_features, &old_descriptors);

  NaiveMatcher2D2D feature_matcher;
  feature_matcher.AddImageFeatures(old_features, old_descriptors);
  feature_matcher.AddImageFeatures(features, descriptors);

  // Use all matches for 2D to 2D feature matching (not just the best n).
  const bool kTempOption = options_.matcher_options.only_keep_best_matches;
  options_.matcher_options.only_keep_best_matches = false;

  PairwiseImageMatchList image_matches;
  if (!feature_matcher.MatchImages(options_.matcher_options, image_matches)) {
    options_.matcher_options.only_keep_best_matches = kTempOption;
    return Status::Cancelled("Failed to match first and second images.");
  }
  options_.matcher_options.only_keep_best_matches = kTempOption;

  CHECK_EQ(1, image_matches.size());
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
  if (!e_solver.ComputeExtrinsics(E, feature_matches, intrinsics_, intrinsics_,
                                  relative_pose)) {
    return Status::Cancelled("Failed to decompose essential matrix.");
  }

  // If we got to here, we can initialize a second view.
  CameraExtrinsics extrinsics(relative_pose);

  Camera camera2;
  camera2.SetExtrinsics(extrinsics);
  camera2.SetIntrinsics(intrinsics_);
  View::Ptr second_view = View::Create(camera2);
  second_view->CreateAndAddObservations(features, descriptors);

  // Initialize tracks for all observations in the second view.
  tracks_.clear();
  int triangulated_count = 0;
  for (auto& observation : second_view->Observations()) {
    Landmark::Ptr track = Landmark::Create();

    // If the same feature was seen in the first view, triangulate it.
    // TODO: This is pretty inefficient, even though we only do it once.
    bool found_matched_observation = false;
    for (const auto& feature_match : f_problem.Inliers()) {
      if (feature_match.feature2_ == observation->Feature()) {
        for (const auto& old_observation : first_view->Observations()) {
          if (feature_match.feature1_ == old_observation->Feature()) {
            track->IncorporateObservation(old_observation);
            found_matched_observation = true;
            break;
          }
        }
      }
      if (found_matched_observation) break;
    }

    // Now add the observation from the second view.
    if (track->IncorporateObservation(observation)) {
      if (found_matched_observation) {
        CHECK(track->IsEstimated());
        triangulated_count++;
      }
    }
    track->SetDescriptor(observation->Descriptor());
    tracks_.push_back(track->Index());
  }
  printf("triangulated %d\n", triangulated_count);

  // Annotate tracks and features in the second frame.
  if (options_.draw_features) {
    annotator_.AnnotateFeatures(features);
  }
  if (options_.draw_tracks) {
    annotator_.AnnotateTracks(tracks_);
  }
  if (options_.draw_tracks || options_.draw_features) {
    annotator_.Draw();
  }

  if (triangulated_count < options_.num_landmarks_to_initialize) {
    // Delete all landmarks and return false.
    Landmark::ResetLandmarks();
    tracks_.clear();
    return Status::Cancelled("Did not triangulate enough landmarks to initialize.");
  }

  // We successfully initialized! Store the new view.
  current_keyframe_ = second_view->Index();
  view_indices_.push_back(second_view->Index());
  return Status::Ok();
}

Status KeyframeVisualOdometry::UpdateFeatureTracks(
    const std::vector<Feature>& features,
    const std::vector<Descriptor>& descriptors, ViewIndex view_index,
    bool is_keyframe) {
  CHECK_EQ(features.size(), descriptors.size());

  // Add all features and descriptors to the view as observations.
  View::Ptr view = View::GetView(view_index);
  CHECK_NOTNULL(view.get());
  view->CreateAndAddObservations(features, descriptors);

  // Match observations seen by this view with existing tracks.
  NaiveMatcher2D3D feature_matcher;
  if (!feature_matcher.Match(options_.matcher_options, view_index, tracks_)) {
    return Status::Cancelled(
        "Failed to match 2D descriptors with existing landmarks.");
  }

  // Get a list of tracks that were matched with.
  std::set<LandmarkIndex> matched_tracks;
  for (const auto& observation : view->Observations()) {
    CHECK_NOTNULL(observation.get());
    if (observation->IsMatched()) {
      LandmarkIndex index = observation->GetLandmarkIndex();
      matched_tracks.insert(index);

      Landmark::Ptr track = Landmark::GetLandmark(index);
      CHECK_NOTNULL(track.get());
      track->IncorporateObservation(observation);
      track->SetDescriptor(observation->Descriptor());
    }
  }

  std::vector<Observation::Ptr> test;
  view->MatchedObservations(&test);
  printf("(1) have %lu matches here.\n", test.size());

  // Get a list of tracks that were not matched with.
  std::set<size_t> unmatched_track_inds;
  for (size_t ii = 0; ii < tracks_.size(); ++ii) {
    if (matched_tracks.find(tracks_[ii]) == matched_tracks.end()) {
      unmatched_track_inds.insert(ii);
    }
  }

  // Remove any tracks that were not matched, and have not been seen in at least
  // a few of the last sliding window views.
  std::vector<ViewIndex> sliding_window = SlidingWindowViewIndices();
  std::set<size_t>::reverse_iterator it = unmatched_track_inds.rbegin();
  for (; it != unmatched_track_inds.rend(); ++it) {
    Landmark::Ptr track = Landmark::GetLandmark(tracks_[*it]);
    CHECK_NOTNULL(track.get());
    if (track->SeenByAtLeastNViews(sliding_window, 1))
      continue;

    // If the track was triangulated, store it for visualization (it will no
    // longer be used for pose estimation).
    if (track->IsEstimated())
      frozen_landmarks_.push_back(track->Index());

    tracks_.erase(tracks_.begin() + *it);
  }

  view->MatchedObservations(&test);
  printf("(2) have %lu matches here.\n", test.size());

  if (is_keyframe) {
    printf("Was a keyframe.\n");
    // Add all unmatched features as new tracks.
    current_keyframe_ = view_index;

    for (const auto& observation : view->Observations()) {
      CHECK_NOTNULL(observation.get());

      if (!observation->IsMatched()) {
        Landmark::Ptr track = Landmark::Create();
        track->IncorporateObservation(observation);
        tracks_.push_back(track->Index());
      }
    }
  }

  view->MatchedObservations(&test);
  printf("(3) have %lu matches here.\n", test.size());

  return Status::Ok();
}

Status KeyframeVisualOdometry::EstimatePose(ViewIndex view_index) {
  View::Ptr view = View::GetView(view_index);
  CHECK_NOTNULL(view.get());

  // Use PnP RANSAC to find the pose of this camera using the 2D<-->3D matches.
  PnPRansacProblem pnp_problem;
  pnp_problem.SetIntrinsics(intrinsics_);

  std::vector<Observation::Ptr> matched_observations;
  view->MatchedObservations(&matched_observations);
  printf("(4) at the top, had %lu matches\n", matched_observations.size());

  // Make sure the observations are only of triangulated landmarks.
  std::vector<Observation::Ptr> valid_observations;
  for (const auto& observation : matched_observations) {
    CHECK_NOTNULL(observation.get());
    if (observation->GetLandmark()->IsEstimated())
      valid_observations.push_back(observation);
  }
  pnp_problem.SetData(valid_observations);
  printf("valid observations: %lu\n", valid_observations.size());

  Ransac<Observation::Ptr, PnPRansacModel> pnp_solver;
  pnp_solver.SetOptions(options_.pnp_ransac_options);
  pnp_solver.Run(pnp_problem);

  // If RANSAC fails, set that the next frame should be a keyframe and return.
  if (!pnp_problem.SolutionFound()) {
    initialize_new_keyframe_ = true;
    return Status::Cancelled(
        "Failed to compute new camera pose with PnP RANSAC.");
  }

  // Get the camera pose from RANSAC.
  const CameraExtrinsics& computed_extrinsics =
      pnp_problem.Model().camera_.Extrinsics();
  view->MutableCamera().SetExtrinsics(computed_extrinsics);

  // If the computed relative translation from the last keyframe is large
  // enough, it's time to initialize a new keyframe on the next iteration.
  View::Ptr keyframe = View::GetView(current_keyframe_);
  CHECK_NOTNULL(keyframe.get());
  const Pose T1 = keyframe->Camera().Extrinsics().WorldToCamera();
  const Pose T2 = computed_extrinsics.WorldToCamera();
  const Pose delta = T1.Delta(T2);
  if (delta.Translation().norm() > options_.min_keyframe_translation ||
      delta.AxisAngle().norm() > options_.min_keyframe_rotation) {
    initialize_new_keyframe_ = true;
  }

  return Status::Ok();
}

Status KeyframeVisualOdometry::GetKeypoints(const Image& image,
                                            std::vector<Keypoint>* keypoints) {
  CHECK_NOTNULL(keypoints)->clear();

  // Detect keypoints in the image.
  if (!keypoint_detector_.DetectKeypoints(image, *keypoints)) {
    return Status::Cancelled("Failed to detect keypoints.");
  }

  return Status::Ok();
}

Status KeyframeVisualOdometry::GetDescriptors(
    const Image& image, std::vector<Keypoint>* keypoints,
    std::vector<Feature>* features, std::vector<Descriptor>* descriptors) {
  CHECK_NOTNULL(features)->clear();
  CHECK_NOTNULL(descriptors)->clear();

  // TODO: Seed descriptor matching with track information.
  if (!descriptor_extractor_.DescribeFeatures(image, *keypoints, *features,
                                              *descriptors)) {
    return Status::Cancelled("Failed to describe features.");
  }
  printf("got %lu descriptors\n", descriptors->size());
  return Status::Ok();
}

unsigned int KeyframeVisualOdometry::NumEstimatedTracks() const {
  unsigned int estimated_count = 0;
  for (const auto& track_index : tracks_) {
    const Landmark::Ptr track = Landmark::GetLandmark(track_index);
    CHECK_NOTNULL(track.get());
    if (track->IsEstimated()) {
      estimated_count++;
    }
  }
  return estimated_count;
}

// Return view indices in the sliding window.
std::vector<ViewIndex> KeyframeVisualOdometry::SlidingWindowViewIndices() {
  std::vector<ViewIndex> sw_view_indices;
  int start = std::max(static_cast<int>(view_indices_.size()) -
                       static_cast<int>(options_.sliding_window_length), 0);

  for (size_t ii = start; ii < view_indices_.size(); ++ii) {
    sw_view_indices.push_back(view_indices_[ii]);
  }

  return sw_view_indices;
}

}  //\namespace bsfm
