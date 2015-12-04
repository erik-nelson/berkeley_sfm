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
// This file is the program entry point for offline visual odometry. The program
// will load a video, and process its frames 1-by-1 to localize the camera's
// position in the world from feature matches across frames, up to scale.
//
///////////////////////////////////////////////////////////////////////////////

#include <gflags/gflags.h>
#include <opencv2/opencv.hpp>

#include <camera/camera.h>
#include <camera/camera_intrinsics.h>
#include <geometry/rotation.h>
#include <image/image.h>
#include <image/drawing_utils.h>
#include <matching/feature.h>
#include <matching/feature_match.h>
#include <matching/naive_matcher_2d2d.h>
#include <sfm/view.h>
#include <slam/keyframe_visual_odometry.h>
#include <slam/visual_odometry_options.h>
#include <strings/join.h>
#include <strings/join_filepath.h>
#include <util/status.h>
#include <util/timer.h>

// DEFINE_string(video_file, "visual_odometry_test1.mp4",
DEFINE_string(video_file, "../../../../Desktop/KITTI_datasets/KITTI2.mp4",
// DEFINE_string(video_file, "KITTI2.mp4",
              "Name of the video file to perform visual odometry on.");
DEFINE_string(video_output_file, "annotated_video.mp4",
              "Name of the annotated output. If this string is empty, an "
              "output video will not be created.");
DEFINE_bool(rotate_images, false, "Rotate the images clockwise by 90 degrees.");

using bsfm::Camera;
using bsfm::CameraIntrinsics;
using bsfm::Descriptor;
using bsfm::D2R;
using bsfm::Image;
using bsfm::Feature;
using bsfm::FeatureMatchList;
using bsfm::KeyframeVisualOdometry;
using bsfm::NaiveMatcher2D2D;
using bsfm::PairwiseImageMatchList;
using bsfm::Status;
using bsfm::View;
using bsfm::VisualOdometryOptions;
using bsfm::drawing::AnnotateLandmarks;
using bsfm::drawing::DrawImageFeatureMatches;
using bsfm::strings::Join;
using bsfm::strings::JoinFilepath;
using bsfm::util::Timer;

int main(int argc, char** argv) {
  const std::string video_file = JoinFilepath(
      BSFM_EXEC_DIR, "visual_odometry_offline", FLAGS_video_file.c_str());

  // Open up the video.
  cv::VideoCapture capture(video_file);
  if (!capture.isOpened()) {
    VLOG(1) << "Failed to open video file: " << video_file << ". Exiting.";
    return EXIT_FAILURE;
  }
  const double frame_rate = capture.get(CV_CAP_PROP_FPS);
  const double wait_in_seconds = 1.0 / frame_rate;

  // Create a window for visualization.
  const std::string window_name =
      Join("Visual Odometry: ", FLAGS_video_file.c_str());
  cv::namedWindow(window_name.c_str(), CV_WINDOW_AUTOSIZE);

  // Initialize a timer.
  Timer timer;

  // Initialize visual odometry.
  Camera initial_camera;
  CameraIntrinsics intrinsics;

#if 0
  // HTC one.
  intrinsics.SetImageLeft(0);
  intrinsics.SetImageTop(0);
  intrinsics.SetImageWidth(432);
  intrinsics.SetImageHeight(786);
  intrinsics.SetFU(1890.0);
  intrinsics.SetFV(1890.0);
  intrinsics.SetCU(216);
  intrinsics.SetCV(393);
  intrinsics.SetK(0.06455, -0.16778, -0.02109, 0.03352, 0.0);
#endif

#if 0
  // HTC one, large images.
  intrinsics.SetImageLeft(0);
  intrinsics.SetImageTop(0);
  intrinsics.SetImageWidth(720);
  intrinsics.SetImageHeight(1280);
  intrinsics.SetFU(1890.0);
  intrinsics.SetFV(1890.0);
  intrinsics.SetCU(360);
  intrinsics.SetCV(640);
  intrinsics.SetK(0.06455, -0.16778, -0.02109, 0.03352, 0.0);
#endif

#if 0
  // KITTI 2011_09_26
  intrinsics.SetImageLeft(0);
  intrinsics.SetImageTop(0);
  intrinsics.SetImageWidth(1392);
  intrinsics.SetImageHeight(512);
  intrinsics.SetFU(959.7910);
  intrinsics.SetFV(956.9251);
  intrinsics.SetCU(696.0217);
  intrinsics.SetCV(224.1806);
  intrinsics.SetK(-0.3691481, 0.1968681, 0.001353473, 0.0005677587, -0.06770705);
#endif

// #if 0
  // KITTI 2011_09_30
  intrinsics.SetImageLeft(0);
  intrinsics.SetImageTop(0);
  intrinsics.SetImageWidth(1242);
  intrinsics.SetImageHeight(375);
  intrinsics.SetFU(721.5377);
  intrinsics.SetFV(721.5377);
  intrinsics.SetCU(609.5593);
  intrinsics.SetCV(172.8540);
// #endif

  initial_camera.SetIntrinsics(intrinsics);

  VisualOdometryOptions vo_options;
  vo_options.feature_type = "FAST";
  vo_options.descriptor_type = "SIFT";
  vo_options.sliding_window_length = 1;
  vo_options.use_grid_filter = true;
  vo_options.grid_rows = 7;
  vo_options.grid_cols = 4;
  vo_options.grid_min_num_features = 1000;

  vo_options.min_num_feature_tracks = 500;
  vo_options.num_observations_to_triangulate = 2;
  vo_options.min_keyframe_translation = 0.5;
  vo_options.min_keyframe_rotation = D2R(5.0);
  vo_options.num_landmarks_to_initialize = 30;

  vo_options.draw_tracks = true;

  vo_options.matcher_options.use_lowes_ratio = true;
  vo_options.matcher_options.lowes_ratio = 0.85;
  vo_options.matcher_options.min_num_feature_matches = 8;
  vo_options.matcher_options.require_symmetric_matches = true;
  vo_options.matcher_options.only_keep_best_matches = false;
  vo_options.matcher_options.num_best_matches = 0;
  vo_options.matcher_options.enforce_maximum_descriptor_distance = false;
  vo_options.matcher_options.maximum_descriptor_distance = 0.0;
  vo_options.matcher_options.distance_metric = "SCALED_L2";
  vo_options.matcher_options.threshold_image_distance = true;
  vo_options.matcher_options.maximum_image_distance = 20;  // pixels.

  // RANSAC iterations chosen using ~10% outliers @ 99% chance to sample from
  // Table 4.3 of H&Z.
  vo_options.fundamental_matrix_ransac_options.iterations = 100;
  vo_options.fundamental_matrix_ransac_options.acceptable_error = 1e-1;
  vo_options.fundamental_matrix_ransac_options.minimum_num_inliers = 35;
  vo_options.fundamental_matrix_ransac_options.num_samples = 8;

  vo_options.pnp_ransac_options.iterations = 100;
  vo_options.pnp_ransac_options.acceptable_error = 4.0;
  vo_options.pnp_ransac_options.minimum_num_inliers = 10;
  vo_options.pnp_ransac_options.num_samples = 6;

  vo_options.perform_bundle_adjustment = false;
  vo_options.bundle_adjustment_options.solver_type = "SPARSE_SCHUR";
  vo_options.bundle_adjustment_options.print_summary = false;
  vo_options.bundle_adjustment_options.print_progress = false;
  vo_options.bundle_adjustment_options.max_num_iterations = 5;
  vo_options.bundle_adjustment_options.function_tolerance = 1e-16;
  vo_options.bundle_adjustment_options.gradient_tolerance = 1e-16;

  KeyframeVisualOdometry vo(vo_options, intrinsics);

  // Draw and process frames of the video.
  cv::Mat cv_video_frame;
  capture.read(cv_video_frame);
  Image last_frame(cv_video_frame);
  if (FLAGS_rotate_images)
    last_frame.RotateClockwise();

  vo.Update(last_frame);

  // If we are writing any output, initialize a video writer.
  cv::VideoWriter writer;
  if (!FLAGS_video_output_file.empty()) {
    const std::string video_output_file = JoinFilepath(
      BSFM_EXEC_DIR, "visual_odometry_offline", FLAGS_video_output_file.c_str());

    const int w = capture.get(CV_CAP_PROP_FRAME_WIDTH);
    const int h = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
    const int codec = CV_FOURCC('m', 'p', '4', 'v');

    cv::Size frame_size;
    if (FLAGS_rotate_images) {
      frame_size = cv::Size(h, w);
    } else {
      frame_size = cv::Size(w, h);
    }

    writer.open(video_output_file.c_str(), codec, frame_rate,
                frame_size, false /*no grayscale*/);
  }

  // Skip several frames at the beginning to get a nice baseline.
  const int start = 2;
  capture.set(CV_CAP_PROP_POS_FRAMES, start);

  const int skip = 1;
  for (int frame_iterator = start + skip; ; frame_iterator += skip) {
    capture.set(CV_CAP_PROP_POS_FRAMES, frame_iterator);

    // Get the next frame.
    capture.read(cv_video_frame);
    if (cv_video_frame.empty()) {
      break;
    }

    // Process the frame.
    Image frame(cv_video_frame);
    if (FLAGS_rotate_images)
      frame.RotateClockwise();

    Status s = vo.Update(frame);
    if (!s.ok()) {
      std::cout << s.Message() << std::endl;
    }

    // Write the annotated frame into the output video.
    if (writer.isOpened()) {
      Image annotated_image;
      vo.GetAnnotatedImage(&annotated_image);
      cv::Mat cv_annotated_image;
      annotated_image.ToCV(cv_annotated_image);
      writer << cv_annotated_image;
    }

    // Store this frame for next time.
    last_frame = frame;

    // if (frame_iterator > 200)
      // break;
  }
  writer.release();

  vo.WriteTrajectoryToFile("vo_trajectory.csv");
  vo.WriteMapToFile("vo_map.csv");

  return EXIT_SUCCESS;
}
