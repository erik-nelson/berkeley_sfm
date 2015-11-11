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
#include <image/image.h>
#include <image/drawing_utils.h>
#include <matching/feature.h>
#include <matching/feature_match.h>
#include <matching/naive_matcher_2d2d.h>
#include <sfm/view.h>
#include <slam/visual_odometry.h>
#include <slam/visual_odometry_options.h>
#include <strings/join.h>
#include <strings/join_filepath.h>
#include <util/status.h>
#include <util/timer.h>

DEFINE_string(video_file, "visual_odometry_test.mp4",
              "Name of the video file to perform visual odometry on.");

using bsfm::Camera;
using bsfm::CameraIntrinsics;
using bsfm::Descriptor;
using bsfm::Image;
using bsfm::Feature;
using bsfm::FeatureMatchList;
using bsfm::NaiveMatcher2D2D;
using bsfm::PairwiseImageMatchList;
using bsfm::Status;
using bsfm::View;
using bsfm::VisualOdometry;
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
  intrinsics.SetImageLeft(0);
  intrinsics.SetImageTop(0);
  intrinsics.SetImageWidth(960);
  intrinsics.SetImageHeight(540);
  intrinsics.SetFU(1890.0);
  intrinsics.SetFV(1890.0);
  intrinsics.SetCU(480);
  intrinsics.SetCV(270);
  intrinsics.SetK(0.06455, -0.16778, -0.02109, 0.03352, 0.0);
  initial_camera.SetIntrinsics(intrinsics);

  VisualOdometryOptions vo_options;
  vo_options.feature_type = "FAST";
  vo_options.descriptor_type = "ORB";
  vo_options.sliding_window_length = 20;
  vo_options.adaptive_features = true;
  vo_options.adaptive_min = 400;
  vo_options.adaptive_max = 500;
  vo_options.adaptive_iters = 5;

  vo_options.draw_features = true;
  vo_options.draw_landmarks = true;
  vo_options.draw_inlier_observations = true;
  vo_options.draw_tracks = true;

  vo_options.matcher_options.use_lowes_ratio = true;
  vo_options.matcher_options.lowes_ratio = 0.75;
  vo_options.matcher_options.min_num_feature_matches = 8;
  vo_options.matcher_options.require_symmetric_matches = true;
  vo_options.matcher_options.only_keep_best_matches = false;
  vo_options.matcher_options.num_best_matches = 0;
  vo_options.matcher_options.enforce_maximum_descriptor_distance = false;
  vo_options.matcher_options.maximum_descriptor_distance = 0.0;
  vo_options.matcher_options.distance_metric = "HAMMING";

  // RANSAC iterations chosen using ~30% outliers @ 99% chance to sample from
  // Table 4.3 of H&Z.
  // Number of inliers ~= (1-0.3) * 50. Assumes 50 features observed on a frame.
  vo_options.fundamental_matrix_ransac_options.iterations = 78;
  vo_options.fundamental_matrix_ransac_options.acceptable_error = 1e-3;
  vo_options.fundamental_matrix_ransac_options.minimum_num_inliers = 35;
  vo_options.fundamental_matrix_ransac_options.num_samples = 8;

  // Number of inliers ~= (1-0.3) * 30. Assumes ~30 landmarks observed on a frame.
  // Error is squared reprojection. Allow for a 20 pixel error tolerance.
  vo_options.pnp_ransac_options.iterations = 100;
  vo_options.pnp_ransac_options.acceptable_error = 100.0;
  vo_options.pnp_ransac_options.minimum_num_inliers = 20;
  vo_options.pnp_ransac_options.num_samples = 6;

  vo_options.bundle_adjustment_options.solver_type = "SPARSE_SCHUR";
  vo_options.bundle_adjustment_options.print_summary = true;
  vo_options.bundle_adjustment_options.print_progress = false;
  vo_options.bundle_adjustment_options.max_num_iterations = 50;
  vo_options.bundle_adjustment_options.function_tolerance = 1e-16;
  vo_options.bundle_adjustment_options.gradient_tolerance = 1e-16;

  VisualOdometry vo(vo_options, initial_camera);

  // Draw and process frames of the video.
  cv::Mat cv_video_frame;
  capture.read(cv_video_frame);  // HACK: throw away first frame to draw matches.
  Image last_frame(cv_video_frame);
  vo.Update(last_frame);

  int iter = 0;
  while(true) {
    // Get the next frame.
    capture.read(cv_video_frame);
    if (cv_video_frame.empty()) {
      break;
    }

    // Process the frame.
    Image frame(cv_video_frame);
    Status s = vo.Update(frame);
    if (!s.ok()) std::cout << s.Message() << std::endl;

    // Store this frame for next time.
    last_frame = frame;

#if 0
    // cv::imshow(window_name.c_str(), cv_video_frame);

    // Wait based on the framerate. Our timer is slightly more reliable than
    // using cv::waitKey() alone.
    while (timer.Toc() < wait_in_seconds) {
      cv::waitKey(1);
    }
    timer.Tic();
#endif
  }

  return EXIT_SUCCESS;
}
