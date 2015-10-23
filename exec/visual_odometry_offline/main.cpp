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

#include <image/image.h>
#include <strings/join.h>
#include <strings/join_filepath.h>
#include <util/progress_tracker.h>
#include <util/timer.h>

DEFINE_string(video_file, "visual_odometry_test.mp4",
              "Name of the video file to perform visual odometry on.");

using bsfm::strings::Join;
using bsfm::strings::JoinFilepath;
using bsfm::util::ProgressTracker;
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

  // Initialize a timer and progress tracker bar.
  Timer timer;
  unsigned int frame_number = 0;
  ProgressTracker progress("Video playback",
                           capture.get(CV_CAP_PROP_FRAME_COUNT));

  // Draw frames of the video.
  cv::Mat cv_video_frame;
  while(true) {
    progress.Update(frame_number++);

    // Get the next frame.
    capture.read(cv_video_frame);
    if (cv_video_frame.empty()) {
      break;
    }

    // Display the frame.
    cv::imshow(window_name.c_str(), cv_video_frame);

    // Wait based on the framerate. Our timer is slightly more reliable than
    // using cv::waitKey() alone.
    while (timer.Toc() < wait_in_seconds) {
      cv::waitKey(1);
    }
    timer.Tic();
  }

  return EXIT_SUCCESS;
}
