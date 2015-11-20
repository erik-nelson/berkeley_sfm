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

#include "keypoint_detector.h"

#include <glog/logging.h>

namespace bsfm {

KeypointDetector::KeypointDetector()
    : grid_filter_(false),
      grid_rows_(0),
      grid_cols_(0),
      grid_min_num_points_(0) {
  cv::initModule_nonfree();
}

void KeypointDetector::SetDetector(const std::string& detector_type) {
  // Set the detector type.
  detector_type_ = detector_type;

  // Create an OpenCV feature detector.
    if (grid_filter_) {
      detector_ =
          cv::Ptr<cv::FeatureDetector>(new cv::GridAdaptedFeatureDetector(
              cv::FeatureDetector::create(detector_type), grid_min_num_points_,
              grid_rows_, grid_cols_));
    } else {
      detector_ = cv::FeatureDetector::create(detector_type);
    }
}

bool KeypointDetector::DetectKeypoints(const Image& image,
                                       KeypointList& keypoints_out) {
  // Make the user has called SetDetector().
  if (detector_type_.empty()) {
    VLOG(1) << "Detector has not been specified via SetDetector(). Failed to "
               "detect keypoints.";
    return false;
  }
  CHECK(detector_) << "The feature detector is null.";

  // Clear the output.
  keypoints_out.clear();

  // Convert the input image to OpenCV's format.
  cv::Mat cv_image;
  image.ToCV(cv_image);

  // Convert to grayscale if the image was in color.
  if (image.IsColor()) {
    cv::cvtColor(cv_image, cv_image, CV_BGR2GRAY);
  }

  // Detect keypoints in the image.
  try {
    detector_->detect(cv_image, keypoints_out);
  } catch (const std::exception& e) {
    VLOG(1) << "Failed to detect keypoints: " << e.what();
    return false;
  }

  return true;
}

// Turn the grid filter on.
void KeypointDetector::SetGridOn(unsigned int rows, unsigned int cols,
                                 unsigned int min_num_points) {
  grid_filter_ = true;
  grid_rows_ = rows;
  grid_cols_ = cols;
  grid_min_num_points_ = min_num_points;

  // If the feature detector is already set, re-initialize it.
  if (detector_ != nullptr) {
    SetDetector(detector_type_);
  }
}

// Turn grid filter off.
void KeypointDetector::SetGridOff() {
  grid_filter_ = false;
  grid_rows_ = 0;
  grid_cols_ = 0;
  grid_min_num_points_ = 0;

  // If the feature detector is already set, re-initialize it.
  if (detector_ != nullptr) {
    SetDetector(detector_type_);
  }
}

}  //\namespace bsfm
