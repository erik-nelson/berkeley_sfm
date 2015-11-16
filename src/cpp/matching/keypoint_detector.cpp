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
    : adaptive_(false),
      adaptive_min_(0),
      adaptive_max_(0),
      adaptive_iters_(0) {
  cv::initModule_nonfree();
}

bool KeypointDetector::SetDetector(const std::string& detector_type) {
  // Set the detector type.
  detector_type_ = detector_type;
  bool valid_detector_type = true;

  // Create an OpenCV feature detector. SURF, FAST, and STAR support adjusters.
  if (detector_type.compare("SURF") == 0) {

    if (adaptive_) {
      cv::Ptr<cv::AdjusterAdapter> adjuster(
          cv::AdjusterAdapter::create("SURF"));
      detector_ =
          cv::Ptr<cv::FeatureDetector>(new cv::DynamicAdaptedFeatureDetector(
              adjuster, adaptive_min_, adaptive_max_, adaptive_iters_));
    } else {
      detector_ = cv::FeatureDetector::create("SURF");
    }

  } else if (detector_type.compare("FAST") == 0) {

    if (adaptive_) {
      cv::Ptr<cv::AdjusterAdapter> adjuster(
          cv::AdjusterAdapter::create("FAST"));
      detector_ =
          cv::Ptr<cv::FeatureDetector>(new cv::DynamicAdaptedFeatureDetector(
              adjuster, adaptive_min_, adaptive_max_, adaptive_iters_));
    } else {
      detector_ = cv::FeatureDetector::create("FAST");
    }

  } else if (detector_type.compare("STAR") == 0) {

    if (adaptive_) {
      cv::Ptr<cv::AdjusterAdapter> adjuster(
          cv::AdjusterAdapter::create("STAR"));
      detector_ =
          cv::Ptr<cv::FeatureDetector>(new cv::DynamicAdaptedFeatureDetector(
              adjuster, adaptive_min_, adaptive_max_, adaptive_iters_));
    } else {
      detector_ = cv::FeatureDetector::create("STAR");
    }

  } else if (detector_type.compare("SIFT") == 0) {

    detector_ = cv::FeatureDetector::create("SIFT");

  } else if (detector_type.compare("ORB") == 0) {

    detector_ = cv::FeatureDetector::create("ORB");

  } else if (detector_type.compare("BRISK") == 0) {

    detector_ = cv::FeatureDetector::create("BRISK");

  } else if (detector_type.compare("MSER") == 0) {

    detector_ = cv::FeatureDetector::create("MSER");

  } else if (detector_type.compare("GFTT") == 0) {

    detector_ = cv::FeatureDetector::create("GFTT");

  } else if (detector_type.compare("GFTT") == 0) {

    detector_ = cv::FeatureDetector::create("GFTT");

  } else if (detector_type.compare("HARRIS") == 0) {

    detector_ = cv::FeatureDetector::create("HARRIS");

  } else if (detector_type.compare("DENSE") == 0) {

    detector_ = cv::FeatureDetector::create("Dense");

  } else if (detector_type.compare("SIMPLEBLOB") == 0) {

    detector_ = cv::FeatureDetector::create("SimpleBlob");

  } else {
    VLOG(1) << "Detector type \"" << detector_type
            << "\"is not available. Defaulting to FAST.";

    // Default to a FAST detector.
    if (adaptive_) {
      cv::Ptr<cv::AdjusterAdapter> adjuster(
          cv::AdjusterAdapter::create("FAST"));
      detector_ =
          cv::Ptr<cv::FeatureDetector>(new cv::DynamicAdaptedFeatureDetector(
              adjuster, adaptive_min_, adaptive_max_, adaptive_iters_));
    } else {
      detector_ = cv::FeatureDetector::create("FAST");
    }

    valid_detector_type = false;
    detector_type_ = "FAST";
  }

  return valid_detector_type;
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

// Turn adaptive features on.
void KeypointDetector::SetAdaptiveOn(unsigned int min, unsigned int max,
                                     unsigned int iters) {
  if (!SupportsAdaptiveAdjustment()) {
    VLOG(1) << "The detector type \"" << detector_type_
            << "\" does not support adaptive feature count adjustment. Type "
               "must be one of {FAST, SURF, STAR}";
    return;
  }

  adaptive_ = true;
  adaptive_min_ = min;
  adaptive_max_ = max;
  adaptive_iters_ = iters;

  // If the feature detector is already set, re-initialize it.
  if (detector_ != nullptr) {
    SetDetector(detector_type_);
  }
}

// Turn adaptive feature counts off.
void KeypointDetector::SetAdaptiveOff() {
if (!SupportsAdaptiveAdjustment()) {
  VLOG(1) << "The detector type \"" << detector_type_
          << "\" does not support adaptive feature count adjustment, and "
             "therefore is already static.";
    return;
  }

  adaptive_ = false;
  adaptive_min_ = 0;
  adaptive_max_ = 0;
  adaptive_iters_ = 0;

  // If the feature detector is already set, re-initialize it.
  if (detector_ != nullptr) {
    SetDetector(detector_type_);
  }
}

// Adaptive adjustment is supported with FAST, SURF, and STAR features.
bool KeypointDetector::SupportsAdaptiveAdjustment() const {
  if (detector_type_.compare("FAST") == 0 ||
      detector_type_.compare("SURF") == 0 ||
      detector_type_.compare("STAR") == 0) {
    return true;
  }

  return false;
}

}  //\namespace bsfm
