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
// This struct defines options for visual odometry.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_SLAM_VISUAL_ODOMETRY_OPTIONS_H
#define BSFM_SLAM_VISUAL_ODOMETRY_OPTIONS_H

#include <string>

#include "../matching/feature_matcher_options.h"
#include "../ransac/ransac_options.h"
#include "../sfm/bundle_adjustment_options.h"

namespace bsfm {

struct VisualOdometryOptions {
  // The type of feature that will be extracted on each image. Options are:
  // - SURF
  // - FAST
  // - STAR
  // - SIFT
  // - ORB
  // - BRISK
  // - MSER
  // - GFTT
  // - HARRIS
  // - DENSE
  // - SIMPLEBLOB
  std::string feature_type = "FAST";

  // Turn on or off feature grid filtering. By default this feature is disabled.
  // If it is enabled, the row, column, and minimum number of points fields must
  // be specified.
  bool use_grid_filter = false;
  unsigned int grid_rows = 0;
  unsigned int grid_cols = 0;
  unsigned int grid_min_num_features = 0;

  // The type of descriptor used to describe features in each image. Options
  // are:
  // - SIFT
  // - SURF
  // - BRIEF
  // - BRISK
  // - FREAK
  // - ORB
  std::string descriptor_type = "ORB";

  // Length of the sliding window (number of images). The higher this value, the
  // more accurate the localization and mapping, but the slower the process.
  unsigned int sliding_window_length = 10;

  // Turn on or off bundle adjustment over the sliding window.
  bool perform_bundle_adjustment = true;

  // We need to triangulated at least this many landmarks to begin doing 2D to
  // 3D pose estimation.
  unsigned int num_landmarks_to_initialize = 20;

  // ---------------------- DRAWING OPTIONS ---------------------- //
  // If any of the drawing features below are enabled, an OpenCV window will be
  // displayed with the selected options overlaid on the current frame.

  // Draw detected features as colored circles.
  bool draw_features = true;

  // Draw landmarks seen by at least one camera in the sliding window as
  // squares. Also print normalized distance from the camera to each landmark.
  bool draw_landmarks = true;

  // Draw inlier matches between 2D features in the frame and 3D landmarks. If
  // either of the above 2 options are false, this option will be overridden and
  // matches will not be displayed.
  bool draw_inlier_observations = true;

  // Draw landmark tracks. This is done by finding each landmark in other views
  // in the sliding window, and drawing where those views would have seen the
  // landmark. This option gives a sense of the motion of the camera.
  bool draw_tracks = true;
  // ---------------------- DRAWING OPTIONS ---------------------- //

  // A set of options used for feature matching. Default values are specified in
  // the matching/feature_matcher_options.h header.
  FeatureMatcherOptions matcher_options;

  // A set of options used for finding the fundamental matrix between two
  // cameras with RANSAC. Default values are specified in the
  // ransac/ransac_options.h header.
  RansacOptions fundamental_matrix_ransac_options;

  // A set of options used for running RANSAC to find the pose of a new camera
  // by matching it against known 3D landmarks. Default values are specified in
  // the ransac/ransac_options.h header.
  RansacOptions pnp_ransac_options;

  // Options for bundle adjustment. Default values are specified in the
  // sfm/bundle_adjustment_options.h header.
  BundleAdjustmentOptions bundle_adjustment_options;

};  //\struct VisualOdometryOptions

}  //\namespace bsfm

#endif
