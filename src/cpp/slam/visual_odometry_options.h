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

#include "../geometry/fundamental_matrix_solver_options.h"
#include "../matching/feature_matcher_options.h"

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

  // Turn on or off adaptive features. Adaptive features only work when using
  // SURF, FAST, or STAR features. By default this feature is disabled. If it is
  // enabled, the minimum and maximum amount of features must be specified
  // manually, as must the number of iterations to adapt over per call to the
  // feature detector.
  bool adaptive_features = false;
  unsigned int adaptive_min = 0;
  unsigned int adaptive_max = 0;
  unsigned int adaptive_iters = 0;

  // The type of descriptor used to describe features in each image. Options
  // are:
  // - SIFT
  // - SURF
  // - BRIEF
  // - BRISK
  // - FREAK
  // - ORB
  std::string descriptor_type = "ORB";

  // A set of options used for feature matching. Default values are specified in
  // the matching/feature_matcher_options.h header.
  FeatureMatcherOptions matcher_options;

  // A set of options used for computing the fundamental matrix between two
  // cameras. Default values are specified in the
  // geometry/fundamental_matrix_solver_options.h header.
  FundamentalMatrixSolverOptions f_solver_options;

};  //\struct VisualOdometryOptions

}  //\namespace bsfm

#endif
