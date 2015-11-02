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
// This class defines a bundle adjustment problem solver. Bundle adjustment is
// the problem of determining the 3D positions of n points and the 3D positions
// of m cameras simultaneously using features that are matched between the m
// views. Bundle adjustment is a non-linear least squares optimization that
// attempts to minimize the reprojection error of the 3D points in the views of
// each camera.
//
// With a large number of points and cameras, this quickly gets to be a tough
// optimization problem. In many cases, bundle adjustment requires a long time
// to solve, and is also dependent on a good initialization (i.e. a prior on the
// 3D positions of all cameras and points).
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_SFM_BUNDLE_ADJUSTER_H
#define BSFM_SFM_BUNDLE_ADJUSTER_H

#include <Eigen/Core>
#include <vector>

#include "bundle_adjustment_options.h"
#include "view.h"
#include "../slam/landmark.h"
#include "../util/disallow_copy_and_assign.h"
#include "../util/types.h"

namespace bsfm {

using Eigen::Map;
using Eigen::Matrix3d;
using Eigen::Vector3d;

class BundleAdjuster {
 public:
  BundleAdjuster() { }
  ~BundleAdjuster() { }

  // Solve the bundle adjustment problem, internally updating the positions of
  // all views in 'view_indices', as well as all landmarks that they jointly
  // observe (any landmark seen by at least 2 views).
  bool Solve(const BundleAdjustmentOptions& options,
             const std::vector<ViewIndex>& view_indices) const;

 private:
  DISALLOW_COPY_AND_ASSIGN(BundleAdjuster)

};  //\class BundleAdjuster

}  //\namespace bsfm

#endif
