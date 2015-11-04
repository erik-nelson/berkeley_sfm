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

#include "bundle_adjuster.h"

#include <ceres/ceres.h>
#include <glog/logging.h>

#include "../optimization/cost_functors.h"

namespace bsfm {

// Solve the bundle adjustment problem, internally updating the positions of all
// views in 'view_indices', as well as all landmarks that they jointly observe
// (any landmark seen by at least 2 views).
bool BundleAdjuster::Solve(const BundleAdjustmentOptions& options,
                           const std::vector<ViewIndex>& view_indices) const {
  // Create a ceres optimization problem.
  ceres::Problem problem;

  // Create storage containers for optimization variables. We only do this
  // because camera extrinsics by default are represented as a 4x4 homogeneous
  // matrix, and we would prefer to optimize 6 camera variables (rotation and
  // translation) instead of 16.
  std::vector<Vector3d> rotations(view_indices.size());
  std::vector<Vector3d> translations(view_indices.size());

  for (size_t ii = 0; ii < view_indices.size(); ++ii) {
    View::Ptr view = View::GetView(view_indices[ii]);
    if (view == nullptr) {
      LOG(WARNING) << "View is null. Cannot perform bundle adjustment.";
      return false;
    }

    // Get camera extrinsics matrix for optimization.
    rotations[ii] = view->Camera().AxisAngleRotation();
    translations[ii] = view->Camera().Translation();

    // Get static camera intrinsics for evaluating cost function.
    Matrix3d K = view->Camera().K();

    // Make a new residual block on the problem for every 3D point that this
    // view sees.
    const std::vector<Observation::Ptr> observations = view->Observations();
    for (size_t jj = 0; jj < observations.size(); ++jj) {
      CHECK_NOTNULL(observations[jj].get());
      if (!observations[jj]->IsMatched())
        continue;

      Landmark::Ptr landmark = observations[jj]->GetLandmark();
      if (landmark == nullptr) {
        LOG(WARNING) << "Landmark is null. Cannot perform bundle adjustment.";
        return false;
      }

      // Make sure the landmark has been seen by at least two of the views we
      // are doing bundle adjustment over.
      if (!landmark->SeenByAtLeastTwoViews(view_indices))
        continue;

      // Add a residual block to the cost function.
      problem.AddResidualBlock(
          BundleAdjustmentError::Create(observations[jj]->Feature(), K),
          NULL, /* squared loss */
          rotations[ii].data(),
          translations[ii].data(),
          landmark->PositionData());
    }
  }

  // Solve the bundle adjustment problem.
  ceres::Solver::Summary summary;
  ceres::Solver::Options ceres_options;
  // ceres_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  ceres_options.linear_solver_type = ceres::DENSE_SCHUR;
  ceres::Solve(ceres_options, &problem, &summary);

  // Assign optimized camera parameters back into views.
  for (size_t ii = 0; ii < view_indices.size(); ++ii) {
    Pose pose;
    pose.FromAxisAngle(rotations[ii]);

    // We already know this view is not null.
    View::Ptr view = View::GetView(view_indices[ii]);
    view->MutableCamera().MutableExtrinsics().SetWorldToCamera(pose);
    view->MutableCamera().MutableExtrinsics().SetTranslation(translations[ii]);
  }

  return summary.IsSolutionUsable();
}

}  //\namespace bsfm
