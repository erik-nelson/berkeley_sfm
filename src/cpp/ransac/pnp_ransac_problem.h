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
 * Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 *          Erik Nelson            ( eanelson@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// These classes define the PnPRansacProblem API, and derive from the base
// RansacProblem and RansacModel class/struct.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_RANSAC_2D_3D_RANSAC_PROBLEM_H
#define BSFM_RANSAC_2D_3D_RANSAC_PROBLEM_H

#include <Eigen/Dense>
#include <vector>

#include "ransac_problem.h"
#include "../camera/camera.h"
#include "../camera/camera_intrinsics.h"
#include "../geometry/point_3d.h"
#include "../slam/landmark.h"
#include "../slam/observation.h"
#include "../matching/feature.h"
#include "../util/disallow_copy_and_assign.h"

namespace bsfm {

using Eigen::Matrix3d;
using Eigen::Vector3d;

// ------------ PnPRansacModel derived ------------ //

struct PnPRansacModel : public RansacModel<Observation::Ptr> {
  PnPRansacModel();
  virtual ~PnPRansacModel();

  // Define an additional constructor specifically for this model.
  PnPRansacModel(const Camera& camera,
		 const std::vector<Observation::Ptr>& matches);

  // Return model error.
  virtual double Error() const;

  // Evaluate model on a single data element and update error.
  virtual bool IsGoodFit(const Observation::Ptr& observation,
			 double error_tolerance);

  // Compute reprojection error of this landmark.
  double EvaluateReprojectionError(const Observation::Ptr& observation) const;

  // Model-specific member variables.
  const Camera camera_;
  const std::vector<Observation::Ptr> matches_;
  double error_;
};  //\struct PnPRansacModel


// ------------ PnPRansacProblem derived ------------ //

class PnPRansacProblem
    : public RansacProblem<Observation::Ptr, PnPRansacModel> {
 public:
  PnPRansacProblem();
  virtual ~PnPRansacProblem();

  // Set intrinsics.
  void SetIntrinsics(CameraIntrinsics& intrinsics);

  // Subsample the data.
  virtual std::vector<Observation::Ptr> SampleData(unsigned int num_samples);

  // Return the data that was not sampled.
  virtual std::vector<Observation::Ptr> RemainingData(
      unsigned int num_sampled_previously) const;

  // Fit a model to the provided data using PoseEstimatorPnP.
  virtual PnPRansacModel FitModel(
      const std::vector<Observation::Ptr>& input_data) const;

 private:
  CameraIntrinsics intrinsics_;
  DISALLOW_COPY_AND_ASSIGN(PnPRansacProblem)
};  //\class PnPRansacProblem

} //\namespace bsfm

#endif
