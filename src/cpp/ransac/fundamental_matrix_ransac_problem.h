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
// These classes defines the FundamentalMatrixRansacProblem API, and derive from
// the base RansacProblem and RansacModel class/struct.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_RANSAC_FUNDAMENTAL_MATRIX_RANSAC_PROBLEM_H
#define BSFM_RANSAC_FUNDAMENTAL_MATRIX_RANSAC_PROBLEM_H

#include <Eigen/Dense>
#include <vector>

#include "ransac_problem.h"
#include "../matching/feature_match.h"
#include "../util/disallow_copy_and_assign.h"

namespace bsfm {

using Eigen::Matrix3d;
using Eigen::Vector3d;

// ------------ FundamentalMatrixRansacModel derived ------------ //

struct FundamentalMatrixRansacModel : public RansacModel<FeatureMatch> {
  FundamentalMatrixRansacModel();
  virtual ~FundamentalMatrixRansacModel();

  // Define an additional constructor specifically for this model.
  FundamentalMatrixRansacModel(const Matrix3d& F);

  // Return model error.
  virtual double Error() const;

  // Evaluate model on a single data element and update error.
  virtual bool IsGoodFit(const FeatureMatch& data_point,
                         double error_tolerance) const;

  // Compute x1' * F_ * x2 for the input match.
  double EvaluateEpipolarCondition(const FeatureMatch& match) const;

  // Model-specific member variables.
  Matrix3d F_;
  double error_;
};  //\struct FundamentalMatrixRansacModel


// ------------ FundamentalMatrixRansacProblem derived ------------ //

class FundamentalMatrixRansacProblem
    : public RansacProblem<FeatureMatch, FundamentalMatrixRansacModel > {
 public:
  FundamentalMatrixRansacProblem();
  virtual ~FundamentalMatrixRansacProblem();

  // Subsample the data.
  virtual std::vector<FeatureMatch> SampleData(unsigned int num_samples);

  // Return the data that was not sampled.
  virtual std::vector<FeatureMatch> RemainingData(
      unsigned int num_sampled_previously) const;

  // Fit a model to the provided data using the 8-point algorithm.
  virtual FundamentalMatrixRansacModel FitModel(
      const std::vector<FeatureMatch>& input_data) const;

 private:
  DISALLOW_COPY_AND_ASSIGN(FundamentalMatrixRansacProblem)
};  //\class FundamentalMatrixRansacProblem

} //\namespace bsfm

#endif
