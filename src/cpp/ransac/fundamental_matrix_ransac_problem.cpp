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
// This class defines the FundamentalMatrixRansacModel class, which
// is derived from the abstract base class RansacModel.
//
///////////////////////////////////////////////////////////////////////////////

#include <iostream>

#include <algorithm>
#include <Eigen/Core>
#include <gflags/gflags.h>
#include <vector>

#include "ransac_problem.h"
#include "fundamental_matrix_ransac_problem.h"
#include "../math/random_generator.h"
#include "../geometry/eight_point_algorithm_solver.h"
#include "../geometry/fundamental_matrix_solver_options.h"

namespace bsfm {

// ------------ FundamentalMatrixRansacModel methods ------------ //

// Default constructor.
FundamentalMatrixRansacModel::FundamentalMatrixRansacModel()
    : F_(Matrix3d::Identity()), error_(0.0) {}

FundamentalMatrixRansacModel::FundamentalMatrixRansacModel(
    const Matrix3d& F)
    : F_(F), error_(0.0) {}

// Destructor.
FundamentalMatrixRansacModel::~FundamentalMatrixRansacModel() {}

// Return model error.
double FundamentalMatrixRansacModel::Error() const {
  return error_;
}

// Evaluate model on a single data element and update error.
bool FundamentalMatrixRansacModel::IsGoodFit(
    const FeatureMatch& data_point,
    double error_tolerance) {
  const double error = EvaluateEpipolarCondition(data_point);

  // Test squared error against the provided tolerance.
  if (error * error < error_tolerance) {
    return true;
  }
  return false;
}

double FundamentalMatrixRansacModel::EvaluateEpipolarCondition(
    const FeatureMatch& match) const {
  // Construct vectors for 2D keypoints in match.
  Vector3d kp1, kp2;
  kp1 << match.feature1_.u_, match.feature1_.v_, 1;
  kp2 << match.feature2_.u_, match.feature2_.v_, 1;

  // Compute deviation from the epipolar condition.
  const double epipolar_condition = kp2.transpose() * F_ * kp1;
  return epipolar_condition;
}

// ------------ FundamentalMatrixRansacProblem methods ------------ //

// RansacProblem constructor.
FundamentalMatrixRansacProblem::FundamentalMatrixRansacProblem() {}

// RansacProblem destructor.
FundamentalMatrixRansacProblem::~FundamentalMatrixRansacProblem() {}

// Subsample the data.
std::vector<FeatureMatch> FundamentalMatrixRansacProblem::SampleData(
    unsigned int num_samples) {
  // Randomly shuffle the entire dataset and take the first elements.
  std::random_shuffle(data_.begin(), data_.end());

  // Make sure we don't over step.

  if (static_cast<size_t>(num_samples) > data_.size()) {
    VLOG(1) << "Requested more RANSAC data samples than are available. "
               "Returning all data.";
    num_samples = data_.size();
  }

  // Get samples.
  std::vector<FeatureMatch> samples(
      data_.begin(), data_.begin() + static_cast<size_t>(num_samples));

  return samples;
}

// Return all data that was not sampled.
std::vector<FeatureMatch> FundamentalMatrixRansacProblem::RemainingData(
    unsigned int num_sampled_previously) const {
  // In Sample(), the data was shuffled and we took the first
  // 'num_sampled_previously' elements. Here, take the remaining elements.
  if (num_sampled_previously >= data_.size()) {
    VLOG(1) << "No remaining RANSAC data to sample.";
    return std::vector<FeatureMatch>();
  }

  return std::vector<FeatureMatch>(
      data_.begin() + num_sampled_previously, data_.end());
}

// Fit a model to the provided data using the 8-point algorithm.
FundamentalMatrixRansacModel FundamentalMatrixRansacProblem::FitModel(
    const std::vector<FeatureMatch>& input_data) const {
  // Create an empty fundamental matrix.
  Matrix3d F;

  // Run the 8-point algorithm with default options.
  EightPointAlgorithmSolver solver;
  FundamentalMatrixSolverOptions options;
  solver.SetOptions(options);

  if (solver.ComputeFundamentalMatrix(input_data, F)) {
    // Create a new RansacModel using the computed fundamental matrix.
    FundamentalMatrixRansacModel model_out(F);

    // Record sum of squared error over all matches.
    model_out.error_ = 0.0;
    for (const auto& feature_match : input_data) {
      const double error = model_out.EvaluateEpipolarCondition(feature_match);
      model_out.error_ += error * error;
    }

    return model_out;
  }
  // Set a large error - we didn't find a model that fits the data.
  FundamentalMatrixRansacModel model_out(Matrix3d::Identity());
  model_out.error_ = std::numeric_limits<double>::infinity();
  return model_out;
}

}  //\namespace bsfm
