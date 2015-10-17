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
// This file defines an abstract base class to be derived by classes
// implementing a fundamental matrix solver (e.g. 4-point, 5-point, 7-point,
// 8-point algorithm solvers). These solvers determine the fundamental matrix
// between two cameras using a set of matched features in a two-view image pair.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_FUNDAMENTAL_MATRIX_SOLVER_H
#define BSFM_FUNDAMENTAL_MATRIX_SOLVER_H

#include <Eigen/Core>
#include <glog/logging.h>
#include <vector>

#include "fundamental_matrix_solver_options.h"
#include "../matching/pairwise_image_match.h"
#include "../util/disallow_copy_and_assign.h"

namespace bsfm {

class FundamentalMatrixSolver {
 public:
  FundamentalMatrixSolver() {}
  virtual inline ~FundamentalMatrixSolver() {}

  // Add data from one matched image pair.
  virtual inline void AddMatchedImagePair(
      const PairwiseImageMatch& matched_image_data);

  // Add data from a set of matched image pairs.
  virtual inline void AddMachedImagePairs(
      const PairwiseImageMatchList& matched_image_data);

  // Set options.
  virtual inline void SetOptions(const FundamentalMatrixSolverOptions& options);

  // Compute the fundamental matrix for each image pair.
  virtual inline bool ComputeFundamentalMatrices(
      std::vector<Eigen::Matrix3d>& fundamental_matrices);

  // Abstract method to compute the fundamental matrix for a single image pair.
  // Override this in the derived solver class to implement it.
  virtual bool ComputeFundamentalMatrix(
      const FeatureMatchList& matched_features,
      Eigen::Matrix3d& fundamental_matrix) const = 0;

 protected:
  // The matched image data contains a list of matched features for a set of
  // images. The matched features contain (u, v) coordinates in each image. This
  // is a list because we can solve for a bunch of fundamental matrices for a
  // set of independent image matches at once.
  PairwiseImageMatchList matched_image_data_;

  // A set of options used for computing the fundamental matrix.
  FundamentalMatrixSolverOptions options_;

private:
  DISALLOW_COPY_AND_ASSIGN(FundamentalMatrixSolver)

};  //\class FundamentalMatrixSolver


// ------------------- Implementation ------------------- //

// Append two-view image match data to the list of image matches.
void FundamentalMatrixSolver::AddMatchedImagePair(
    const PairwiseImageMatch& matched_image_data) {
  matched_image_data_.push_back(matched_image_data);
}

// Append a set of data from two-view image matchesto the list of image matches.
void FundamentalMatrixSolver::AddMachedImagePairs(
    const PairwiseImageMatchList& matched_image_data) {
  matched_image_data_.insert(matched_image_data_.end(),
                             matched_image_data.begin(),
                             matched_image_data.end());
}

void FundamentalMatrixSolver::SetOptions(
    const FundamentalMatrixSolverOptions& options) {
  options_ = options;
}

bool FundamentalMatrixSolver::ComputeFundamentalMatrices(
    std::vector<Eigen::Matrix3d>& fundamental_matrices) {
  // Clear the output.
  fundamental_matrices.clear();

  // Determine a fundamental matrix for each pair of images.
  for (const auto& pair_data : matched_image_data_) {
    Eigen::Matrix3d fundamental_matrix;
    if (ComputeFundamentalMatrix(pair_data.feature_matches_,
                                 fundamental_matrix)) {
      fundamental_matrices.push_back(fundamental_matrix);
    } else {
      VLOG(1) << "Failed to compute funamental matrix between images "
              << pair_data.image1_index_ << " and " << pair_data.image2_index_
              << ".";
    }
  }

  // Return whether or not we computed any fundamental matrices.
  if (fundamental_matrices.empty()) {
    VLOG(1) << "Unable to compute a fundamental matrix for any of the input "
               "image pairs.";
    return false;
  }

  return true;
}

}  //\namespace bsfm

#endif
