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

#ifndef BSFM_GEOMETRY_FUNDAMENTAL_MATRIX_SOLVER_H
#define BSFM_GEOMETRY_FUNDAMENTAL_MATRIX_SOLVER_H

#include <Eigen/Core>
#include <vector>

#include "fundamental_matrix_solver_options.h"
#include "../matching/pairwise_image_match.h"
#include "../util/disallow_copy_and_assign.h"

namespace bsfm {

using Eigen::Matrix3d;

class FundamentalMatrixSolver {
 public:
  FundamentalMatrixSolver() {}
  virtual ~FundamentalMatrixSolver() {}

  // Add data from one matched image pair.
  virtual void AddMatchedImagePair(
      const PairwiseImageMatch& matched_image_data);

  // Add data from a set of matched image pairs.
  virtual void AddMatchedImagePairs(
      const PairwiseImageMatchList& matched_image_data);

  // Set options.
  virtual void SetOptions(const FundamentalMatrixSolverOptions& options);

  // Compute the fundamental matrix for each image pair.
  virtual bool ComputeFundamentalMatrices(
      std::vector<Matrix3d>& fundamental_matrices);

  // Abstract method to compute the fundamental matrix for a single image pair.
  // Override this in the derived solver class to implement it.
  virtual bool ComputeFundamentalMatrix(
      const FeatureMatchList& matched_features,
      Matrix3d& fundamental_matrix) const = 0;

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

}  //\namespace bsfm

#endif
