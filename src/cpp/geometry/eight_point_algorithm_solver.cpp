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

#include "eight_point_algorithm_solver.h"

#include <Eigen/SVD>
#include <glog/logging.h>

#include "normalization.h"

namespace bsfm {

bool EightPointAlgorithmSolver::ComputeFundamentalMatrix(
    const FeatureMatchList& matched_features,
    Matrix3d& fundamental_matrix) const {
  // Following: https://www8.cs.umu.se/kurser/TDBD19/VT05/reconstruct-4.pdf

  // First make sure we even have enough matches to run the eight-point
  // algorithm.
  if (matched_features.size() < 8) {
    VLOG(1) << "Cannot use the eight-point algorithm with less than 8 feature "
               "matches.";
    return false;
  }

  // Build the A matrix from matched features.
  MatrixXd A;
  A.resize(matched_features.size(), 9);

  // If requested, normalize feature positions prior to computing the A matrix.
  Matrix3d T1(MatrixXd::Identity(3, 3));
  Matrix3d T2(MatrixXd::Identity(3, 3));
  if (options_.normalize_features) {
    T1 = ComputeNormalization(matched_features, true /*feature set 1*/);
    T2 = ComputeNormalization(matched_features, false /*feature set 2*/);
  }

  // Incrementally add rows to A.
  for (size_t ii = 0; ii < matched_features.size(); ++ii) {
    double u1 = matched_features[ii].feature1_.u_;
    double v1 = matched_features[ii].feature1_.v_;
    double u2 = matched_features[ii].feature2_.u_;
    double v2 = matched_features[ii].feature2_.v_;

    if (options_.normalize_features) {
      u1 = T1(0, 0) * u1 + T1(0, 2);
      v1 = T1(1, 1) * v1 + T1(1, 2);
      u2 = T2(0, 0) * u2 + T2(0, 2);
      v2 = T2(1, 1) * v2 + T2(1, 2);
    }

    A(ii, 0) = u1*u2;
    A(ii, 1) = v1*u2;
    A(ii, 2) =    u2;
    A(ii, 3) = u1*v2;
    A(ii, 4) = v1*v2;
    A(ii, 5) =    v2;
    A(ii, 6) =    u1;
    A(ii, 7) =    v1;
    A(ii, 8) =     1;
  }

  // Get svd(A). Save some time and compute a thin U. We still need a full V.
  Eigen::JacobiSVD<Eigen::MatrixXd> svd;
  svd.compute(A, Eigen::ComputeThinU | Eigen::ComputeFullV);
  if (!svd.computeV()) {
    VLOG(1) << "Failed to compute a singular value decomposition of A matrix.";
    return false;
  }

  // Get the fundamental matrix elements from the SVD decomposition.
  const VectorXd f_vec = svd.matrixV().col(8);

  // Turn the elements of the fundamental matrix into an actual matrix.
  fundamental_matrix.row(0) = f_vec.topRows(3).transpose();
  fundamental_matrix.row(1) = f_vec.middleRows(3, 3).transpose();
  fundamental_matrix.row(2) = f_vec.bottomRows(3).transpose();

  // If requested, make sure that the computed fundamental matrix has rank 2.
  // This is step 2 of the eight-point algorithm from the slides.
  if (options_.enforce_fundamental_matrix_rank_deficiency) {
    // Get svd(F). We need full U and V to reconstruct the rank deficient F.
    svd.compute(fundamental_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
    svd.compute(fundamental_matrix);
    if (!svd.computeU() || !svd.computeV()) {
      VLOG(1) << "Failed to compute a singular value decomposition of "
                 "fundamental matrix.";
      return false;
    }

    // Build a matrix of the first 8 singular values down the diagonal. Make the
    // last diagonal entry 0.
    MatrixXd S_deficient(MatrixXd::Zero(3, 3));
    S_deficient(0, 0) = svd.singularValues()(0);
    S_deficient(1, 1) = svd.singularValues()(1);
    fundamental_matrix = svd.matrixU() * S_deficient * svd.matrixV().transpose();
  }

  // If normalization was requested, we need to 'un-normalize' the fundamental
  // matrix.
  if (options_.normalize_features) {
    fundamental_matrix = T2.transpose() * fundamental_matrix * T1;
  }

  return true;
}

}  //\namespace bsfm
