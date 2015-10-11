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
// This class implements the eight-point algorithm, which determines the
// fundamental matrix for a pair of cameras from a set of matched features in a
// two-view image pair. The implementation follows slide 2 here:
//
// https://www8.cs.umu.se/kurser/TDBD19/VT05/reconstruct-4.pdf
//
// Or alternatively, pages 281-282 of Hartley and Zisserman, Multi-View Geometry
// in Computer Vision.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_GEOMETRY_EIGHT_POINT_ALGORITHM_H
#define BSFM_GEOMETRY_EIGHT_POINT_ALGORITHM_H

#include <glog/logging.h>
#include <Eigen/Core>
#include <Eigen/SVD>

#include "fundamental_matrix_solver.h"
#include "../matching/feature_match.h"
#include "../util/disallow_copy_and_assign.h"

namespace bsfm {

class EightPointAlgorithmSolver : public FundamentalMatrixSolver {
 public:
  EightPointAlgorithmSolver() { }
  virtual ~EightPointAlgorithmSolver() { }

 private:
  DISALLOW_COPY_AND_ASSIGN(EightPointAlgorithmSolver)

  // Use the eight point algorithm to compute the fundamental matrix for a set
  // of features matched between two images.
  bool ComputeFundamentalMatrix(const FeatureMatchList& matched_features,
                                Eigen::Matrix3d& fundamental_matrix);

  // Determine a normalization matrix for the specified set of features. Since
  // both sets of features are stored in the FeatureMatchList, the 'use_feature_set1'
  // must be specified to pick out a normalization for either feature set 1 or
  // feature set 2.
  Eigen::Matrix3d ComputeNormalization(const FeatureMatchList& matched_features,
                                       bool use_feature_set1);

};  //\class EightPointAlgorithmSolver


// ------------------- Implementation ------------------- //

bool EightPointAlgorithmSolver::ComputeFundamentalMatrix(
    const FeatureMatchList& matched_features,
    Eigen::Matrix3d& fundamental_matrix) {
  // Following: https://www8.cs.umu.se/kurser/TDBD19/VT05/reconstruct-4.pdf

  // Build the A matrix from matched features.
  Eigen::MatrixXd A;
  A.resize(matched_features.size(), 9);

  // If requested, normalize feature positions prior to computing the A matrix.
  Eigen::Matrix3d T1(Eigen::MatrixXd::Identity(3, 3));
  Eigen::Matrix3d T2(Eigen::MatrixXd::Identity(3, 3));
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
      u1 = T1(0, 0) * u1 - T1(0, 2);
      v1 = T1(1, 1) * v1 - T1(1, 2);
      u2 = T2(0, 0) * u2 - T2(0, 2);
      v2 = T2(1, 1) * v2 - T2(1, 2);
    }

    A(ii, 0) = u1*u2;
    A(ii, 1) = u1*v2;
    A(ii, 2) = u1   ;
    A(ii, 3) = v1*u2;
    A(ii, 4) = v1*v2;
    A(ii, 5) = v1   ;
    A(ii, 6) = u2   ;
    A(ii, 7) = v2   ;
    A(ii, 8) = 1    ;
  }

  // Get svd(A).
  Eigen::JacobiSVD<Eigen::MatrixXd> svd;
  svd.compute(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  if (!svd.computeU() || !svd.computeV()) {
    VLOG(1) << "Failed to compute a singular value decomposition of A matrix.";
    return false;
  }

  // Get the fundamental matrix elements from the SVD decomposition.
  const Eigen::MatrixXd V_t = svd.matrixV().transpose();
  const Eigen::VectorXd f_vec = V_t.rightCols(1);

  // Turn the elements of the fundamental matrix into an actual matrix.
  fundamental_matrix.row(0) = f_vec.topRows(3).transpose();
  fundamental_matrix.row(1) = f_vec.middleRows(3, 3).transpose();
  fundamental_matrix.row(2) = f_vec.bottomRows(3).transpose();

  // If requested, make sure that the computed fundamental matrix has rank 2.
  // This is step 2 of the eight-point algorithm from the slides.
  if (options_.enforce_fundamental_matrix_rank_deficiency) {
    Eigen::Matrix3d S_deficient(Eigen::Matrix3d::Zero());
    S_deficient(0, 0) = svd.singularValues()(0);
    S_deficient(1, 1) = svd.singularValues()(1);
    fundamental_matrix = svd.matrixU() * S_deficient * V_t;
  }

  // If normalization was requested, we need to 'un-normalize' the fundamental
  // matrix.
  if (options_.normalize_features) {
    fundamental_matrix = T1.transpose() * fundamental_matrix * T2;
  }

  return true;
}

Eigen::Matrix3d EightPointAlgorithmSolver::ComputeNormalization(
    const FeatureMatchList& matched_features, bool use_feature_set1) {
  // Compute a mean translation from the origin.
  double mean_u = 0.0;
  double mean_v = 0.0;
  for (size_t ii = 0; ii < matched_features.size(); ++ii) {
    if (use_feature_set1) {
      mean_u += matched_features[ii].feature1_.u_;
      mean_v += matched_features[ii].feature1_.v_;
    } else {
      mean_u += matched_features[ii].feature2_.u_;
      mean_v += matched_features[ii].feature2_.v_;
    }
  }
  mean_u /= static_cast<double>(matched_features.size());
  mean_v /= static_cast<double>(matched_features.size());

  // Compute a scale factor such that after translation, all points will be an
  // average distance of sqrt(2) away from the origin.
  // This uses Eqs. 1.28 - 1.37 from here:
  // http://www.ecse.rpi.edu/Homepages/qji/CV/8point.pdf
  double scale = 0.0;
  double du = 0.0, dv = 0.0;
  for (size_t ii = 0; ii < matched_features.size(); ++ii) {
    double u = 0.0, v = 0.0;
    if (use_feature_set1) {
      u = matched_features[ii].feature1_.u_;
      v = matched_features[ii].feature1_.v_;
    } else {
      u = matched_features[ii].feature2_.u_;
      v = matched_features[ii].feature2_.v_;
    }

    scale += sqrt(((u - mean_u)*(u - mean_u) + (v - mean_v)*(v-mean_v)) / 2.0);
    du += u - mean_u;
    dv += v - mean_v;
  }
  scale /= static_cast<double>(matched_features.size());
  du /= static_cast<double>(matched_features.size());
  dv /= static_cast<double>(matched_features.size());

  // Populate the output matrix.
  Eigen::Matrix3d T(Eigen::MatrixXd::Identity(3, 3));
  T(0, 0) = 1.0 / du;
  T(1, 1) = 1.0 / dv;
  T(0, 2) = -mean_u / du;
  T(1, 2) = -mean_v / dv;

  return T;
}

}  //\namespace bsfm

#endif
