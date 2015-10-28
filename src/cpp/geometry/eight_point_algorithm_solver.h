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

#ifndef BSFM_GEOMETRY_EIGHT_POINT_ALGORITHM_SOLVER_H
#define BSFM_GEOMETRY_EIGHT_POINT_ALGORITHM_SOLVER_H

#include <Eigen/Core>

#include "fundamental_matrix_solver.h"
#include "../matching/feature_match.h"
#include "../util/disallow_copy_and_assign.h"

namespace bsfm {

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class EightPointAlgorithmSolver : public FundamentalMatrixSolver {
 public:
  EightPointAlgorithmSolver() { }
  virtual ~EightPointAlgorithmSolver() { }

  // Use the eight point algorithm to compute the fundamental matrix for a set
  // of features matched between two images.
  virtual bool ComputeFundamentalMatrix(
      const FeatureMatchList& matched_features,
      Matrix3d& fundamental_matrix) const;

 private:
  DISALLOW_COPY_AND_ASSIGN(EightPointAlgorithmSolver)

};  //\class EightPointAlgorithmSolver

}  //\namespace bsfm

#endif
