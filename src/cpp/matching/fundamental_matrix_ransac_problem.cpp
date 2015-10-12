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

#include "fundamental_matrix_ransac_problem.h"
#include <math/random_generator.h>
#include <geometry/fundamental_matrix_solver.h>
#include <geometry/eight_point_algorithm_solver.h>

#include <gtest/gtest.h>

#define SUBSAMPLE_SIZE 8

//DEFINE_INT(SUBSAMPLE_SIZE, 8,
//	   "Number of points required for each minimal sampling in RANSAC.");

namespace bsfm {
    
  // Constructor.
  FundamentalMatrixRansacProblem::FundamentalMatrixRansacProblem
  (std::vector<FundamentalMatrixRansacDataElement>& data) 
    : data_(data), 
      solution_found_(false) { 
    model_ = NullModel();
  }

  // Destructor.
  ~FundamentalMatrixRansacProblem::FundamentalMatrixRansacProblem();
    
  // Subsample the data.
  std::vector<FundamentalMatrixRansacDataElement> 
  FundamentalMatrixRansacProblem::SampleData() {

    // Randomly shuffle the entire dataset and take the first 8 elements.
    std::random_shuffle(data_.begin(), data_.end(), math::IntegerUniform);

    std::vector<FundamentalMatrixRansacDataElement> samples = 
      std::vector<FundamentalMatrixRansacDataElement>();
    for (int i = 0; i < SUBSAMPLE_SIZE; i++)
      samples.push_back(data_[i]);

    return samples;
  }

  // Return the entire dataset.
  std::vector<FundamentalMatrixRansacDataElement> 
  FundamentalMatrixRansacProblem::UnsampledData() { return data_; }

  // Fit a model to the provided data using the 8-point algorithm.
  FundamentalMatrixRansacModel FundamentalMatrixRansacProblem::FitModel
  (const std::vector<FundamentalMatrixRansacDataElement>& data) {
    
    // Create an empty fundamental matrix.
    Eigen::Matrix3d F = Eigen::Matrix3d()

    // Run the 8-point algorithm.
    FundamentalMatrixSolver solver;
    if (solver.ComputeFundamentalMatrix(data, F))
      return FundamentalMatrixRansacModel(F);
    return FundamentalMatrixRansacModel();
  }

  }

} //\namespace bsfm

#endif
