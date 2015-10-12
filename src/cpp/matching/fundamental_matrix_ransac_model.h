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
// This class defines the FundamentalMatrixRansacModel class API, which 
// is derived from the abstract base class RansacModel.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_MATCHING_FUNDAMENTAL_MATRIX_RANSAC_MODEL_H
#define BSFM_MATCHING_FUNDAMENTAL_MATRIX_RANSAC_MODEL_H

#include "ransac_problem.h"
#include "fundamental_matrix_ransac_data_element.h"
#include <Eigen/Dense>

namespace bsfm {

class FundamentalMatrixRansacModel : public RansacModel {
 private:
  Eigen::Matrix3d F_;
  double error_;

 public:
    
  // Constructor.
  FundamentalMatrixRansacModel(Eigen::Matrix3d& F); 

  // Destructor.
  ~FundamentalMatrixRansacModel();
    
  // Return model error.
  double Error();

  // Evaluate model on a single data element and update error.
  bool IsGoodFit(FundamentalMatrixRansacDataElement, double error_tolerance);

  }

} //\namespace bsfm

#endif
