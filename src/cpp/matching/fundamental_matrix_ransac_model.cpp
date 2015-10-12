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

#include "fundamental_matrix_ransac_model.h"

namespace bsfm {

  // Constructor.
  FundamentalMatrixRansacModel::FundamentalMatrixRansacModel(Eigen::Matrix3d& F) 
    : F_(F), error_(0.0) {}

  // Destructor.
  ~FundamentalMatrixRansacModel::FundamentalMatrixRansacDataElement() {}
    
  // Return model error.
  double FundamentalMatrixRansacModel::Error() { return error_; }

  // Evaluate model on a single data element and update error.
  bool FundamentalMatrixRansacModel::IsGoodFit(FundamentalMatrixRansacDataElement match, 
					       double error_tolerance) {
    // Construct vectors for 2D points in match.
    Eigen::Vector3d kp1, kp2;
    kp1 << match.feature1_.u_, match.feature1_.v_, 1;
    kp2 << match.feature2_.u_, match.feature2_.v_, 1;

    // Compute error and record its square.
    double epipolar_condition = kp1.transpose() * F_ * kp2;
    error_ += epipolar_condition * epipolar_condition;

    // Test against the provided tolerance.
    if (error_ < error_tolerance)
      return true;
    return false;
  }

} //\namespace bsfm

#endif
