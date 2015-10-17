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
// This header defines a set of functions for converting a fundamental matrix
// and a pair of camera intrinsics into an essential matrix, and from an
// essential matrix to and a set of camera extrinsics.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_ESSENTIAL_MATRIX_SOLVER_H
#define BSFM_ESSENTIAL_MATRIX_SOLVER_H

#include <Eigen/Core>
#include <glog/logging.h>
#include <vector>

#include <camera/camera_intrinsics.h>
#include <camera/camera_extrinsics.h>

namespace bsfm {

class EssentialMatrixSolver {
public:
  // Empty constructor and destructor. No member variables.
  EssentialMatrixSolver() {}
  ~EssentialMatrixSolver() {}
  
  // Compute the essential matrix from a fundamental matrix and camera intrinsics.
  Eigen::Matrix3d ComputeEssentialMatrix(Eigen::Matrix3d F,
					 CameraIntrinsics K1,
					 CameraIntrinsics K2);
  
  // Compute camera extrinsics from an essential matrix and a list of keypoint matches.
  bool ComputeExtrinsics(CameraExtrinsics& extrinsics,
				     Eigen::Matrix3d E, 
				     PairwiseImageMatchList matches);
    
private:
  DISALLOW_COPY_AND_ASSIGN(EssentialMatrixSolver)

};  //\class EssentialMatrixSolver


// ------------------- Implementation ------------------- //

// Compute the essential matrix from a fundamental matrix and camera intrinsics.
Eigen::Matrix3d EssentialMatrixSolver::ComputeEssentialMatrix(Eigen::Matrix3d F,
							      CameraIntrinsics cam1,
							      CameraIntrinsics cam2) {
  // Extract intrinsics matrices.
  Eigen::Matrix3d K1, K2;
  K1 = cam1.IntrinsicsMatrix();
  K2 = cam2.IntrinsicsMatrix();

  // Calculate the essential matrix.
  Eigen::Matrix3d E = K2.transpose() * F * K1;
  return E;
}
 
// Compute camera extrinsics from an essential matrix and a list of keypoint matches.
// NOTE: this implementation is based on Hartley & Zisserman's MVG, pg. 258.
bool EssentialMatrixSolver::ComputeExtrinsics(CameraExtrinsics& extrinsics;
					      Eigen::Matrix3d E, 
					      PairwiseImageMatchList matches) {
  // Initialize the W matrix.
  Eigen::Matrix3d W;
  W << 
    0.0, -1.0, 0.0, 
    1.0, 0.0, 0.0, 
    0.0, 0.0, 1.0;

  // Perform an SVD on the essential matrix.
  Eigen::JacobiSVD<Eigen::Matrix3d> svd;
  svd.compute(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
  if (!svd.computeV()) {
    VLOG(1) << "Failed to compute a singular value decomposition of the essential matrix.";
    return false;
  }
  
  // TODO: finish this!! Refer to SimpleSFM/BasicFunctions.E2Rt().

  return true;
}

}  //\namespace bsfm

#endif
