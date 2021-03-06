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
// This struct defines a set of options that are used when solving for the
// fundamental matrix relating the geometry of two cameras.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_GEOMETRY_FUNDAMENTAL_MATRIX_SOLVER_OPTIONS_H
#define BSFM_GEOMETRY_FUNDAMENTAL_MATRIX_SOLVER_OPTIONS_H

namespace bsfm {

struct FundamentalMatrixSolverOptions {
  // Normalize the (u, v) image space positions of features prior to computing
  // the. fundamental matrix. This is pretty much necessary for good results.
  bool normalize_features = true;

  // By definition the fundamental matrix is rank deficient (rank 2 for a 3x3
  // matrix). However, most solution methods come up with answers that are not
  // necessarily rank deficient. Enforcing rank deficiency improves results in
  // most cases, but adds a tiny amount to computation time.
  bool enforce_fundamental_matrix_rank_deficiency = true;

};  //\struct FundamentalMatrixSolverOptions

}  //\namespace bsfm

#endif
