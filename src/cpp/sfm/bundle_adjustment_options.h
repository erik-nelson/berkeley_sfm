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
// This struct defines a set of options that are used during bundle adjustment.
// Most of the options define the behavior of the non-linear least-squares
// optimization, which is performed by Ceres. More details on these options can
// be found at: http://ceres-solver.org/nnls_solving.html#solver-options
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_SFM_BUNDLE_ADJUSTMENT_OPTIONS_H
#define BSFM_SFM_BUNDLE_ADJUSTMENT_OPTIONS_H

#include <string>

namespace bsfm {

struct BundleAdjustmentOptions {
  // Set ceres solver type. DENSE_SCHUR and SPARSE_SCHUR are good solvers for
  // bundle adjustment problems, where DENSE_SCHUR will be faster for small-ish
  // problems, and SPARSE_SCHUR will be faster for larger problems (i.e. >1000
  // cameras, >1000 points). Valid options (as well as benchmark time using 100
  // points and 100 cameras) are:
  // - DENSE_QR               - 19.011 seconds
  // - DENSE_NORMAL_CHOLESKY  -  1.587 seconds
  // - SPARSE_NORMAL_CHOLESKY -  0.044 seconds
  // - CGNR                   -  0.028 seconds
  // - DENSE_SCHUR            -  0.050 seconds
  // - SPARSE_SCHUR           -  0.110 seconds
  // - ITERATIVE_SCHUR        -  0.026 seconds
  std::string solver_type = "DENSE_SCHUR";

  // Print the full ceres report after finishing bundle adjustment.
  bool print_summary = false;

  // Print optimization progress as ceres is solving.
  bool print_progress = false;

  // Maximum number of steps that the optimizer will take.
  unsigned int max_num_iterations = 50;

  // The solver will terminate if | delta cost | / cost < function_tolerance.
  double function_tolerance = 1e-16;

  // The solver will terminate if the computed gradient is less than this.
  double gradient_tolerance = 1e-16;

  // TODO(eanelson): Add option to optimize camera parameters, e.g. focal
  // length, skew, aspect ratio, radial distortion, principal point.

};  //\struct BundleAdjustmentOptions

}  //\namespace bsfm

#endif
