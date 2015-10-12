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
// This struct defines a set of options that are used for RANSAC (generic).
// Note that because the Ransac class solves generic RANSAC problems, the
// default values in these options could be very far from a good choice for the
// specific RANSAC problem!
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_MATCHING_RANSAC_OPTIONS_H
#define BSFM_MATCHING_RANSAC_OPTIONS_H

namespace bsfm {

struct RansacOptions {

  // Number of iterations to run RANSAC for.
  unsigned int iterations = 100;

  // In order to be considered an inlier, a data point must fit the RANSAC model
  // to at least this error. This value is extremely arbitrary - tweak it for
  // the specific problem!
  double acceptable_error = 1e-3;

  // The minimum number of inliers needed to consider a model "good". If ANY
  // model is considered "good", then RANSAC has found a solution, and will
  // simply try to find a better one until it has run out of iterations.
  // Again, this value is extremely arbitrary!
  unsigned int minimum_num_inliers = 10;

};  //\struct RansacOptions

}  //\namespace bsfm
#endif
