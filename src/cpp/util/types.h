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
// This file defines typedefs for generic types and primitives used for SfM.
//
// ////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_UTIL_TYPES_H
#define BSFM_UTIL_TYPES_H

#include <Eigen/Core>
#include <opencv2/features2d/features2d.hpp>
#include <vector>

namespace bsfm {

// -------------------- Custom types -------------------- //

// Each ViewIndex corresponds to a unique View that is constructed at runtime
// and can be accessed with View::GetView(ViewIndex index).
typedef unsigned int ViewIndex;



// -------------------- Third-party typedefs -------------------- //

// When extracting N-dimensional descriptors from a set of M keypoints, OpenCV
// will store the desriptors in a (M-K)xN matrix, where K is the number of
// keypoints that OpenCV failed to compute a descriptor for. In other words,
// rows correspond to keypoints, and columns to indices of the descriptor.
typedef ::cv::Mat DescriptorList;

// Keypoints contain (u, v) image-space coordinates.
typedef ::cv::KeyPoint Keypoint;
typedef std::vector<Keypoint> KeypointList;

// Used to represent [R | t] and P, two matrices commonly used to represent
// camera extrinsics and projections.
typedef ::Eigen::Matrix<double, 3, 4> Matrix34d;

}  //\namespace bsfm

#endif
