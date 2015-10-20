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
// This file defines functions used for feature normalization. In many geometry
// problems, 2D image coordinates require normalization (i.e. zero-mean
// coordinates, with an average distance of sqrt(2) from the origin). This is
// because the homogeneous coordinate is typically chosen as 1, and most cameras
// have resolutions ~ O(10^3). This is a big difference, leading to numerical
// precision errors in many algorithm implementations.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_GEOMETRY_NORMALIZATION_H
#define BSFM_GEOMETRY_NORMALIZATION_H

#include <Eigen/Core>
#include "point_3d.h"
#include "../matching/feature.h"
#include "../matching/feature_match.h"

namespace bsfm {

// Compute a matrix that when used to left-multiply the input features will
// normalize them. Since both sets of features are stored in the
// FeatureMatchList, the 'use_feature_set1' parameter must be specified to
// pick out a normalization for either feature set 1 or feature set 2.
Eigen::Matrix3d ComputeNormalization(const FeatureMatchList& matched_features,
                                     bool use_feature_set1);

// Compute a matrix that when used to left-multiply the input features will
// normalize them.
Eigen::Matrix3d ComputeNormalization(const FeatureList& features);

// Compute a matrix that when used to left-multiply the input points will
// normalize them.
Eigen::Matrix4d ComputeNormalization(const Point3DList& points);

}  //\namespace bsfm

#endif
