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
// This file defines methods that can be used to compute the reprojection error
// from multiple keypoint matches.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_GEOMETRY_REPROJECTION_ERROR_H
#define BSFM_GEOMETRY_REPROJECTION_ERROR_H

#include "point_3d.h"
#include "../camera/camera.h"
#include "../matching/feature.h"
#include "../slam/observation.h"

namespace bsfm {

// Evaluate reprojection error on the given Feature and Point3D pair.
double ReprojectionError(const Feature& feature, const Point3D& point,
                         const Camera& camera);

// Repeats the ReprojectionError() function on a list of feature point pairs,
// returning the mean reprojection error.
double ReprojectionError(const FeatureList& features, const Point3DList& points,
                         const Camera& camera);

// Evaluate the reprojection error on the given Observation.
double ReprojectionError(const Observation::Ptr& observation,
                         const Camera& camera);

// Repeats the ReprojectionError() function on a list of obserations, returning
// the mean reprojection error.
double ReprojectionError(const std::vector<Observation::Ptr>& observations,
                         const Camera& camera);

}  //\namespace bsfm

#endif
