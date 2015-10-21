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
// This file defines methods that can be used to triangulate a point in 3D from
// multiple 2D observations.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_GEOMETRY_TRIANGULATION_H
#define BSFM_GEOMETRY_TRIANGULATION_H

#include "point_3d.h"
#include "../camera/camera.h"
#include "../matching/feature.h"
#include "../matching/feature_match.h"

namespace bsfm {

// Triangulates a single 3D point from > 2 views. The input assumes that
// features correspond to the cameras they were taken from. e.g. features[i] was
// taken from cameras[i]. Returns false if:
// 1) The number of features and cameras are not equal.
// 2) A point cannot be uniquely determined.
// 3) The triangulated point does not reproject into all cameras.
// This uses the inhomogeneous DLT method from H&Z: Multi-View Geometry, Ch 2.2.
bool Triangulate(const FeatureList& features,
                 const std::vector<Camera>& cameras, Point3D& point);

// Triangulate the 3D position of a point from a 2D correspondence and two
// sets of camera extrinsics and intrinsics. Returns false if:
// 1) A point cannot be uniquely determined.
// 2) The triangulated point does not reproject into both cameras.
// This uses the homogeneous DLT method from H&Z: Multi-View Geometry, Ch 2.2.
bool Triangulate(const FeatureMatch& feature_match, const Camera& camera1,
                 const Camera& camera2, Point3D& point);

// Repeats the Triangulate() function on a list of feature matches, returning a
// list of triangulated 3D points. Returns all triangulated points, even if one
// point fails. If triangulating a point fails, that point will be stored as (0,
// 0, 0) in 'points'.
bool Triangulate(const FeatureMatchList& feature_matches, const Camera& camera1,
                 const Camera& camera2, Point3DList& points);

}  //\namespace bsfm

#endif
