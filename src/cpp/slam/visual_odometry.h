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
// The VisualOdometry class runs visual odometry from incremental image inputs.
// As images are input, the system extracts features and descriptors, matches
// them against other images or against existing 3D points, and computes the 3D
// pose of the camera. At any time, the class can be queried for the 3D pose of
// the camera as well as the positions of all known landmarks.
//
// The VisualOdometry class handles initialization by itself using the first few
// images.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_SLAM_VISUAL_ODOMETRY_H
#define BSFM_SLAM_VISUAL_ODOMETRY_H

#include "../image/image.h"
#include "../util/disallow_copy_and_assign.h"
#include "../util/status.h"
#include "../util/types.h"

namespace bsfm {

class VisualOdometry {
 public:
  VisualOdometry();
  ~VisualOdometry();

  // Update the estimate of the camera's position and all landmark positions.
  Status Update(const Image& image);

 private:
  DISALLOW_COPY_AND_ASSIGN(VisualOdometry)

  // Specifies whether visual odometry is initialized. This happens the
  // first time that two images are matched and 3D points are triangulated.
  bool is_initialized_;

  // A list of all views that have been added via visual odometry. Iterating
  // through these will generate the camera's trajectory.
  std::vector<ViewIndex> view_indices_;

};  //\class VisualOdometry

}  //\namespace bsfm

#endif
