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
// The VisualOdometryAnnotator class is used in conjunction with the
// VisualOdometry class to display features, landmarks, tracks, and matches.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_SLAM_VISUAL_ODOMETRY_ANNOTATOR_H
#define BSFM_SLAM_VISUAL_ODOMETRY_ANNOTATOR_H

#include <memory>
#include <string>
#include <vector>

#include "../image/image.h"
#include "../matching/feature.h"
#include "../slam/observation.h"
#include "../util/disallow_copy_and_assign.h"
#include "../util/types.h"

namespace bsfm {

class VisualOdometryAnnotator {
 public:
  VisualOdometryAnnotator();
  ~VisualOdometryAnnotator();

  // The input image will be copied so that it can be annotated.
  void SetImage(const Image& image);

  // Annotation functions.
  void AnnotateFeatures(const FeatureList& features);
  void AnnotateLandmarks(ViewIndex view_index);
  void AnnotateObservations(ViewIndex view_index,
                            const std::vector<Observation::Ptr>& observations);
  void AnnotateTracks();

  // The draw function will open up an OpenCV window and display the annotated
  // image.
  void Draw(unsigned int wait_time = 0);

 private:
  DISALLOW_COPY_AND_ASSIGN(VisualOdometryAnnotator)

  // A pointer to the image that will be annotated and drawn.
  std::shared_ptr<Image> image_;

  // The name of the OpenCV window for drawing.
  const std::string window_name_;

};  //\class VisualOdometryAnnotator

}  //\namespace bsfm

#endif
