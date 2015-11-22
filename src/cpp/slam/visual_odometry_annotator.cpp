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

#include "visual_odometry_annotator.h"

#include "../image/drawing_utils.h"

namespace bsfm {

VisualOdometryAnnotator::VisualOdometryAnnotator()
    : window_name_("Visual Odometry") {
  image_.reset(new Image());
}

VisualOdometryAnnotator::~VisualOdometryAnnotator() {}

void VisualOdometryAnnotator::SetImage(const Image& image) {
  // Copy the input.
  *image_ = image;
}

void VisualOdometryAnnotator::AnnotateFeatures(const FeatureList& features) {
  const int kRadius = 3;
  const int kLineThickness = 1;
  drawing::AnnotateFeatures(features, image_.get(), kRadius, kLineThickness);
}

void VisualOdometryAnnotator::AnnotateTracks(const std::vector<LandmarkIndex>& tracks) {
  const int kRadius = 3;
  const int kLineThickness = 1;
  drawing::AnnotateTracks(tracks, image_.get(), kRadius, kLineThickness);
}

#if 0
void VisualOdometryAnnotator::AnnotateLandmarks(
    const std::vector<LandmarkIndex>& landmark_indices, const Camera& camera) {
  const int kLineThickness = 1;
  const int kSquareWidth = 8;
  const bool kPrintDistances = true;
  drawing::AnnotateLandmarks(landmark_indices,
                             camera,
                             image_.get(),
                             kLineThickness,
                             kSquareWidth,
                             kPrintDistances);
}

void VisualOdometryAnnotator::AnnotateObservations(
    ViewIndex view_index, const std::vector<Observation::Ptr>& observations) {
  const int kLineThickness = 2;
  drawing::AnnotateObservations(view_index,
                                observations,
                                image_.get(),
                                kLineThickness);
}

void VisualOdometryAnnotator::AnnotateTracks(
    const std::vector<LandmarkIndex>& landmark_indices,
    const std::vector<ViewIndex>& view_indices) {
  const int kLineThickness = 1;
  drawing::AnnotateTracks(landmark_indices,
                          view_indices,
                          image_.get(),
                          kLineThickness);
}
#endif

void VisualOdometryAnnotator::Draw(unsigned int wait_time) {
  image_->ImShow(window_name_, wait_time);
}

void VisualOdometryAnnotator::GetImageCopy(Image* image) const {
  CHECK_NOTNULL(image);
  *image = *image_;
}

}  //\namespace bsfm
