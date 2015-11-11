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
// This file defines helpful drawing/debugging utilities for images, feature
// matching, and SFM.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_IMAGE_DRAWING_UTILS_H
#define BSFM_IMAGE_DRAWING_UTILS_H

#include "image.h"
#include "../matching/feature_match.h"
#include "../slam/observation.h"
#include "../util/types.h"

namespace bsfm {
namespace drawing {

// Annotate an image by drawing features on it.
void AnnotateFeatures(const FeatureList& features, Image* image,
                      unsigned int radius = 3,
                      unsigned int line_thickness = 1);

// Draw features as circles in an image.
void DrawImageFeatures(const FeatureList& features, const Image& image,
                       const std::string& window_name = std::string(),
                       unsigned int radius = 3,
                       unsigned int line_thickness = 1);

// Given two images, and data containing their features and matches between
// those features, draw the two images side by side, with matches drawn as lines
// between them.
void DrawImageFeatureMatches(const Image& image1, const Image& image2,
                             const FeatureMatchList& feature_matches,
                             const std::string& window_name = std::string(),
                             unsigned int line_thickness = 1);

// Project landmarks into a view and draw them as squares on the input image. If
// 'print_text_distances' is true, draw the distance from the camera to each
// landmark (up to scale).
void AnnotateLandmarks(ViewIndex view_index, Image* image,
                       unsigned int line_thickness = 2,
                       unsigned int square_width = 10,
                       bool print_text_distances = false);

// Draw landmarks as squares in an image. If 'print_text_distances' is true,
// draw the distance from the camera to each landmark (up to scale).
void DrawLandmarks(ViewIndex view_index, const Image& image,
                   const std::string& window_name = std::string(),
                   unsigned int line_thickness = 2,
                   unsigned int square_width = 10,
                   bool print_text_distances = false);

// Annotate 2D<-->3D matches stored in a set of observations. The view index is
// passed in so that we only draw observations that came from the correct view
// index. This method does not annotate the landmarks and features themselves.
void AnnotateObservations(ViewIndex view_index,
                          const std::vector<Observation::Ptr>& observations,
                          Image* image,
                          unsigned int line_thickness = 2);

// Draw 2D<-->3D matches stored in the set of input observations. See
// description for AnnotateObservations().
void DrawObservations(ViewIndex view_index,
                      const std::vector<Observation::Ptr>& observations,
                      const Image& image,
                      const std::string& window_name = std::string(),
                      unsigned int line_thickness = 2);

}  //\namespace drawing
}  //\namespace bsfm

#endif
