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

#ifndef BSFM_IMAGE_KEYPOINT_DETECTOR_H
#define BSFM_IMAGE_KEYPOINT_DETECTOR_H

#include "../image/image.h"
#include "../util/disallow_copy_and_assign.h"

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

namespace bsfm {

typedef cv::KeyPoint Keypoint;
typedef std::vector<cv::KeyPoint> KeypointList;

class KeypointDetector {
 public:
  KeypointDetector();
  ~KeypointDetector() {}

  // The detector must be set prior to calling DetectKeypoints().
  bool SetDetector(const std::string& detector_type);

  // Detects keypoints in the input image, returning them in the output list.
  bool DetectKeypoints(const Image& image, KeypointList& keypoints_out);

  // Turn on adaptive feature counts. Only works if the detector type is FAST,
  // SURF, or STAR.
  // - min:   minimum desired number of features.
  // - max:   maximum desired number of features.
  // - iters: number of iterations spend adjusting the feature detector
  //          parameters. High numbers are fine for FAST, but time consuming
  //          for SURF and STAR.
  void SetAdaptiveOn(unsigned int min, unsigned int max, unsigned int iters);
  void SetAdaptiveOff();

  // Return whether or not the detector_type supports adaptive feature count
  // adjustment.
  bool SupportsAdaptiveAdjustment() const;

 private:
  DISALLOW_COPY_AND_ASSIGN(KeypointDetector)

  std::string detector_type_;
  cv::Ptr<cv::FeatureDetector> detector_;
  bool adaptive_;

  // Adaptive feature count parameters. See 'SetAdaptiveOn()' for descriptions.
  unsigned int adaptive_min_;
  unsigned int adaptive_max_;
  unsigned int adaptive_iters_;

};  //\class KeypointDetector

}  //\namespace bsfm
#endif
