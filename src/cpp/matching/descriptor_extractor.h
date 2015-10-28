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

#ifndef BSFM_IMAGE_DESCRIPTOR_EXTRACTOR_H
#define BSFM_IMAGE_DESCRIPTOR_EXTRACTOR_H

#include <Eigen/Core>
#include <glog/logging.h>
#include <opencv2/features2d/features2d.hpp>

#include "feature.h"
#include "keypoint_detector.h"

#include "../image/image.h"
#include "../util/disallow_copy_and_assign.h"
#include "../util/types.h"

namespace bsfm {

using Eigen::MatrixXd;

class DescriptorExtractor {
 public:

  DescriptorExtractor() { }
  ~DescriptorExtractor() { }

  // The descriptor type must be set prior to calling ExtractDescriptors().
  bool SetDescriptor(const std::string& descriptor_type);

  // Creates a list of features by extracting descriptors for each keypoint and
  // associating the two together into an object.
  bool DescribeFeatures(const Image& image, KeypointList& keypoints,
                        std::vector<Feature>& features_out,
                        std::vector<Descriptor>& descriptors_out);

 private:
  DISALLOW_COPY_AND_ASSIGN(DescriptorExtractor)

  std::string descriptor_type_;
  cv::Ptr<cv::DescriptorExtractor> extractor_;
};  //\class DescriptorExtractor

}  //\namespace bsfm
#endif
