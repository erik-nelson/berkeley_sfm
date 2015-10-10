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

#include "descriptor_extractor.h"

#include <glog/logging.h>

namespace bsfm {

DescriptorExtractor::DescriptorExtractor() {
  cv::initModule_nonfree();
}

bool DescriptorExtractor::SetDescriptor(const std::string& descriptor_type) {
  // Set the descriptor type.
  descriptor_type_ = descriptor_type;
  bool valid_descriptor_type = true;

  // Create an OpenCV descriptor extractor.
  if (descriptor_type.compare("SIFT") == 0) {
    extractor_ = cv::DescriptorExtractor::create("SIFT");
  } else if (descriptor_type_.compare("SURF") == 0) {
    extractor_ = cv::DescriptorExtractor::create("SURF");
  } else if (descriptor_type_.compare("BRIEF") == 0) {
    extractor_ = cv::DescriptorExtractor::create("BRIEF");
  } else if (descriptor_type_.compare("BRISK") == 0) {
    extractor_ = cv::DescriptorExtractor::create("BRISK");
  } else if (descriptor_type_.compare("FREAK") == 0) {
    extractor_ = cv::DescriptorExtractor::create("FREAK");
  } else {
    VLOG(1) << "Descriptor type \"" << descriptor_type
            << "\"is not available. Defaulting to SIFT.";
    extractor_ = cv::DescriptorExtractor::create("SIFT");
    valid_descriptor_type = false;
    descriptor_type_ = "SIFT";
  }

  return valid_descriptor_type;
}

bool DescriptorExtractor::DescribeFeatures(const Image& image,
                                           KeypointList& keypoints,
                                           std::vector<Feature>& features_out) {
  // Make the user has called SetDescriptor().
  if (descriptor_type_.empty()) {
    VLOG(1)
        << "Descriptor has not been specified via SetDescriptor(). Failed to "
           "extract descriptors.";
    return false;
  }

  CHECK(extractor_) << "The descriptor extractor is null.";
  features_out.clear();

  // Convert the input image to OpenCV's format. Note that descriptors must be
  // extracted on the grayscale image, and that the image format must be CV_8U.
  cv::Mat cv_image;
  image.GetCV(cv_image);
  cv_image.convertTo(cv_image, CV_8U, 255);

  // Extract descriptors from the provided keypoints in the image.
  cv::Mat descriptors;
  try {
    extractor_->compute(cv_image, keypoints, descriptors);
  } catch (const std::exception& e) {
    VLOG(1) << "Failed to extract descriptors: " << e.what();
    return false;
  }

  // Convert the computed descriptors and keypoints into a list of features.
  for (size_t ii = 0; ii < keypoints.size(); ++ii) {
    Feature feature;
    feature.u_ = keypoints[ii].pt.x;
    feature.v_ = keypoints[ii].pt.y;
    OpenCVToEigenVec(descriptors.row(ii), feature.descriptor_);
    features_out.push_back(feature);
  }

  return true;
}

}  //\namespace bsfm
