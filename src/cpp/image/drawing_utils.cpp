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

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "drawing_utils.h"
#include "../matching/feature.h"
#include "../math/random_generator.h"

namespace bsfm {
namespace drawing {

void AnnotateFeatures(const FeatureList& features, Image& image,
                      unsigned int radius, unsigned int line_thickness) {
  // Create a random number generator for random colors.
  math::RandomGenerator rng(math::RandomGenerator::Seed());

  // Get OpenCV mat from the input image.
  cv::Mat cv_image;
  image.ToCV(cv_image);

  for (const auto& feature : features) {
    cv::Point cv_feature;
    cv_feature.x = feature.u_;
    cv_feature.y = feature.v_;

    const cv::Scalar color(rng.Double(), rng.Double(), rng.Double());
    cv::circle(cv_image, cv_feature, radius, color, line_thickness);
  }

  // Store the OpenCV mat in the image.
  image.FromCV(cv_image);
}

void DrawImageFeatures(const FeatureList& features, const Image& image,
                       const std::string& window_name, unsigned int radius,
                       unsigned int line_thickness) {
  // Annotate features on a copy of the image.
  Image copy(image);
  AnnotateFeatures(features, copy, radius, line_thickness);

  // Convert the copied image to OpenCV format.
  cv::Mat cv_copy;
  copy.ToCV(cv_copy);

  // Draw the copied and annotated image in a named window.
  cv::namedWindow(window_name.c_str(), CV_WINDOW_AUTOSIZE);
  cv::imshow(window_name.c_str(), cv_copy);
  cv::waitKey(0);
}

void DrawImageFeatureMatches(const Image& image1, const Image& image2,
                             const FeatureMatchList& feature_matches,
                             const std::string& window_name,
                             unsigned int line_thickness) {
  // Create a random number generator for random colors.
  math::RandomGenerator rng(math::RandomGenerator::Seed());

  // Get OpenCV mats from the input images.
  cv::Mat cv_image1, cv_image2;
  image1.ToCV(cv_image1);
  image2.ToCV(cv_image2);

  // Create an OpenCV mat of the two images side by side.
  cv::Mat combined_image;
  cv::hconcat(cv_image1, cv_image2, combined_image);

  // Draw lines between feature locations in the combined image.
  for (const auto& feature_match : feature_matches) {
    cv::Point feature1;
    feature1.x = feature_match.feature1_.u_;
    feature1.y = feature_match.feature1_.v_;

    cv::Point feature2;
    feature2.x = feature_match.feature2_.u_;
    feature2.y = feature_match.feature2_.v_;
    feature2 += cv::Point(image2.Width(), 0);

    // Color channel values between [0, 1).
    const cv::Scalar color(rng.Double(), rng.Double(), rng.Double());
    cv::line(combined_image, feature1, feature2, color, line_thickness);

    // Draw circles around each feature.
    cv::circle(combined_image, feature1, line_thickness * 3 /*radius*/, color,
               line_thickness * 2);
    cv::circle(combined_image, feature2, line_thickness * 3 /*radius*/, color,
               line_thickness * 2);
  }

  // Draw the new image in a named window.
  cv::namedWindow(window_name.c_str(), CV_WINDOW_AUTOSIZE);
  cv::imshow(window_name.c_str(), combined_image);
  cv::waitKey(0);
}

} //\namespace drawing
} //\namespace bsfm
