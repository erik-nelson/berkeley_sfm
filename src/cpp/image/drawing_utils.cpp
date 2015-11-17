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

#include <Eigen/Core>
#include <iomanip>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "drawing_utils.h"
#include "../matching/feature.h"
#include "../math/random_generator.h"
#include "../sfm/view.h"
#include "../slam/landmark.h"

namespace bsfm {
namespace drawing {

using Eigen::Vector3d;

void AnnotateFeatures(const FeatureList& features, Image* image,
                      unsigned int radius, unsigned int line_thickness) {
  CHECK_NOTNULL(image);

  // Create a random number generator for random colors.
  math::RandomGenerator rng(math::RandomGenerator::Seed());

  // Get OpenCV mat from the input image.
  cv::Mat cv_image;
  image->ToCV(cv_image);

  for (const auto& feature : features) {
    cv::Point cv_feature;
    cv_feature.x = feature.u_;
    cv_feature.y = feature.v_;

    const cv::Scalar color(255 * rng.Double(),
                           255 * rng.Double(),
                           255 * rng.Double());
    cv::circle(cv_image, cv_feature, radius, color, line_thickness);
  }

  // Store the OpenCV mat in the image.
  image->FromCV(cv_image);
}

void DrawImageFeatures(const FeatureList& features, const Image& image,
                       const std::string& window_name, unsigned int radius,
                       unsigned int line_thickness) {
  // Annotate features on a copy of the image.
  Image copy(image);
  AnnotateFeatures(features, &copy, radius, line_thickness);

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

    // Color channel values between [0, 255].
    const cv::Scalar color(255 * rng.Double(),
                           255 * rng.Double(),
                           255 * rng.Double());
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

void AnnotateLandmarks(const std::vector<LandmarkIndex>& landmark_indices,
                       const Camera& camera,
                       Image* image,
                       unsigned int line_thickness,
                       unsigned int square_width,
                       bool print_text_distances) {
  CHECK_NOTNULL(image);

  // Create a random number generator for random colors.
  math::RandomGenerator rng(math::RandomGenerator::Seed());

  // Get OpenCV mat from the input image.
  cv::Mat cv_image;
  image->ToCV(cv_image);

  // Landmarks will be colored by distance to camera, so we need to find the max
  // distance first for normalization.
  double max_distance = 0.0;
  std::vector<double> distances;
  for (const auto& landmark_index : landmark_indices) {
    Landmark::Ptr landmark = Landmark::GetLandmark(landmark_index);
    CHECK_NOTNULL(landmark.get());
    const Vector3d p = landmark->Position().Get();
    const Vector3d c = camera.Translation();
    const double d = (p - c).norm();
    if (d > max_distance)
      max_distance = d;

    distances.push_back(d);
  }
  if (distances.size() > 0)
    CHECK(max_distance > 1e-8);

  // Generate colors for each landmark. darker = further.
  // Use 0 distance ~= (255, 255, 0) [light blue].
  //   max distance ~= (100, 100, 0) [dark blue].
  std::vector<cv::Scalar> colors;
  for (size_t ii = 0; ii < distances.size(); ++ii) {
    double intensity = 155.0 * (1.0 - distances[ii] / max_distance) + 100.0;
    const cv::Scalar color(intensity, intensity, 0);
    colors.push_back(color);
  }

  // Iterate over all landmarks that this view can see, annotating them as small
  // rectangles in the image.
  unsigned int color_iter = 0;
  std::vector<cv::Point> text_positions;
  std::vector<double> text_distances;
  for (const auto& landmark_index : landmark_indices) {
    // Already checked nullity.
    Point3D p = Landmark::GetLandmark(landmark_index)->Position();

    // Project the landmark into the view.
    double u = 0.0, v = 0.0;
    if (!camera.WorldToImage(p.X(), p.Y(), p.Z(), &u, &v)) {
      color_iter++;
      continue;
    }

    cv::Point cv_top_left, cv_bot_right;
    cv_top_left.x = u - square_width / 2;
    cv_top_left.y = v - square_width / 2;
    cv_bot_right.x = u + square_width / 2;
    cv_bot_right.y = v + square_width / 2;

    cv::rectangle(cv_image,
                  cv_top_left,
                  cv_bot_right,
                  colors[color_iter],
                  line_thickness);

    // Store the feature location so that we can write text distances on top.
    text_positions.push_back(cv_top_left + cv::Point(-25, -5));
    text_distances.push_back(distances[color_iter]);
    color_iter++;
  }

  // Print distance from the camera to each landmark next to the landmarks.
  // Distance is only up to scale.
  if (print_text_distances) {
    for (size_t ii = 0; ii < text_positions.size(); ++ii) {
      std::ostringstream ss;
      ss << "d: " << std::fixed << std::setprecision(2) << text_distances[ii];
      cv::putText(cv_image, ss.str(), text_positions[ii],
                  cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);
    }
  }

  // Store the OpenCV mat in the image.
  image->FromCV(cv_image);
}

void DrawLandmarks(const std::vector<LandmarkIndex>& landmark_indices,
                   const Camera& camera,
                   const Image& image,
                   const std::string& window_name,
                   unsigned int line_thickness,
                   unsigned int square_width) {
  // Annotate landmarks on a copy of the image.
  Image copy(image);
  AnnotateLandmarks(landmark_indices,
                    camera,
                    &copy,
                    line_thickness,
                    square_width);

  // Convert the copied image to OpenCV format.
  cv::Mat cv_copy;
  copy.ToCV(cv_copy);

  // Draw the copied and annotated image in a named window.
  cv::namedWindow(window_name.c_str(), CV_WINDOW_AUTOSIZE);
  cv::imshow(window_name.c_str(), cv_copy);
  cv::waitKey(0);
}

// Annotate 2D<-->3D matches stored in a set of observations. The view index is
// passed in so that we only draw observations that came from the correct view
// index. If 'only_draw_lines' is true, 2D features and 3D landmarks will not be
// drawn as circles/squares, and only the connecting lines will be drawn.
void AnnotateObservations(ViewIndex view_index,
                          const std::vector<Observation::Ptr>& observations,
                          Image* image, unsigned int line_thickness) {
  CHECK_NOTNULL(image);

  // Get OpenCV mat from the input image.
  cv::Mat cv_image;
  image->ToCV(cv_image);

  // Grab the view and its camera.
  View::Ptr view = View::GetView(view_index);
  CHECK_NOTNULL(view.get());
  const Camera camera = view->Camera();

  for (const auto& observation : observations) {
    // Only annotate observations from this view.
    if (observation->GetViewIndex() != view_index)
      continue;

    // Only annotate observations that are incorporated with landmarks.
    if (!observation->IsIncorporated())
      continue;

    // Get the landmark and project it into the image.
    Landmark::Ptr landmark = observation->GetLandmark();
    CHECK_NOTNULL(landmark.get());
    Point3D point = landmark->Position();

    double ul = 0.0, vl = 0.0;
    if (!camera.WorldToImage(point.X(), point.Y(), point.Z(), &ul, &vl))
      continue;

    const Feature& feature = observation->Feature();
    const double uf = feature.u_;
    const double vf = feature.v_;

    // Draw a green line between the feature and the landmark.
    cv::Point p1, p2;
    p1.x = ul;
    p1.y = vl;
    p2.x = uf;
    p2.y = vf;
    cv::line(cv_image, p1, p2, cv::Scalar(0, 0, 255), line_thickness);
  }

  // Store the OpenCV mat in the image.
  image->FromCV(cv_image);
}

// Draw 2D<-->3D matches stored in the set of input observations. See
// description for AnnotateObservations().
void DrawObservations(ViewIndex view_index,
                      const std::vector<Observation::Ptr>& observations,
                      const Image& image,
                      const std::string& window_name,
                      unsigned int line_thickness) {
  // Annotate observations on a copy of the image.
  Image copy(image);
  AnnotateObservations(view_index, observations, &copy, line_thickness);

  // Convert the copied image to OpenCV format.
  cv::Mat cv_copy;
  copy.ToCV(cv_copy);

  // Draw the copied and annotated image in a named window.
  cv::namedWindow(window_name.c_str(), CV_WINDOW_AUTOSIZE);
  cv::imshow(window_name.c_str(), cv_copy);
  cv::waitKey(0);
}

// Annotate landmark tracks across the specified views. This will iterate over
// all landmarks, attempt to find each landmark in each one of the views,
// annotates a line segment connecting the features corresponding to the
// landmark in each view.
void AnnotateTracks(const std::vector<LandmarkIndex>& landmark_indices,
                    const std::vector<ViewIndex>& view_indices,
                    Image* image,
                    unsigned int line_thickness) {
  CHECK_NOTNULL(image);

  // Get OpenCV mat from the input image.
  cv::Mat cv_image;
  image->ToCV(cv_image);

  for (const auto& landmark_index : landmark_indices) {

    // Track the landmark across the provided view indices.
    std::vector<cv::Point> landmark_track;
    for (const auto& view_index : view_indices) {
      View::Ptr view = View::GetView(view_index);
      CHECK_NOTNULL(view.get());

      for (const auto& observation : view->Observations()) {
        CHECK_NOTNULL(observation.get());

        // If this observation sees our landmark, store the feature position.
        if (observation->IsIncorporated() &&
            observation->GetLandmarkIndex() == landmark_index) {
          cv::Point feature;
          feature.x = observation->Feature().u_;
          feature.y = observation->Feature().v_;
          landmark_track.push_back(feature);
        }
      }
    }

    // Draw the landmark track in the image in green (BGR). Color is based on
    // how long ago the landmark was seen.
    for (size_t ii = 0; ii < landmark_track.size() - 1; ++ii) {
      const int intensity = (((ii+1) * 255) / (landmark_track.size()-1));
      cv::Scalar color(0, 0, intensity);
      cv::line(cv_image,
               landmark_track[ii + 0],
               landmark_track[ii + 1],
               color,
               line_thickness);
    }
  }

  // Store the OpenCV mat in the image.
  image->FromCV(cv_image);
}

} //\namespace drawing
} //\namespace bsfm
