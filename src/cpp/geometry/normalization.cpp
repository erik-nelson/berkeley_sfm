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

#include "normalization.h"

#include <glog/logging.h>

namespace bsfm {

Eigen::Matrix3d ComputeNormalization(const FeatureMatchList& matched_features,
                                     bool use_feature_set1) {
  if (matched_features.empty()) {
    LOG(WARNING) << "Received no data to normalize. Returning identity.";
    return Eigen::MatrixXd::Identity(3, 3);
  }

  // Compute a mean translation from the origin.
  double mean_u = 0.0;
  double mean_v = 0.0;
  for (size_t ii = 0; ii < matched_features.size(); ++ii) {
    if (use_feature_set1) {
      mean_u += matched_features[ii].feature1_.u_;
      mean_v += matched_features[ii].feature1_.v_;
    } else {
      mean_u += matched_features[ii].feature2_.u_;
      mean_v += matched_features[ii].feature2_.v_;
    }
  }
  mean_u /= static_cast<double>(matched_features.size());
  mean_v /= static_cast<double>(matched_features.size());

  // Compute a scale factor such that after translation, all points will be an
  // average distance of sqrt(2) away from the origin.
  // This uses Eqs. 1.28 - 1.37 from here:
  // http://www.ecse.rpi.edu/Homepages/qji/CV/8point.pdf
  double scale = 0.0;
  for (size_t ii = 0; ii < matched_features.size(); ++ii) {
    double u = 0.0, v = 0.0;
    if (use_feature_set1) {
      u = matched_features[ii].feature1_.u_;
      v = matched_features[ii].feature1_.v_;
    } else {
      u = matched_features[ii].feature2_.u_;
      v = matched_features[ii].feature2_.v_;
    }

    scale += sqrt((u - mean_u) * (u - mean_u) + (v - mean_v) * (v - mean_v));
  }
  scale /= static_cast<double>(matched_features.size());
  scale /= sqrt(2.0);

  // If the calculated scale, for some reason, is ridiculously small (i.e.
  // dividing by zero), return an identity output matrix. This will make us not
  // normalize the data, but we should still get an okay fundamental matrix.
  if (scale < 1e-4) {
    LOG(WARNING)
        << "When normalizing feature coordinates scale factor was nearly zero. "
           "Proceeding without normalizing data.";
    return Eigen::MatrixXd::Identity(3, 3);
  }

  // Populate the output matrix.
  Eigen::Matrix3d T(Eigen::MatrixXd::Identity(3, 3));
  T(0, 0) = 1.0 / scale;
  T(1, 1) = 1.0 / scale;
  T(0, 2) = -mean_u / scale;
  T(1, 2) = -mean_v / scale;

  return T;
}

Eigen::Matrix3d ComputeNormalization(const FeatureList& features) {
  if (features.empty()) {
    LOG(WARNING) << "Received no data to normalize. Returning identity.";
    return Eigen::MatrixXd::Identity(3, 3);
  }

  // Compute a mean translation from the origin.
  double mean_u = 0.0;
  double mean_v = 0.0;
  for (size_t ii = 0; ii < features.size(); ++ii) {
    mean_u += features[ii].u_;
    mean_v += features[ii].v_;
  }
  mean_u /= static_cast<double>(features.size());
  mean_v /= static_cast<double>(features.size());

  // Compute a scale factor such that after translation, all points will be an
  // average distance of sqrt(2) away from the origin.
  // This uses Eqs. 1.28 - 1.37 from here:
  // http://www.ecse.rpi.edu/Homepages/qji/CV/8point.pdf
  double scale = 0.0;
  for (size_t ii = 0; ii < features.size(); ++ii) {
    double u = 0.0, v = 0.0;
    u = features[ii].u_;
    v = features[ii].v_;

    scale += sqrt((u - mean_u) * (u - mean_u) + (v - mean_v) * (v - mean_v));
  }
  scale /= static_cast<double>(features.size());
  scale /= sqrt(2.0);

  // If the calculated scale, for some reason, is ridiculously small (i.e.
  // dividing by zero), return an identity output matrix. This will make us not
  // normalize the data, but we should still get an okay fundamental matrix.
  if (scale < 1e-4) {
    LOG(WARNING)
        << "When normalizing feature coordinates scale factor was nearly zero. "
           "Proceeding without normalizing data.";
    return Eigen::MatrixXd::Identity(3, 3);
  }

  // Populate the output matrix.
  Eigen::Matrix3d T(Eigen::MatrixXd::Identity(3, 3));
  T(0, 0) = 1.0 / scale;
  T(1, 1) = 1.0 / scale;
  T(0, 2) = -mean_u / scale;
  T(1, 2) = -mean_v / scale;

  return T;
}

Eigen::Matrix4d ComputeNormalization(const Point3DList& points) {
  if (points.empty()) {
    LOG(WARNING) << "Received no data to normalize. Returning identity.";
    return Eigen::MatrixXd::Identity(4, 4);
  }

  // Compute a mean translation from the origin.
  double mean_x = 0.0;
  double mean_y = 0.0;
  double mean_z = 0.0;
  for (size_t ii = 0; ii < points.size(); ++ii) {
    mean_x += points[ii].X();
    mean_y += points[ii].Y();
    mean_z += points[ii].Z();
  }
  mean_x /= static_cast<double>(points.size());
  mean_y /= static_cast<double>(points.size());
  mean_z /= static_cast<double>(points.size());

  // Compute a scale factor such that after translation, all points will be an
  // average distance of sqrt(2) away from the origin.
  // This uses Eqs. 1.28 - 1.37 from here:
  // http://www.ecse.rpi.edu/Homepages/qji/CV/8point.pdf
  double scale = 0.0;
  for (size_t ii = 0; ii < points.size(); ++ii) {
    double x = 0.0, y = 0.0, z = 0.0;
    x = points[ii].X();
    y = points[ii].Y();
    z = points[ii].Z();

    scale += sqrt((x - mean_x) * (x - mean_x)
                + (y - mean_y) * (y - mean_y)
                + (z - mean_z) * (z - mean_z));
  }
  scale /= static_cast<double>(points.size());
  scale /= sqrt(2.0);

  // If the calculated scale, for some reason, is ridiculously small (i.e.
  // dividing by zero), return an identity output matrix. This will make us not
  // normalize the data, but we should still get an okay fundamental matrix.
  if (scale < 1e-4) {
    LOG(WARNING)
        << "When normalizing feature coordinates scale factor was nearly zero. "
           "Proceeding without normalizing data.";
    return Eigen::MatrixXd::Identity(4, 4);
  }

  // Populate the output matrix.
  Eigen::Matrix4d T(Eigen::MatrixXd::Identity(4, 4));
  T(0, 0) = 1.0 / scale;
  T(1, 1) = 1.0 / scale;
  T(2, 2) = 1.0 / scale;
  T(0, 3) = -mean_x / scale;
  T(1, 3) = -mean_y / scale;
  T(2, 3) = -mean_z / scale;

  return T;
}

}  //\namespace bsfm
