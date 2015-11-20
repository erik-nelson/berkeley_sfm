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

#include "triangulation.h"

#include <Eigen/Core>
#include <Eigen/SVD>
#include <glog/logging.h>

#include "../util/types.h"

namespace bsfm {

using Eigen::MatrixXd;

// Triangulates a single 3D point from > 2 views using the inhomogeneous DLT
// method from H&Z: Multi-View Geometry, Ch 2.2.
bool Triangulate(const FeatureList& features,
                 const std::vector<Camera>& cameras,
                 Point3D& point,
                 double& uncertainty) {
  if (features.size() != cameras.size()) {
    LOG(WARNING)
        << "Number of features does not match number of cameras.";
    return false;
  }

  if (features.size() < 2) {
    LOG(WARNING) << "Need at least two features and cameras to triangulate.";
    return false;
  }

  // Construct the A matrix on page 312.
  MatrixXd A;
  A.resize(features.size() * 2, 4);
  for (size_t ii = 0; ii < features.size(); ++ii) {
    double u = features[ii].u_;
    double v = features[ii].v_;

    const Matrix34d P = cameras[ii].P();
    A.row(2*ii+0) = u * P.row(2) - P.row(0);
    A.row(2*ii+1) = v * P.row(2) - P.row(1);
  }

  // Get svd(A). Save some time and compute a thin U. We still need a full V.
  Eigen::JacobiSVD<MatrixXd> svd;
  svd.compute(A, Eigen::ComputeThinU | Eigen::ComputeFullV);
  if (!svd.computeV()) {
    VLOG(1) << "Failed to compute a singular value decomposition of A matrix.";
    return false;
  }

  // The 3D point is the eigenvector corresponding to the minimum eigenvalue.
  point = Point3D(svd.matrixV().block(0, 3, 3, 1) / svd.matrixV()(3,3));

  // Return false if the point is not visible from all cameras.
  for (size_t ii = 0; ii < cameras.size(); ++ii) {
    double u = 0.0, v = 0.0;
    if (!cameras[ii].WorldToImage(point.X(), point.Y(), point.Z(), &u, &v)) {
      return false;
    }
  }

  // Store the uncertainty as the inverse of the triangulation angle.
  const double angle = MaximumAngle(cameras, point);
  if (angle == 0.0) {  // we actually do want floating point comparison.
    uncertainty = std::numeric_limits<double>::max();
  } else {
    uncertainty = 1.0 / MaximumAngle(cameras, point);
  }

  return true;;
}

// Triangulate the 3D position of a point from a 2D correspondence and two sets
// of camera extrinsics and intrinsics.
bool Triangulate(const FeatureMatch& feature_match, const Camera& camera1,
                 const Camera& camera2, Point3D& point, double& uncertainty) {
  FeatureList features;
  features.push_back(feature_match.feature1_);
  features.push_back(feature_match.feature2_);

  std::vector<Camera> cameras;
  cameras.push_back(camera1);
  cameras.push_back(camera2);

  return Triangulate(features, cameras, point, uncertainty);
}

// Repeats the above function on a list of feature matches, returning a list of
// triangulated 3D points, where each point is computed from a single 2D <--> 2D
// correspondence.
bool Triangulate(const FeatureMatchList& feature_matches, const Camera& camera1,
                 const Camera& camera2, Point3DList& points, double& uncertainty) {
  // Clear output.
  points.clear();

  bool triangulated_all_points = true;
  for (size_t ii = 0; ii < feature_matches.size(); ++ii) {
    Point3D point;

    // Continue on failure, but store (0, 0, 0).
    double point_uncertainty = 0.0;
    if (!Triangulate(feature_matches[ii],
                     camera1,
                     camera2,
                     point,
                     point_uncertainty)) {
      uncertainty += point_uncertainty;
      triangulated_all_points = false;
      point = Point3D();
    }
    uncertainty += point_uncertainty;
    points.push_back(point);
  }

  return triangulated_all_points;
}

// Compute the maximum angle between each pair of observation angles.
double MaximumAngle(const std::vector<Camera>& cameras, const Point3D& point) {
  std::vector<Vector3d> vecs;
  for (const auto& camera : cameras)
    vecs.push_back((point.Get() - camera.Translation()).normalized());

  double largest_angle = 0.0;
  for (size_t ii = 0; ii < cameras.size() - 1; ++ii) {
    for (size_t jj = ii + 1; jj < cameras.size(); ++jj) {
      double angle = std::acos(vecs[ii].dot(vecs[jj]));

      if (std::isnan(angle) || std::isinf(angle)) {
        LOG(WARNING) << "Observation angle is NaN.";
        return std::numeric_limits<double>::max();
      }

      // We want two 90 degree observations to have the maximum possible angle.
      // Two observations with a 180 degree angle should map back to 0, as the
      // triangulated point will be colinear with the translation between the
      // two cameras, which makes it unobservable.
      if (angle > M_PI_2) {
        angle = M_PI - angle;
      }

      if (angle > largest_angle) {
        largest_angle = angle;
      }
    }
  }
  return largest_angle;
}

}  //\namespace bsfm
