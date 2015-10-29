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
                 const std::vector<Camera>& cameras, Point3D& point) {
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

  return true;
}

// Triangulate the 3D position of a point from a 2D correspondence and two sets
// of camera extrinsics and intrinsics.
bool Triangulate(const FeatureMatch& feature_match, const Camera& camera1,
                 const Camera& camera2, Point3D& point) {
  FeatureList features;
  features.push_back(feature_match.feature1_);
  features.push_back(feature_match.feature2_);

  std::vector<Camera> cameras;
  cameras.push_back(camera1);
  cameras.push_back(camera2);

  return Triangulate(features, cameras, point);
}

// Repeats the above function on a list of feature matches, returning a list of
// triangulated 3D points, where each point is computed from a single 2D <--> 2D
// correspondence.
bool Triangulate(const FeatureMatchList& feature_matches, const Camera& camera1,
                 const Camera& camera2, Point3DList& points) {
  // Clear output.
  points.clear();

  bool triangulated_all_points = true;
  for (size_t ii = 0; ii < feature_matches.size(); ++ii) {
    Point3D point;

    // Continue on failure, but store (0, 0, 0).
    if (!Triangulate(feature_matches[ii], camera1, camera2, point)) {
      triangulated_all_points = false;
      point = Point3D();
    }
    points.push_back(point);
  }

  return triangulated_all_points;
}

}  //\namespace bsfm
