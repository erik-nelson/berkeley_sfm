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
 * Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 *          Erik Nelson            ( eanelson@eecs.berkeley.edu )
 */

#include <pose/pose.h>

namespace bsfm {

// Initialize to the identity.
Pose::Pose() {
  Rt_ = Eigen::Matrix4d::Identity();
  this->ToAxisAngle();
}

// Construct a new Pose from a rotation matrix and translation vector.
Pose::Pose(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
  Rt_ = Eigen::Matrix4d::Identity();
  Rt_.block(0, 0, 3, 3) = R;
  Rt_.col(3).head(3) = t;
  this->ToAxisAngle();
}

// Construct a new Pose from a 4x4 Rt matrix.
Pose::Pose(const Eigen::Matrix4d& Rt) {
  Rt_ << Rt;
  this->ToAxisAngle();
}

// Deep copy constructor.
Pose::Pose(const Pose& other) {
  Rt_ << other.Rt_;
  aa_ << other.aa_;
}

// Multiply a homgenized point into a Pose.
Eigen::Vector4d Pose::Project(const Eigen::Vector4d& pt3d) {
  Eigen::Vector4d proj = Rt_ * pt3d;
  return proj;
} 

// Project a 3D point into this Pose.
Eigen::Vector2d Pose::ProjectTo2D(const Eigen::Vector3d& pt3d) {
  Eigen::Vector4d pt3d_h = Eigen::Vector4d::Constant(1.0);
  pt3d_h.head(3) = pt3d;

  const Eigen::Vector4d proj_h = Rt_ * pt3d_h;
  Eigen::Vector2d proj = proj_h.head(2);
  proj /= proj_h(2);

  return proj;
}

// Test if this pose (Rt_ only) is approximately equal to another pose.
bool Pose::IsApprox(const Pose& other) const {
  return Rt_.isApprox(other.Rt_);
}

// Compose this Pose with the given pose so that both refer to the identity Pose as
// specified by the given Pose.
void Pose::Compose(const Pose& other) {
  Rt_ *= other.Rt_;
}

// Invert this Pose. 
Pose Pose::Inverse() {
  Pose inv = Pose(Rt_.inverse());
  return inv;
}

// Multiply two Poses.
Pose operator* (const Pose& lhs, const Pose& rhs) const { return lhs * rhs; }

// Extract just the extrinsics matrix as a 3x4 matrix.
Eigen::Matrix<double, 3, 4> Pose::Dehomogenize() {
  Eigen::Matrix<double, 3, 4> extrinsics = Eigen::Matrix<double, 3, 4>();
  extrinsics = Rt_.block(0, 0, 3, 4);
  return extrinsics;
}

// Convert to axis-angle representation.
Eigen::VectorXd Pose::ToAxisAngle() {
  // from https://en.wikipedia.org/wiki/Axis-angle-representation
  const double angle = acos(0.5 * (Rt_.trace() - 2.0));
  Eigen::Vector3d axis = Eigen::Vector3d(Rt_(2, 1) - Rt_(1, 2),
					 Rt_(0, 2) - Rt_(2, 0),
					 Rt_(1, 0) - Rt_(0, 1)) * 0.5 / sin(angle);

  axis /= axis.norm();
  aa_ = axis * angle;

  return aa_;
}

// Convert to matrix representation.
Eigen::Matrix4d Pose::FromAxisAngle() {

  // from https://en.wikipedia.org/wiki/Rotation_matrix
  double angle = aa_.norm();
  Eigen::Vector3d axis = aa_ / angle;

  Eigen::Matrix3d cross;
  cross <<
    0.0, -axis(2), axis(1),
    axis(2), 0.0, -axis(0),
    -axis(1), axis(0), 0.0;

  Eigen::Matrix3d tensor;
  tensor <<
    axis(0)*axis(0), axis(0)*axis(1), axis(0)*axis(2),
    axis(0)*axis(1), axis(1)*axis(1), axis(1)*axis(2),
    axis(0)*axis(2), axis(1)*axis(2), axis(2)*axis(2);

  Eigen::Matrix3d R =
    cos(angle) * Eigen::Matrix3d::Identity() +
    sin(angle) * cross + (1-cos(angle)) * tensor;
  Rt_.block(0, 0, 3, 3) = R;

  return Rt_;
}

// Print to StdOut.
void Pose::Print() const {
  std::cout << "Pose matrix:\n" << Rt_ << std::endl;
  std::cout << "Pose axis-angle:\n" << aa_ << std::endl;
}

} // namespace bsfm
