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

#include <iostream>

namespace bsfm {

// Initialize to the identity.
Pose::Pose() {
  Rt_ = Matrix4d::Identity();
}

// Construct a new Pose from a rotation matrix and translation vector.
Pose::Pose(const Matrix3d& R, const Vector3d& t) {
  Rt_ = Matrix4d::Identity();
  Rt_.block(0, 0, 3, 3) = R;
  Rt_.col(3).head(3) = t;
}

// Construct a new Pose from a de-homogenized 3x4 [R | t] matrix.
Pose::Pose(const Matrix34d& Rt) {
  Rt_ = Matrix4d::Identity();
  Rt_.block(0, 0, 3, 4) = Rt;
}

// Construct a new Pose from a 4x4 Rt matrix.
Pose::Pose(const Matrix4d& Rt) {
  Rt_ << Rt;
}

// Deep copy constructor.
Pose::Pose(const Pose& other) {
  Rt_ << other.Rt_;
}

// Access operators delegate to the Eigen access operators.
double& Pose::operator()(int i, int j) {
  return Rt_(i, j);
}

const double& Pose::operator()(int i, int j) const {
  return Rt_(i, j);
}

// Access the homogeneous transformation matrix.
Matrix4d& Pose::Get() {
  return Rt_;
}

const Matrix4d& Pose::Get() const {
  return Rt_;
}

// Get the transformation's rotation components.
Matrix3d Pose::Rotation() const {
  return Rt_.block(0, 0, 3, 3);
}

// Get the transformation's translation components.
Vector3d Pose::Translation() const {
  return Rt_.block(0, 3, 3, 1);
}

// Set the homogeneous transformation matrix.
void Pose::Set(const Matrix4d& transformation) {
  Rt_ = transformation;
}

// Set the Rt matrix.
void Pose::Set(const Matrix34d& Rt) {
  Rt_.block(0, 0, 3, 4) = Rt;
}

// Set rotation and translation directly.
void Pose::SetRotation(const Matrix3d& rotation) {
  Rt_.block(0, 0, 3, 3) = rotation;
}

void Pose::SetTranslation(const Vector3d& translation) {
  Rt_.block(0, 3, 3, 1) = translation;
}

// Overloaded multiplication operator for composing two poses.
Pose Pose::operator*(const Pose& other) const {
  Matrix4d Rt_out = Rt_ * other.Rt_;
  return Pose(Rt_out);
}

// Multiply a homgenized point into a Pose.
Vector4d Pose::Project(const Vector4d& pt3d) {
  Vector4d proj = Rt_ * pt3d;
  return proj;
}

// Project a 3D point into this Pose.
Vector2d Pose::ProjectTo2D(const Vector3d& pt3d) {
  Vector4d pt3d_h = Vector4d::Constant(1.0);
  pt3d_h.head(3) = pt3d;

  const Vector4d proj_h = Rt_ * pt3d_h;
  Vector2d proj = proj_h.head(2);
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
Pose Pose::Inverse() const {
  Matrix4d Rt_inverse = Rt_.inverse();
  Pose out(Rt_inverse);
  return out;
}

// Extract just the extrinsics matrix as a 3x4 matrix.
Matrix34d Pose::Dehomogenize() {
  return Rt_.block(0, 0, 3, 4);
}

// Convert to axis-angle representation.
Vector3d Pose::AxisAngle() const {
  // From https://en.wikipedia.org/wiki/Axis-angle_representation.
  const double angle = acos(0.5 * (Rt_.trace() - 2.0));
  Vector3d axis = Vector3d(Rt_(2, 1) - Rt_(1, 2),
					                 Rt_(0, 2) - Rt_(2, 0),
					                 Rt_(1, 0) - Rt_(0, 1));

  // Handle identity rotation separately.
  if (axis.isApprox(Vector3d::Zero(), 1e-16)) {
    return axis;
  }

  axis /= axis.norm();
  return axis * angle;
}

// Convert to matrix representation.
Matrix4d Pose::FromAxisAngle(const Vector3d& aa) {
  // From https://en.wikipedia.org/wiki/Rotation_matrix.
  double angle = aa.norm();
  Vector3d axis = aa / angle;

  Matrix3d cross;
  cross <<
    0.0, -axis(2), axis(1),
    axis(2), 0.0, -axis(0),
    -axis(1), axis(0), 0.0;

  Matrix3d tensor;
  tensor <<
    axis(0)*axis(0), axis(0)*axis(1), axis(0)*axis(2),
    axis(0)*axis(1), axis(1)*axis(1), axis(1)*axis(2),
    axis(0)*axis(2), axis(1)*axis(2), axis(2)*axis(2);

  Matrix3d R =
    cos(angle) * Matrix3d::Identity() +
    sin(angle) * cross + (1-cos(angle)) * tensor;
  Rt_.block(0, 0, 3, 3) = R;

  return Rt_;
}

void Pose::SetX(double x) {
  Rt_(0, 3) = x;
}

void Pose::SetY(double y) {
  Rt_(1, 3) = y;
}

void Pose::SetZ(double z) {
  Rt_(2, 3) = z;
}

const double& Pose::X() const {
  return Rt_(0, 3);
}

const double& Pose::Y() const {
  return Rt_(1, 3);
}

const double& Pose::Z() const {
  return Rt_(2, 3);
}

double& Pose::MutableX() {
  return Rt_(0, 3);
}

double& Pose::MutableY() {
  return Rt_(1, 3);
}

double& Pose::MutableZ() {
  return Rt_(2, 3);
}

void Pose::TranslateX(double dx) {
  Rt_(0, 3) += dx;
}

void Pose::TranslateY(double dy) {
  Rt_(1, 3) += dy;
}

void Pose::TranslateZ(double dz) {
  Rt_(2, 3) += dz;
}

// Print matrix entries.
void Pose::Print(const std::string& prefix) const {
  if (!prefix.empty()) {
    std::cout << prefix << std::endl;
  }
  std::cout << Rt_ << std::endl;
}

// Get the relative transformation from this Pose to another one.
Pose Pose::Delta(const Pose& rhs) const {
  return this->Inverse() * rhs;
}

} // namespace bsfm
