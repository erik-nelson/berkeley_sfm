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

#ifndef BSFM_POSE_POSE_H
#define BSFM_POSE_POSE_H

#include <Eigen/Dense>
#include <glog/logging.h>
#include <string>

#include "../util/types.h"

namespace bsfm {

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

class Pose {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Initialize to the identity.
  Pose();

  // Construct a new Pose from a rotation matrix and translation vector.
  Pose(const Matrix3d& R, const Vector3d& t);

  // Construct a new Pose from a de-homogenized 3x4 [R | t] matrix.
  Pose(const Matrix34d& Rt);

  // Deep copy constructor.
  Pose(const Pose& other);

  // Construct a new Pose from a 4x4 Rt matrix.
  Pose(const Matrix4d& other);

  // Destroy this Pose.
  ~Pose() { };

  // Individual element accessor.
  double& operator()(int i, int j);
  const double& operator()(int i, int j) const;

  // Access the homogeneous transformation matrix.
  Matrix4d& Get();
  const Matrix4d& Get() const;

  // Get the transformation's rotation components.
  Matrix3d Rotation() const;

  // Get the transformation's rotation components.
  Vector3d Translation() const;

  // Set the homogeneous transformation matrix.
  void Set(const Matrix4d& transformation);

  // Set rotation and translation directly.
  void SetRotation(const Matrix3d& rotation);
  void SetTranslation(const Vector3d& translation);

  // Multiply two Poses.
  Pose operator*(const Pose& other) const;

  // Multiply a homgenized point into a Pose.
  Vector4d Project(const Vector4d&);

  // Project a 3D point into this Pose.
  Vector2d ProjectTo2D(const Vector3d&);

  // Test if this pose (Rt_ only) is approximately equal to another Pose.
  bool IsApprox(const Pose&) const;

  // Compose this Pose with the given pose so that both refer to the identity
  // Pose as specified by the given Pose.
  void Compose(const Pose& other);

  // Invert this pose.
  Pose Inverse() const;

  // Extract just the extrinsics matrix as a 3x4 matrix.
  Matrix34d Dehomogenize();

  // Output axis-angle representation.
  VectorXd ToAxisAngle();

  // Set based on axis-angle input.
  Matrix4d FromAxisAngle(const Vector3d& aa);

  // Set translation directly.
  void SetX(double x);
  void SetY(double y);
  void SetZ(double z);

  // Get translation elements.
  const double& X() const;
  const double& Y() const;
  const double& Z() const;
  double& MutableX();
  double& MutableY();
  double& MutableZ();

  // Translate.
  void TranslateX(double dx);
  void TranslateY(double dy);
  void TranslateZ(double dz);

  // Get the relative transformation from this Pose to another one.
  Pose Delta(const Pose& rhs) const;

  // Print matrix entries.
  void Print(const std::string& prefix = std::string()) const;

 private:
  Matrix4d Rt_;  // 4x4 homogeneous Pose matrix
};  //\class Pose

};  // namespace bsfm

#endif
