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

#include "rotation.h"

#include <cmath>
#include <Eigen/LU>
#include <glog/logging.h>

namespace bsfm {

// Convert from Euler angles to a rotation matrix. Phi, theta, and psi define the
// angles of the intermediate rotations about x (R_x), y (R_y), and z (R_z)
// respectively. See https://en.wikipedia.org/wiki/Rotation_matrix.
Eigen::Matrix3d EulerAnglesToMatrix(double phi, double theta, double psi) {
  double c1 = std::cos(phi);
  double c2 = std::cos(theta);
  double c3 = std::cos(psi);
  double s1 = std::sin(phi);
  double s2 = std::sin(theta);
  double s3 = std::sin(psi);

  Eigen::Matrix3d R;
  R(0, 0) = c2*c3;
  R(0, 1) = c3*s1*s2 - c1*s3;
  R(0, 2) = s1*s3 + c1*c3*s2;
  R(1, 0) = c2*s3;
  R(1, 1) = c1*c3 + s1*s2*s3;
  R(1, 2) = c1*s2*s3 - c3*s1;
  R(2, 0) = -s2;
  R(2, 1) = c2*s1;
  R(2, 2) = c1*c2;

  return R;
}

// Same thing as above, but where phi, theta, and psi are specified as a vector.
Eigen::Matrix3d EulerAnglesToMatrix(const Eigen::Vector3d& euler_angles) {
  return EulerAnglesToMatrix(euler_angles(0), euler_angles(1), euler_angles(2));
}

// Convert from a rotation matrix to Euler angles.
// From: http://staff.city.ac.uk/~sbbh653/publications/euler.pdf
// Note that the solution that is returned is only unique when phi, theta, and
// psi are all <= 0.5 * PI. If this is not the case, they will still be correct,
// but may not be unique!
Eigen::Vector3d MatrixToEulerAngles(const Eigen::Matrix3d& R) {
  // Make sure R is actually a rotation matrix.
  if (std::abs(R.determinant() - 1) > 1e-4) {
    LOG(WARNING) << "R does not have a determinant of 1.";
    return Eigen::Vector3d::Zero();
  }

  double theta = -std::asin(R(2, 0));

  if (std::abs(cos(theta)) < 1e-8) {
    LOG(WARNING) << "Theta is approximately +/- PI/2, which yields a "
                    "singularity. Cannot decompose matrix into Euler angles.";
    return Eigen::Vector3d(theta, 0.0, 0.0);
  }

  double phi = std::atan2(R(2, 1), R(2, 2));
  double psi = std::atan2(R(1, 0) / std::cos(theta), R(0, 0) / std::cos(theta));

  return Eigen::Vector3d(phi, theta, psi);
}

// Get roll angle from a rotation matrix.
// Just like above, the solution will only be unique if roll < 0.5 * PI.
double Roll(const Eigen::Matrix3d& R) {
  double theta = -std::asin(R(2, 0));
  if (std::abs(std::cos(theta)) < 1e-8)
    return 0.0;
  return std::atan2(R(2, 1), R(2, 2));
}

// Get pitch angle from a rotation matrix.
double Pitch(const Eigen::Matrix3d& R) {
  return -std::asin(R(2, 0));
}

// Get yaw angle from a rotation matrix.
// Just like above, the solution will only be unique if yaw < 0.5 * PI.
double Yaw(const Eigen::Matrix3d& R) {
  double theta = -std::asin(R(2, 0));
  if (std::abs(std::cos(theta)) < 1e-8)
    return 0.0;
  return std::atan2(R(1, 0) / std::cos(theta), R(0, 0) / std::cos(theta));
}

// Unroll an angle to be \in [0, 2*PI)
double Unroll(double angle) {
  angle = fmod(angle, 2.0 * M_PI);
  if (angle < 0)
    angle += 2.0 * M_PI;
  return angle;
}

// Normalize an angle to be \in [-PI, PI)
double Normalize(double angle) {
  angle = fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0)
    angle += 2.0 * M_PI;
  return angle - M_PI;
}

// Computes the shortest distance between two angles on S^1.
// Found by manipulating the first answer on:
// stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
double S1Distance(double from, double to) {
  double d = Unroll(Unroll(to) - Unroll(from));
  if (d > M_PI)
    d -= 2.0*M_PI;
  return Normalize(d);
}

// Convert from degrees to radians.
double D2R(double angle) {
  return angle * M_PI / 180.0;
}

// Convert from radians to degrees.
double R2D(double angle) {
  return angle * 180.0 / M_PI;
}

// An error metric between two rotation matrices on SO3.
double SO3Error(const Eigen::Matrix3d& R1, const Eigen::Matrix3d& R2) {
  const Eigen::Matrix3d R_error = R1.transpose()*R2 - R2.transpose()*R1;
  const Eigen::Vector3d vee(R_error(2,1), R_error(0,2), R_error(1,0));
  return (0.5 * vee).norm();
}

}  //\namespace bsfm
