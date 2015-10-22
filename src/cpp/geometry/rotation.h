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

///////////////////////////////////////////////////////////////////////////////
//
// This file defines rotation utilities that can be used to compose rotations or
// convert between rotation parameterizations.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_GEOMETRY_ROTATION_H
#define BSFM_GEOMETRY_ROTATION_H

#include <Eigen/Core>

namespace bsfm {

// Convert from Euler angles to a rotation matrix. Phi, theta, and psi define the
// angles of the intermediate rotations about x (R_x), y (R_y), and z (R_z)
// respectively. See https://en.wikipedia.org/wiki/Rotation_matrix.
Eigen::Matrix3d EulerAnglesToMatrix(double phi, double theta, double psi);

// Same thing as above, but where phi, theta, and psi are specified as a vector.
Eigen::Matrix3d EulerAnglesToMatrix(const Eigen::Vector3d& euler_angles);

// Convert from a rotation matrix to Euler angles.
// From: http://staff.city.ac.uk/~sbbh653/publications/euler.pdf
// Note that the solution that is returned is only unique when phi, theta, and
// psi are all < 0.5 * PI. If this is not the case, they will still be correct,
// but may not be unique!
Eigen::Vector3d MatrixToEulerAngles(const Eigen::Matrix3d& R);

// Get roll angle from a rotation matrix.
// Just like above, the solution is only unique if roll < 0.5 * PI.
double Roll(const Eigen::Matrix3d& R);

// Get pitch angle from a rotation matrix.
// Just like above, the solution is only unique if pitch < 0.5 * PI.
double Pitch(const Eigen::Matrix3d& R);

// Get yaw angle from a rotation matrix.
// Just like above, the solution is only unique if yaw < 0.5 * PI.
double Yaw(const Eigen::Matrix3d& R);

// Unroll an angle to be \in [0, 2*PI)
double Unroll(double angle);

// Computes the shortest distance between two angles on S^1.
// Found by manipulating the first answer on:
// stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
double S1Distance(double from, double to);

// Normalize an angle to be \in [-PI, PI)
double Normalize(double angle);

// Convert from degrees to radians.
double D2R(double angle);

// Convert from radians to degrees.
double R2D(double angle);

// An error metric between two rotation matrices on SO3.
double SO3Error(const Eigen::Matrix3d& R1, const Eigen::Matrix3d& R2);

}  //\namespace bsfm

#endif
