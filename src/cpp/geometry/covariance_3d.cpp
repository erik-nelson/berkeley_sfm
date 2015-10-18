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

#include <iostream>

#include "covariance_3d.h"

namespace bsfm {

// Default constructor.
Covariance3D::Covariance3D() : data_(Eigen::Matrix3d::Identity()) {}

// Element-wise constructor.
Covariance3D::Covariance3D(double x11, double x12, double x13,
                           double x21, double x22, double x23,
                           double x31, double x32, double x33) {
  data_(0, 0) = x11;
  data_(0, 1) = x12;
  data_(0, 2) = x13;
  data_(1, 0) = x21;
  data_(1, 1) = x22;
  data_(1, 2) = x23;
  data_(2, 0) = x31;
  data_(2, 1) = x32;
  data_(2, 1) = x33;
}

// Copy constructor.
Covariance3D::Covariance3D(const Covariance3D& in) : data_(in.Get()) {}

// Eigen constructor.
Covariance3D::Covariance3D(const Eigen::Matrix3d& in) : data_(in) {}

// Static zero.
Covariance3D Covariance3D::Zero() {
  return Covariance3D(Eigen::Matrix3d::Zero());
}

// Static identity.
Covariance3D Covariance3D::Identity() {
  return Covariance3D(Eigen::Matrix3d::Identity());
}

// Default destructor.
Covariance3D::~Covariance3D() {}

// Setters.
void Covariance3D::Set(const Covariance3D& in) {
  data_ = in.Get();
}

void Covariance3D::Set(const Eigen::Matrix3d& in) {
  data_ = in;
}

// Getters.
double& Covariance3D::operator()(int row, int col) {
  // No bound-checking. Be careful!
  return data_(row, col);
}

const double& Covariance3D::operator()(int row, int col) const {
  // No bound-checking. Be careful!
  return data_(row, col);
}

const Eigen::Matrix3d& Covariance3D::Get() const {
  return data_;
}

// Utility.
void Covariance3D::Print(const std::string& prefix) const {
  if (!prefix.empty()) {
    std::cout << prefix << std::endl;
  }

  std::cout << data_ << std::endl;
}

}  //\namespace bsfm
