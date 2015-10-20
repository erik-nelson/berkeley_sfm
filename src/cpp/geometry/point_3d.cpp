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

#include "point_3d.h"

namespace bsfm {

// Default constructor.
Point3D::Point3D() : data_(Eigen::Vector3d::Zero()) {}

// (x, y, z) constructor.
Point3D::Point3D(double x, double y, double z)
    : data_(Eigen::Vector3d(x, y, z)) {}

// Copy constructors.
Point3D::Point3D(const Point3D& in) : data_(in.Get()) {}

// Eigen constructor.
Point3D::Point3D(const Eigen::Vector3d& in) : data_(in) {}

// Basic destructor.
Point3D::~Point3D() {}

// Setters.
void Point3D::SetX(double x) { data_(0) = x; }

void Point3D::SetY(double y) { data_(1) = y; }

void Point3D::SetZ(double z) { data_(2) = z; }

void Point3D::Set(double x, double y, double z) {
  data_(0) = x;
  data_(1) = y;
  data_(2) = z;
}

void Point3D::Set(const Point3D& in) {
  data_ = in.Get();
}

void Point3D::Set(const Eigen::Vector3d& in) {
  data_ = in;
}

// Getters.
double& Point3D::operator()(int index) {
  // No bound-checking. Be careful!
  return data_(index);
}

const double& Point3D::operator()(int index) const {
  // No bound-checking. Be careful!
  return data_(index);
}

double Point3D::X() const { return data_(0); }

double Point3D::Y() const { return data_(1); }

double Point3D::Z() const { return data_(2); }

// Const reference return.
const Eigen::Vector3d& Point3D::Get() const {
  return data_;
}

// Utility.
void Point3D::Print(const std::string& prefix) const {
  if (!prefix.empty()) {
    std::cout << prefix << std::endl;
  }

  std::cout << data_ << std::endl;
}

// Math operations.
void Point3D::Normalize() {
  data_.normalize();
}

Point3D Point3D::Normalized() const {
  Point3D out(data_);
  out.Normalize();
  return out;
}

double Point3D::Dot(const Point3D& other) const {
  return data_.dot(other.Get());
}

}  //\namespace bsfm
