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
// The Covariance3D class is a light-weight wrapper around an Eigen::Matrix3d.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_GEOMETRY_COVARIANCE_3D_H
#define BSFM_GEOMETRY_COVARIANCE_3D_H

#include <Eigen/Core>
#include <memory>
#include <string>

namespace bsfm {

using Eigen::Matrix3d;

class Covariance3D {
 public:
  typedef std::shared_ptr<Covariance3D> Ptr;
  typedef std::shared_ptr<const Covariance3D> ConstPtr;

  // Constructors.
  Covariance3D();
  Covariance3D(double x11, double x12, double x13,
               double x21, double x22, double x23,
               double x31, double x32, double x33);
  Covariance3D(const Covariance3D& in);
  Covariance3D(const Matrix3d& in);

  // Static defaults.
  static Covariance3D Zero();
  static Covariance3D Identity();

  // Destructor.
  ~Covariance3D();

  // Setters.
  void Set(const Covariance3D& in);
  void Set(const Matrix3d& in);

  // Getters. Primary access is via the operator() method.
  double& operator()(int row, int col);
  const double& operator()(int row, int col) const;
  const Matrix3d& Get() const;

  // Utility.
  void Print(const std::string& prefix = std::string()) const;

 private:
  Matrix3d data_;
};  //\class Covariance3D

}  //\namespace bsfm

#endif
