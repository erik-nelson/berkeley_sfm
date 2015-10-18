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
// The Point3D class is a light-weight wrapper around an Eigen::Vector3d.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_GEOMETRY_POINT_3D_H
#define BSFM_GEOMETRY_POINT_3D_H

#include <Eigen/Core>
#include <memory>
#include <string>

namespace bsfm {

class Point3D {
 public:
  typedef std::shared_ptr<Point3D> Ptr;
  typedef std::shared_ptr<const Point3D> ConstPtr;

  // Constructors.
  Point3D();
  Point3D(double x, double y, double z);
  Point3D(const Point3D& in);
  Point3D(const Eigen::Vector3d& in);

  // Destructor.
  ~Point3D();

  // Setters.
  void SetX(double x);
  void SetY(double x);
  void SetZ(double x);
  void Set(double x, double y, double z);
  void Set(const Point3D& in);
  void Set(const Eigen::Vector3d& in);

  // Getters.
  double& operator()(int index);
  const double& operator()(int index) const;
  double X() const;
  double Y() const;
  double Z() const;
  const Eigen::Vector3d& Get() const;

  // Utility.
  void Print(const std::string& prefix = std::string()) const;

  // Math operations.
  void Normalize();
  Point3D Normalized() const;
  double Dot(const Point3D& other) const;
  Point3D Cross(const Point3D& other) const;

 private:
  Eigen::Vector3d data_;
};  //\class Point3D

}  //\namespace bsfm

#endif
