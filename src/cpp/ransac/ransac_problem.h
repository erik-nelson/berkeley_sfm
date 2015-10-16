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
// This class defines an abstract base class for RANSAC problems. Derived
// classes will probide the specific details, data types, and model type that the
// Ransac class will use.
//
///////////////////////////////////////////////////////////////////////////////


#ifndef BSFM_RANSAC_RANSAC_PROBLEM_H
#define BSFM_RANSAC_RANSAC_PROBLEM_H

#include <vector>

#include "../util/disallow_copy_and_assign.h"

namespace bsfm {

// Derive from this class when defining a specific RANSAC problem!
template <typename DataType>
class RansacDataElement {
 public:
  RansacDataElement(const DataType& data) : data_(data) {}
  virtual ~RansacDataElement() { }

  DataType data_;
}; //\class RansacDataElement



// Derive from this class when defining a specific RANSAC problem!
template <typename DataType>
class RansacModel {
 public:
  RansacModel() { }
  virtual ~RansacModel() { }

  // ----- Define these remaining methods in a derived class! ----- //
  virtual double Error() const = 0;
  virtual bool IsGoodFit(const RansacDataElement<DataType>& data_point,
                         double error_tolerance) = 0;
}; //\class RansacModel



// Derive from this class when defining a specific RANSAC problem!
template <typename DataType, typename ModelType>
class RansacProblem {
 public:
  RansacProblem();
  virtual ~RansacProblem() { }

  virtual void SetModel(const ModelType& model);

  void SetData(const std::vector<RansacDataElement<DataType> >& data);

  virtual bool SolutionFound();
  virtual const ModelType& Model() const;

  // ----- Define these remaining methods in a derived class! ----- //
  virtual std::vector<RansacDataElement<DataType> > SampleData() = 0;
  virtual std::vector<RansacDataElement<DataType> > RemainingData() const = 0;
  virtual ModelType FitModel(
      const std::vector<RansacDataElement<DataType> >& input_data) const = 0;

 protected:
  std::vector<RansacDataElement<DataType> > data_;
  ModelType model_;
  bool solution_found_;

 private:
  DISALLOW_COPY_AND_ASSIGN(RansacProblem)
};  //\class RansacProblem

// -------------------- Implementation -------------------- //

template <typename DataType, typename ModelType>
RansacProblem<DataType, ModelType>::RansacProblem() : solution_found_(false) {}

template <typename DataType, typename ModelType>
void RansacProblem<DataType, ModelType>::SetModel(const ModelType& model) {
  model_ = model;
}

template <typename DataType, typename ModelType>
void RansacProblem<DataType, ModelType>::SetData(
    const std::vector<RansacDataElement<DataType> >& data) {
  data_ = data;
}

template <typename DataType, typename ModelType>
bool RansacProblem<DataType, ModelType>::SolutionFound() {
  return solution_found_;
}

template <typename DataType, typename ModelType>
const ModelType& RansacProblem<DataType, ModelType>::Model() const {
  return model_;
}

}  //\namespace bsfm

#endif
