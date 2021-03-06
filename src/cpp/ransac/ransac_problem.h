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
// This class defines an abstract base class for RANSAC problems.
// Structs/classes deriving from the RansacProblem and RansacModel interfaces
// will provide the specific details and model type that the the specific RANSAC
// problem will use.
//
///////////////////////////////////////////////////////////////////////////////


#ifndef BSFM_RANSAC_RANSAC_PROBLEM_H
#define BSFM_RANSAC_RANSAC_PROBLEM_H

#include <vector>

#include "../util/disallow_copy_and_assign.h"

namespace bsfm {

// Derive from this struct when defining a specific RANSAC problem!
template <typename DataType>
struct RansacModel {
  RansacModel() {}
  virtual ~RansacModel() {}

  // ----- Define these remaining methods in a derived struct! ----- //
  virtual double Error() const = 0;
  virtual bool IsGoodFit(const DataType& data_point,
                         double error_tolerance) const = 0;
};  //\struct RansacModel

// Derive from this class when defining a specific RANSAC problem!
template <typename DataType, typename ModelType>
class RansacProblem {
 public:
  RansacProblem();
  virtual ~RansacProblem() { }

  virtual inline void SetModel(const ModelType& model);
  virtual inline void SetData(
      const std::vector<DataType>& data);
  virtual inline void SetInliers(
      const std::vector<DataType>& inliers);

  virtual inline void SetSolutionFound(bool solution_found);
  virtual inline bool SolutionFound();
  virtual inline const ModelType& Model() const;
  virtual inline const std::vector<DataType>& Inliers() const;

  // ----- Define these remaining methods in a derived class! ----- //
  virtual std::vector<DataType> SampleData(unsigned int num_samples) = 0;
  virtual std::vector<DataType> RemainingData(
      unsigned int num_sampled_previously) const = 0;
  virtual ModelType FitModel(const std::vector<DataType>& input_data) const = 0;

 protected:
  std::vector<DataType> data_;
  std::vector<DataType> inliers_;
  ModelType model_;
  bool solution_found_;

 private:
  DISALLOW_COPY_AND_ASSIGN(RansacProblem)
};  //\class RansacProblem

// -------------------- Implementation -------------------- //

template <typename DataType, typename ModelType>
RansacProblem<DataType, ModelType>::RansacProblem()
    : model_(ModelType()), solution_found_(false) {}

template <typename DataType, typename ModelType>
void RansacProblem<DataType, ModelType>::SetModel(const ModelType& model) {
  model_ = model;
}

template <typename DataType, typename ModelType>
void RansacProblem<DataType, ModelType>::SetData(
    const std::vector<DataType>& data) {
  data_ = data;
}

template <typename DataType, typename ModelType>
void RansacProblem<DataType, ModelType>::SetInliers(
    const std::vector<DataType>& inliers) {
  inliers_ = inliers;
}

template <typename DataType, typename ModelType>
void RansacProblem<DataType, ModelType>::SetSolutionFound(bool solution_found) {
  solution_found_ = solution_found;
}

template <typename DataType, typename ModelType>
bool RansacProblem<DataType, ModelType>::SolutionFound() {
  return solution_found_;
}

template <typename DataType, typename ModelType>
const ModelType& RansacProblem<DataType, ModelType>::Model() const {
  return model_;
}

template <typename DataType, typename ModelType>
const std::vector<DataType>& RansacProblem<DataType, ModelType>::Inliers()
    const {
  return inliers_;
}

}  //\namespace bsfm

#endif
