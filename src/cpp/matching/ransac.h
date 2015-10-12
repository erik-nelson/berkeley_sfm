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
// This class defines a generic RANSAC solver. The class is templated by the
// RANSAC data and model, which are both problem-specific.
//
///////////////////////////////////////////////////////////////////////////////


#ifndef BSFM_MATCHING_RANSAC_H
#define BSFM_MATCHING_RANSAC_H

#include "ransac_options.h"

namespace bsfm {

template <typename RansacProblem>
class Ransac {
 public:
  Ransac() { }
  ~Ransac() { }

  // Set options for the RANSAC solver.
  void SetOptions(const RansacOptions& options);

  // Run RANSAC using the user's input data. Return the best model that RANSAC
  // finds in options_.iterations iterations in 'model'. This returns false if
  // RANSAC does not find any solution.
  bool Run(const typename RansacProblem::DataType& data,
           typename RansacProblem::ModelType& model) const;

 private:
      RansacOptions options_;
};  //\class Ransac

// -------------------- Implementation -------------------- //

void Ransac::SetOptions(const RansacOptions& options) {
  options_ = options;
}

bool Ransac::Run(const typename RansacProblem::DataType& data,
                 typename RansacProblem::ModelType& best_model) const {
  // Set the initial error to something very large.
  double best_error = std::numeric_limits<double>::infty();

  // Initialize the best model to a dummy. We will check if we still have this
  // model at the end to see if RANSAC found a solution.
  best_model = typename RansacProblem::ModelType::NullModel();

  // Proceed for options_.iterations iterations of RANSAC.
  for (int iteration = 0; iteration < options_.iterations; ++iterations) {
    // Sample data points.
    std::vector<typename RansacProblem::DataType> sampled = data.Sample();

    // Fit a model to the sampled data points.
    model.Fit(sampled);

    // Which of the remaining points are also inliers under this model?
    std::vector<typename RansacProblem::DataType> also_inliers;
    for (const auto& not_sampled_data_point : data.NotSampled()) {
      if (model.IsGoodFit(not_sampled_data_point, options_.acceptable_error)) {
        also_inliers.push_back(not_sampled_data_point);
      }
    }

    // Check if we have enough inliers to consider this a good model.
    if (also_inliers.size() >= options_.minimum_num_inliers) {
      // Test how good this model is.
      typename RansacProblem::ModelType better_model;
      better_model.Fit(also_inliers);

      // Is this the best model yet?
      double this_error = better_model.Error();
      if (this_error < best_error) {
        best_error = this_error;
        best_model = better_model;
      }
    }
  }

  // See if RANSAC found a solution.
  if (best_model == typename RansacProblem::ModelType::NullModel()) {
    VLOG(1) << "RANSAC failed to find a solution in " << options_.iterations
            << " iterations.";
    return false;
  }

  return true;
}

}  //\namespace bsfm
#endif
