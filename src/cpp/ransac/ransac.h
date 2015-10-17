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
// This class defines a generic RANSAC solver. The user may define a
// RansacProblem derived class, and plug it in here.
//
///////////////////////////////////////////////////////////////////////////////


#ifndef BSFM_RANSAC_RANSAC_H
#define BSFM_RANSAC_RANSAC_H

#include <limits>
#include <glog/logging.h>

#include "ransac_options.h"
#include "ransac_problem.h"

namespace bsfm {

template <typename DataType, typename ModelType>
class Ransac {
 public:
  Ransac() { }
  ~Ransac() { }

  // Set options for the RANSAC solver.
  inline void SetOptions(const RansacOptions& options);

  // Run RANSAC using the user's input data (stored in 'problem'). Save the best
  // model that RANSAC finds after options_.iterations iterations in the
  // problem. This returns false if RANSAC does not find any solution.
  inline bool Run(RansacProblem<DataType, ModelType>& problem) const;

 private:
      RansacOptions options_;
};  //\class Ransac

// -------------------- Implementation -------------------- //

template <typename DataType, typename ModelType>
void Ransac<DataType, ModelType>::SetOptions(const RansacOptions& options) {
  options_ = options;
}

template <typename DataType, typename ModelType>
bool Ransac<DataType, ModelType>::Run(
    RansacProblem<DataType, ModelType>& problem) const {
  // By default, a valid model has not been found.
  problem.SetSolutionFound(false);

  // Set the initial error to something very large.
  double best_error = std::numeric_limits<double>::infinity();

  // Proceed for options_.iterations iterations of RANSAC.
  for (unsigned int iter = 0; iter < options_.iterations; ++iter) {
    // Sample data points.
    std::vector<DataType> sampled = problem.SampleData(options_.num_samples);

    // Fit a model to the sampled data points.
    ModelType initial_model = problem.FitModel(sampled);

    // Which of the remaining points are also inliers under this model?
    std::vector<DataType> unsampled = problem.RemainingData(options_.num_samples);
    std::vector<DataType> inliers(sampled);
    for (const auto& not_sampled_data_point : unsampled) {
      if (initial_model.IsGoodFit(not_sampled_data_point,
                                  options_.acceptable_error)) {
        inliers.push_back(not_sampled_data_point);
      }
    }

    // Check if we have enough inliers to consider this a good model.
    if (inliers.size() >= options_.minimum_num_inliers) {

      // Test how good this model is.
      ModelType better_model = problem.FitModel(inliers);

      // Is this the best model yet?
      double this_error = better_model.Error();
      if (this_error < best_error) {
        best_error = this_error;
        problem.SetModel(better_model);
        problem.SetInliers(inliers);
        problem.SetSolutionFound(true);
      }
    }
  }

  // See if RANSAC found a solution.
  if (!problem.SolutionFound()) {
    VLOG(1) << "RANSAC failed to find a solution in " << options_.iterations
            << " iterations.";
    return false;
  }

  return true;
}

}  //\namespace bsfm

#endif
