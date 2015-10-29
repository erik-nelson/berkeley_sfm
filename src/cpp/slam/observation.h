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
// The Observation struct defines an observation of a feature (with associated
// descriptor) from a specific camera view. This also includes the 3D point
// landmark that the feature corresponds to, if the observation has been
// matched.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_SLAM_OBSERVATION_H
#define BSFM_SLAM_OBSERVATION_H

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <memory>

#include "../matching/distance_metric.h"
#include "../matching/feature.h"
#include "../util/types.h"

namespace bsfm {

class Landmark;
class View;

class Observation {
 public:
  typedef std::shared_ptr<Observation> Ptr;
  typedef std::shared_ptr<const Observation> ConstPtr;

  // Initialize an observation with the view that it came from, an image-space
  // feature coordinate pair, and an associated descriptor. Implicitly
  // initializes the landmark to be invalid, since the observation has not been
  // matched with a 3D landmark yet.
  Observation(const std::shared_ptr<View>& view_ptr,
              const ::bsfm::Feature& feature,
              const ::bsfm::Descriptor& descriptor);
  ~Observation();

  // Factory method.
  static Observation::Ptr Create(const std::shared_ptr<View>& view_ptr,
                                 const ::bsfm::Feature& feature,
                                 const ::bsfm::Descriptor& descriptor);

  // Get the view that this observation was seen from.
  std::shared_ptr<View> GetView() const;

  // Get the landmark that this observation corresponds to. Returns a null
  // pointer if the observation has not been matched with a landmark.
  std::shared_ptr<Landmark> GetLandmark() const;

  // Associates a landmark with this observation. This is called by
  // Landmark::IncorporateObservation();
  void SetLandmark(LandmarkIndex landmark_index);

  // Returns whether or not the observation has been matched with a landmark. If
  // this returns false, 'GetLandmark()' will return a null pointer.
  bool IsMatched() const;

  // Get this observation's feature.
  const ::bsfm::Feature& Feature() const;

  // Get the descriptor corresponding to this observation's feature.
  const ::bsfm::Descriptor& Descriptor() const;

 private:
  // No default constructor.
  Observation();

  // The index corresponding to the view that this feature was observed from.
  ViewIndex view_index_;

  // The 3D point landmark that this observation is made from. This index is
  // undefined until the observation and a landmark have been matched.
  LandmarkIndex landmark_index_;

  // A boolean flag describing whether or not this observation of a feature has
  // been matched with a 3D point landmark.
  bool is_matched_;

  // A feature containing the (u, v) image space coordinates of the observation.
  ::bsfm::Feature feature_;

  // A descriptor associated with the feature.
  ::bsfm::Descriptor descriptor_;
};  //\class Observation

}  //\namespace bsfm

#endif
