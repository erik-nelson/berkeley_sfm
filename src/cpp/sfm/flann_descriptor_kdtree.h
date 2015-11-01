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
// A wrapper around the Fast Library for Approximate Nearest Neighbor search.
// This is useful for matching descriptors without having to run all pairwise
// comparisons. The FLANN kd tree builds up a high-dimensional spatial index of
// all input descriptors, allowing for fast but approximate nearest-neighbor
// queries in log(n) time, where n is the number of dimensions in the
// descriptor.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_SFM_FLANN_DESCRIPTOR_KDTREE_H
#define BSFM_SFM_FLANN_DESCRIPTOR_KDTREE_H

#include <flann/flann.h>

#include "../matching/distance_metric.h"
#include "../util/disallow_copy_and_assign.h"
#include "../util/types.h"

namespace bsfm {

class FlannDescriptorKDTree {
 public:
  FlannDescriptorKDTree();
  ~FlannDescriptorKDTree();

  // Add descriptors to the index.
  void AddDescriptor(Descriptor& descriptor);
  void AddDescriptors(std::vector<Descriptor>& descriptors);

  // Queries the kd tree for the nearest neighbor of 'query'. Returns whether or
  // not a nearest neighbor was found, and if it was found, the index and
  // distance to the nearest neighbor. Index is based on the order in which
  // descriptors were added with AddDescriptor() and AddDescriptors().
  bool NearestNeighbor(Descriptor& query, int& nn_index, double& nn_distance);

 private:
  DISALLOW_COPY_AND_ASSIGN(FlannDescriptorKDTree)

  std::shared_ptr< flann::Index<flann::L2<double> > > index_;

};  //\class FlannDescriptorKDTree
}  //\namespace bsfm

#endif
