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

#include <limits>
#include <memory>

#include <gflags/gflags.h>
#include <sfm/flann_descriptor_kdtree.h>
#include <util/types.h>

#include <gtest/gtest.h>

namespace bsfm {

TEST(FlannDescriptorKDTree, TestFlannDescriptorKDTree) {
  // Make a FLANN kd tree.
  FlannDescriptorKDTree descriptor_kdtree;

  // Make a bunch of descriptors and incrementally insert them into the kd tree.
  std::vector<Descriptor> descriptors;
  for (int ii = 0; ii < 10; ++ii) {
    descriptors.push_back(Descriptor::Random(64));
    descriptor_kdtree.AddDescriptor(descriptors.back());
  }

  // Add a batch of descriptors at once.
  std::vector<Descriptor> descriptors2;
  for (int ii = 0; ii < 0; ++ii)
    descriptors.push_back(Descriptor::Random(64));
  descriptor_kdtree.AddDescriptors(descriptors2);

  // Query the kd tree for nearest neighbor.
  Descriptor query(Descriptor::Random(64));
  int nn_index = -1;
  double nn_distance = 0.0;
  EXPECT_TRUE(descriptor_kdtree.NearestNeighbor(query, nn_index, nn_distance));
  EXPECT_NE(-1, nn_index);

  // Manually compute distance between all descriptors and the query.
  descriptors.insert(descriptors.end(),
                     descriptors2.begin(),
                     descriptors2.end());

  double min_distance = std::numeric_limits<double>::max();
  size_t min_distance_index = 0;
  for (size_t ii = 0; ii < descriptors.size(); ++ii) {
    double distance = (descriptors[ii] - query).squaredNorm();
    if (distance < min_distance) {
      min_distance = distance;
      min_distance_index = ii;
    }
    printf("desc %lu, distance: %lf\n", ii, distance);
  }
  printf("nn distance: %lf\n", nn_distance);

  EXPECT_EQ(min_distance_index, nn_index);

}

TEST(FlannDescriptorKDTree, TestReleasePointers) {
  // See if FLANN is performing a copy of our descriptors under the hood by
  // creating pointers to data, inserting the data into the FLANN kd tree,
  // releasing the data, and then querying the kd tree.
  FlannDescriptorKDTree descriptor_kdtree;

  // Make some descriptors and insert them into the kd tree.
  std::vector<Descriptor> descriptors;
  for (int ii = 0; ii < 100; ++ii) {
    std::shared_ptr<Descriptor> descriptor_ptr(
        new Descriptor(Descriptor::Random(32)));

    descriptor_kdtree.AddDescriptor(*descriptor_ptr);

    // Copy the data for test comparison and then let the memory free.
    descriptors.push_back(*descriptor_ptr);
  }
  // Descriptors should have deallocated upon leaving for loop scope.

  // Now query for nearest neighbor.
  Descriptor query(Descriptor::Random(32));
  int nn_index = -1;
  double nn_distance = 0.0;
  EXPECT_TRUE(descriptor_kdtree.NearestNeighbor(query, nn_index, nn_distance));
  EXPECT_NE(-1, nn_index);


  // Did it give us the right descriptor?
}

}  //\namespace bsfm
