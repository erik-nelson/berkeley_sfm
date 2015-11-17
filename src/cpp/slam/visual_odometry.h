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
// The VisualOdometry class runs visual odometry from incremental image inputs.
// As images are input, the system extracts features and descriptors, matches
// them against other images or against existing 3D points, and computes the 3D
// pose of the camera. At any time, the class can be queried for the 3D pose of
// the camera as well as the positions of all known landmarks.
//
// The VisualOdometry class handles initialization by itself using the first few
// images.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_SLAM_VISUAL_ODOMETRY_H
#define BSFM_SLAM_VISUAL_ODOMETRY_H

#include "visual_odometry_annotator.h"
#include "visual_odometry_options.h"

#include "../camera/camera.h"
#include "../camera/camera_intrinsics.h"
#include "../image/image.h"
#include "../matching/keypoint_detector.h"
#include "../matching/descriptor_extractor.h"
#include "../sfm/view.h"
#include "../util/disallow_copy_and_assign.h"
#include "../util/status.h"
#include "../util/types.h"

namespace bsfm {

class VisualOdometry {
 public:
  VisualOdometry(const VisualOdometryOptions& options, const Camera& camera);
  ~VisualOdometry();

  // Update the estimate of the camera's position and all landmark positions.
  Status Update(const Image& image);

  // Get the last image used to call Update(), annotated with landmarks, feature
  // tracks, etc.
  void GetAnnotatedImage(Image* image) const;

  // Get indices of all views created by this visual odometry object.
  const std::vector<ViewIndex>& ViewIndices() const;

 private:
  DISALLOW_COPY_AND_ASSIGN(VisualOdometry)

  // The first two images added are handled separately from all others in order
  // to initialize an estimate of the camera pose. The first image will simply
  // have descriptors extracted. The second image will be matched against the
  // first, and matched features will be triangulated. If the second image is
  // successfully matched with the first, future calls to Update() will not call
  // these functions.
  Status InitializeFirstView(const Image& image);
  Status InitializeSecondView(const Image& image);

  // Called for every image after initialization. This function extracts
  // descriptors from the image, matches them against 3D landmarks, computes the
  // new view's pose, and then triangulates new landmarks from the image's
  // observed features.
  Status ProcessImage(const Image& image);

  // Match unincorporated observations from 'new_view' with other unincorporated
  // observations in the sliding window of views. Triangulate the 3D position of
  // new matches, check reprojection error, and initialize new landmarks.
  void InitializeNewLandmarks(const View::Ptr& new_view);

  // Detect features and extract descriptors from the input image. Returns false
  // with an error status if either part fails.
  Status GetFeaturesAndDescriptors(const Image& image,
                                   std::vector<Feature>* features,
                                   std::vector<Descriptor>* descriptors);

  // Get all unincorporated features and descriptors from a view. This is used
  // to triangulate new features that were not previously matched.
  void GetUnusedFeatures(ViewIndex view_index,
                         std::vector<Feature>* features,
                         std::vector<Descriptor>* descriptors);

  // Return view indices in the sliding window.
  std::vector<ViewIndex> SlidingWindowViewIndices();

  // A copy of the set of options passed into the constructor.
  VisualOdometryOptions options_;

  // An annotator object used for drawing what is going on.
  VisualOdometryAnnotator annotator_;

  // A list of all views that have been added via visual odometry. Iterating
  // through these will generate the camera's trajectory.
  std::vector<ViewIndex> view_indices_;

  // Camera intrinsics. These are specified using the camera passed into the
  // constructor.
  CameraIntrinsics intrinsics_;

  // Keypoint detector and descriptor extractor for getting features and
  // descriptors from input images.
  KeypointDetector keypoint_detector_;
  DescriptorExtractor descriptor_extractor_;

  // Flag that is set to true after the first call to Update(). This is needed
  // to determine whether to call InitializeFirstview(), InitializeSecondView(),
  // or ProcessImage().
  bool has_first_view_;

  // The name of the OpenCV window for drawing.
  const std::string window_name = "Visual Odometry";

};  //\class VisualOdometry

}  //\namespace bsfm

#endif
