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
// The KeyframeVisualOdometry class runs visual odometry from incremental image
// inputs using keyframes, which are initialized every time the camera pose has
// moved a threshold distance or every time the number of tracked features
// reduces below a threshold. At any time, the class can be queried for the 3D
// pose of the camera as well as the positions of all known landmarks.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_SLAM_KEYFRAME_VISUAL_ODOMETRY_H
#define BSFM_SLAM_KEYFRAME_VISUAL_ODOMETRY_H

#include "visual_odometry_annotator.h"
#include "visual_odometry_options.h"

#include "../camera/camera_intrinsics.h"
#include "../image/image.h"
#include "../matching/keypoint_detector.h"
#include "../matching/descriptor_extractor.h"
#include "../sfm/view.h"
#include "../util/disallow_copy_and_assign.h"
#include "../util/status.h"
#include "../util/types.h"

namespace bsfm {

class KeyframeVisualOdometry {
 public:
  KeyframeVisualOdometry(const VisualOdometryOptions& options, const CameraIntrinsics& camera);
  ~KeyframeVisualOdometry();

  // Update the estimate of the camera's position and all landmark positions.
  Status Update(const Image& image);

  // Get the last image used to call Update(), annotated with landmarks, feature
  // tracks, etc.
  void GetAnnotatedImage(Image* image) const;

  // Get indices of all views created by this visual odometry object.
  const std::vector<ViewIndex>& ViewIndices() const;

  // Write the camera trajectory to a .csv file.
  Status WriteTrajectoryToFile(const std::string& filename) const;

  // Write all landmarks observed by all cameras to a .csv file.
  Status WriteMapToFile(const std::string& filename) const;

 private:
  DISALLOW_COPY_AND_ASSIGN(KeyframeVisualOdometry)

  // Create the very first view. This will not initialize any tracks yet.
  void InitializeFirstView(const std::vector<Feature>& features,
                           const std::vector<Descriptor>& descriptors);

  // Use 2D<-->2D matches to initialize the position of the second camera (which
  // will be a keyframe). If the view is successfully localized, new tracks will
  // be initialized.
  Status InitializeSecondView(const std::vector<Feature>& features,
                              const std::vector<Descriptor>& descriptors);

  Status UpdateFeatureTracks(const std::vector<Feature>& features,
                             const std::vector<Descriptor>& descriptors,
                             ViewIndex view_index,
                             bool is_keyframe);

  // Use 2D<-->3D matching against landmarks in the filter to determine the
  // camera's pose.
  Status EstimatePose(ViewIndex view_index);

  // Detect keypoints from the input image. Returns false with an error status if
  // feature extraction fails. Non-const method because the detector is adaptive.
  Status GetKeypoints(const Image& image, std::vector<Keypoint>* keypoints);

  // Extract descriptors around the input keypoints. Non-const method because
  // OpenCV's descriptor extractor is non-const.
  Status GetDescriptors(const Image& image,
                        std::vector<Keypoint>* keypoints,
                        std::vector<Feature>* features,
                        std::vector<Descriptor>* descriptors);

  // Return the number of feature tracks that have a triangulated 3D position.
  unsigned int NumEstimatedTracks() const;

  // Return view indices in the sliding window.
  std::vector<ViewIndex> SlidingWindowViewIndices();

  // Boolean that is true whenever the camera pose has translated and rotated a
  // threshold amount (determined by options).
  bool initialize_new_keyframe_;

  // A copy of the set of options passed into the constructor.
  VisualOdometryOptions options_;

  // An annotator object used for drawing what is going on.
  VisualOdometryAnnotator annotator_;

  // A list of all views that have been added via visual odometry. Iterating
  // through these will generate the camera's trajectory.
  std::vector<ViewIndex> view_indices_;

  // A set of all features currently being tracked. If a feature that is
  // triangulated is removed from this list, it will be added to the list of
  // frozen landmarks below.
  std::vector<LandmarkIndex> tracks_;

  // A set of all landmarks that were successfully triangulated and are no
  // longer in the camera frame or being matched with. These are purely so that
  // we can visualize the map after VO finishes.
  std::vector<LandmarkIndex> frozen_landmarks_;

  // The index of the current keyframe.
  ViewIndex current_keyframe_;

  // Camera intrinsics. These are specified using the camera passed into the
  // constructor.
  CameraIntrinsics intrinsics_;

  // Keypoint detector and descriptor extractor for getting features and
  // descriptors from input images.
  KeypointDetector keypoint_detector_;
  DescriptorExtractor descriptor_extractor_;

  // The name of the OpenCV window for drawing.
  const std::string window_name = "Keyframe Visual Odometry";

};  //\class KeyframeVisualOdometry

}  //\namespace bsfm

#endif
