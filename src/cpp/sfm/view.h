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
// This View class models a camera at a specific frame index. On construction, a
// view is given a unique frame index which cannot be changed. Copy and
// assignment will copy the frame index, and will not create a new one.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_SFM_VIEW_H
#define BSFM_SFM_VIEW_H

#include <memory>

#include "../camera/camera.h"

namespace bsfm {

class View {
 public:
  typedef std::shared_ptr<View> Ptr;
  typedef std::shared_ptr<const View> ConstPtr;

  // Constructors will automatically initialize the View's frame number.
  View();
  View(const Camera& camera);
  ~View() {}

  // Copy constructor and assignment operator should not increment the frame
  // index.
  View(const View& other);
  View& operator=(const View& other);

  // Get and set the camera.
  void SetCamera(const class Camera& camera);
  Camera& MutableCamera();
  const class Camera& Camera() const;

  // Get this view's frame index.
  unsigned int FrameIndex() const;

  // For sorting a list of views by their frame indices.
  static bool SortByFrameIndex(const View& lhs, const View& rhs);
  static bool SortByFrameIndexPtr(const View::Ptr& lhs, const View::Ptr& rhs);
  static bool SortByFrameIndexConstPtr(const View::ConstPtr& lhs,
                                       const View::ConstPtr& rhs);

 private:
  // Static method for determining the next frame index across all Views
  // constructed so far. This is called in the View constructor.
  static unsigned int NextFrameIndex();

  // Includes intrinsics and extrinsics.
  class Camera camera_;

  // Provides an ordering on views.
  unsigned int frame_index_;
};  //\class View

}  //\namespace bsfm

#endif
