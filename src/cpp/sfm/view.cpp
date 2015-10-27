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

#include "view.h"

namespace bsfm {

// Automatically initialize the View's frame index.
View::View() : frame_index_(NextFrameIndex()) {}

View::View(const class Camera& camera)
    : frame_index_(NextFrameIndex()), camera_(camera) {}

// Copy constructor and assignment operator should not increment the frame
// index.
View::View(const View& other)
    : camera_(other.camera_), frame_index_(other.frame_index_) {}

View& View::operator=(const View& other) {
  // Check for self-assignment.
  if (this == &other)
    return *this;

  // Copy data without changing frame index.
  this->camera_ = other.camera_;
  this->frame_index_ = other.frame_index_;
  return *this;
}

void View::SetCamera(const class Camera& camera) {
  camera_ = camera;
}

class Camera& View::MutableCamera() {
  return camera_;
}

const class Camera& View::Camera() const {
  return camera_;
}

unsigned int View::FrameIndex() const {
  return frame_index_;
}

bool View::SortByFrameIndex(const View& lhs, const View& rhs) {
  return lhs.FrameIndex() < rhs.FrameIndex();
}

bool View::SortByFrameIndexPtr(const View::Ptr& lhs, const View::Ptr& rhs) {
  return lhs->FrameIndex()< rhs->FrameIndex();
}

bool View::SortByFrameIndexConstPtr(const View::ConstPtr& lhs,
                                    const View::ConstPtr& rhs) {
  return lhs->FrameIndex() < rhs->FrameIndex();
}

// Static method for determining the next frame index across all Views
// constructed so far. This is called in the View constructor.
unsigned int View::NextFrameIndex() {
  static unsigned int current_frame_index = 0;
  return current_frame_index++;
}

}  //\namespace bsfm
