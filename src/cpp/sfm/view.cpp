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

// Declaration of static member variable.
std::unordered_map<unsigned int, View::Ptr> View::view_registry_;

// Factory method. Registers the view and index in the view registry so
// that they can be accessed from the static GetView() method. This guarantees
// that all views will have uniqueindices.
View::Ptr View::Create(const class Camera& camera) {
  // Create a new view, implicitly assigning a unique index.
  View::Ptr view(new View(camera));

  // Register the view.
  view_registry_.insert(std::make_pair(view->Index(), view));

  // Return the created view.
  return view;
}

// Gets the view corresponding to the input view index. If the view has not been
// created yet, this method returns a null pointer.
View::Ptr View::GetView(unsigned int view_index) {
  auto registry_element = view_registry_.find(view_index);
  if (registry_element == view_registry_.end())
    return View::Ptr();

  return registry_element ->second;
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

unsigned int View::Index() const {
  return view_index_;
}

bool View::SortByIndex(const View::Ptr& lhs, const View::Ptr& rhs) {
  return lhs->Index()< rhs->Index();
}

// Hidden constructor. This will be called from the factory method.
View::View(const class Camera& camera)
    : view_index_(NextViewIndex()), camera_(camera) {}

// Static method for determining the next index across all Views
// constructed so far. This is called in the View constructor.
unsigned int View::NextViewIndex() {
  static unsigned int current_view_index = 0;
  return current_view_index++;
}

}  //\namespace bsfm
