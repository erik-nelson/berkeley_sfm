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
// This class defines a wrapper around OpenCV's Mat class.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_IMAGE_IMAGE_H
#define BSFM_IMAGE_IMAGE_H

#include <string>

#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace bsfm {

class Image {
 public:
  Image() : grayscale_(false) {}
  ~Image() {}

  // Copy ctor.
  Image(const Image& other);

  // Basic ctor.
  Image(size_t width, size_t height, size_t channels);

  // Ctor to load from file.
  explicit Image(const std::string& filename, bool grayscale = false);

  // Access the pixel at (u, v), at a specific channel.
  template <typename T>
  T& at(size_t u, size_t v);

  template <typename T>
  const T& at(size_t u, size_t v) const;

  // Access OpenCV mat.
  void GetCV(cv::Mat& out) const;

  // Save and load.
  void Read(const std::string& filename, bool grayscale = false);
  void Write(const std::string& filename) const;

  // Basic information.
  size_t Width() const;
  size_t Height() const;
  size_t Cols() const { return Width(); }
  size_t Rows() const { return Height(); }
  size_t Channels() const;
  bool IsColor() const { return !grayscale_; }

  // Resize to a specific scale or to a specific width and height.
  void Resize(double scale);
  void Resize(size_t new_width, size_t new_height);

  // Convert between grayscale and 3-channel color.
  void ConvertToGrayscale();
  void ConvertToRGB();

  // Open a window to display the image.
  void ImShow(const std::string& window_name = std::string(),
              unsigned int wait_time = 0);

 private:
  bool grayscale_;
  std::shared_ptr<cv::Mat> image_;
}; //\class Image


// ------------------- Implementation ------------------- //

template <typename T>
T& Image::at(size_t u, size_t v) {
  CHECK(image_ != nullptr) << "Image data is not allocated.";
  return image_->template at<T>(u, v);
}

template <typename T>
const T& Image::at(size_t u, size_t v) const {
  CHECK(image_ != nullptr) << "Image data is not allocated.";
  return image_->template at<T>(u, v);
}

} //\namespace bsfm
#endif