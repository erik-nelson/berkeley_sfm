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

#include <memory>
#include <string>

#include <Eigen/Dense>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace bsfm {

class Image {
 public:
  typedef std::shared_ptr<Image> Ptr;
  typedef std::shared_ptr<const Image> ConstPtr;

  Image() : grayscale_(false) {}
  ~Image() {}

  // Copy ctor.
  Image(const Image& other);

  // Basic ctor.
  Image(size_t width, size_t height, size_t channels);

  // Ctor to load from file.
  explicit Image(const std::string& filename, bool grayscale = false);

  // Construct from OpenCV mat.
  explicit Image(const cv::Mat& other);

  // Access the pixel at (u, v), at a specific channel.
  template <typename T>
  T& at(size_t u, size_t v);

  template <typename T>
  const T& at(size_t u, size_t v) const;

  // Copy to and from OpenCV mats.
  void ToCV(cv::Mat& out) const;
  void FromCV(const cv::Mat& in);

  // Convert to Eigen matrix.
  void ToEigen(Eigen::MatrixXf& eigen_out);

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

// Non-member conversion from OpenCV to Eigen matrices.
void OpenCVToEigenMat(const cv::Mat& cv_mat, Eigen::MatrixXf& eigen_mat);
void OpenCVToEigenVec(const cv::Mat& cv_vec, Eigen::VectorXf& eigen_vec);

// Non-member conversions from Eigen to OpenCV matrices.
void EigenMatToOpenCV(const Eigen::MatrixXf& eigen_mat, cv::Mat& cv_mat);
void EigenVecToOpenCV(const Eigen::VectorXf& eigen_vec, cv::Mat& cv_vec);

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
