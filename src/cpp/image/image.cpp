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

#include "image.h"

namespace bsfm {

// Deep copy ctor.
Image::Image(const Image& other) : grayscale_(false) {
  image_ = other.image_;
  grayscale_ = other.grayscale_;
}

// Basic ctor.
Image::Image(size_t width, size_t height, size_t channels) {
  if (channels == 1) {
    grayscale_ = true;
    image_ = std::shared_ptr<cv::Mat>(new cv::Mat(width, height, CV_32F, 0.f));
  } else {
    grayscale_ = false;
    image_ = std::shared_ptr<cv::Mat>(
        new cv::Mat(width, height, CV_32FC3, CV_RGB(0.f, 0.f, 0.f)));
  }
}

// Load from file ctor.
Image::Image(const std::string& filename, bool grayscale) : grayscale_(false) {
  image_ = std::shared_ptr<cv::Mat>(new cv::Mat());
  Read(filename, grayscale);
}

// Construct form OpenCV mat.
Image::Image(const cv::Mat& other) {
  image_ = std::shared_ptr<cv::Mat>(new cv::Mat(other));
  grayscale_ = (this->Channels() == 1);
}

void Image::GetCV(cv::Mat& out) const {
  CHECK(image_.get()) << "Image data is not allocated.";

  // Convert from RGB (internal representation) to BGR (default in OpenCV).
  cv::cvtColor(*image_, out, CV_RGB2BGR);
}

void Image::ToEigen(Eigen::MatrixXf& eigen_out) {
  CHECK(image_.get()) << "Image data is not allocated.";
  OpenCVToEigen(*image_, eigen_out);
}

void Image::Read(const std::string& filename, bool grayscale) {
  if (grayscale) {
    *image_ = cv::imread(filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);

    // Convert from unsigned to floating point.
    image_->convertTo(*image_, CV_32F, 1.f / 255.f);
  } else {
    *image_ = cv::imread(filename.c_str(), CV_LOAD_IMAGE_COLOR);

    // Convert from unsigned to floating point.
    image_->convertTo(*image_, CV_32FC3, 1.f / 255.f);
  }
  CHECK(image_->data) << "Unable to read image file.";


  // Convert from BGR (default in OpenCV) to RGB.
  cv::cvtColor(*image_, *image_, CV_BGR2RGB);
}

void Image::Write(const std::string& filename) const {
  cv::imwrite(filename.c_str(), *image_);
}

size_t Image::Width() const {
  CHECK(image_.get()) << "Image data is not allocated.";
  return image_->cols;
}

size_t Image::Height() const {
  CHECK(image_.get()) << "Image data is not allocated.";
  return image_->rows;
}

size_t Image::Channels() const {
  CHECK(image_.get()) << "Image data is not allocated.";
  return image_->channels();
}

void Image::Resize(double scale) {
  CHECK(image_.get()) << "Image data is not allocated.";
  cv::resize(*image_, *image_, cv::Size(), scale, scale, CV_INTER_LANCZOS4);
}

void Image::Resize(size_t new_width, size_t new_height) {
  CHECK(image_.get()) << "Image data is not allocated.";
  cv::resize(*image_, *image_, cv::Size(new_width, new_height),
             CV_INTER_LANCZOS4);
}

void Image::ConvertToGrayscale() {
  CHECK(image_.get()) << "Image data is not allocated.";

  if (grayscale_) {
    VLOG(1) << "Cannot convert image to grayscale, image is already grayscale.";
    return;
  }

  cv::cvtColor(*image_, *image_, CV_RGB2GRAY);
  grayscale_ = true;
}

void Image::ConvertToRGB() {
  CHECK(image_.get()) << "Image data is not allocated.";

  if (!grayscale_) {
    VLOG(1) << "Cannot convert image to RGB, image is already RGB.";
    return;
  }

  cv::cvtColor(*image_, *image_, CV_GRAY2RGB);
  grayscale_ = false;
}

void Image::ImShow(const std::string& window_name, unsigned int wait_time) {
  CHECK(image_.get()) << "Image data is not allocated.";

  cv::namedWindow(window_name.c_str(), CV_WINDOW_AUTOSIZE);

  // If the image is not grayscale, convert it from RGB to BGR.
  if (!grayscale_) {
    cv::Mat bgr_image;
    GetCV(bgr_image);

    cv::imshow(window_name.c_str(), bgr_image);
  } else {
    cv::imshow(window_name.c_str(), *image_);
  }

  cv::waitKey(wait_time);
}

// Non-member conversion from OpenCV to Eigen matrices.
void OpenCVToEigen(const cv::Mat& cv_mat, Eigen::MatrixXf& eigen_mat) {
  // Make sure the data is grayscale before converting to an eigen matrix.
  if (cv_mat.channels() != 1) {
    cv::Mat grayscale_image;
    cv::cvtColor(cv_mat, grayscale_image, CV_RGB2GRAY);
    cv::cv2eigen(grayscale_image, eigen_mat);
  } else {
    cv::cv2eigen(cv_mat, eigen_mat);
  }
}

// Non-member conversion from Eigen to OpenCV matrices.
void EigenToOpenCV(const Eigen::MatrixXf& eigen_mat, cv::Mat& cv_mat) {
  cv::eigen2cv(eigen_mat, cv_mat);
}

} //\namespace bsfm
