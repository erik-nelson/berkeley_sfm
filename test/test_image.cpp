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

#include <image/image.h>
#include <strings/join_filepath.h>

#include <gflags/gflags.h>
#include <gtest/gtest.h>

DEFINE_string(testing_image_file, "lenna.png",
              "Name of the image used for testing.");

namespace bsfm {

using Eigen::MatrixXf;

const std::string test_image =
    strings::JoinFilepath(BSFM_TEST_DATA_DIR, FLAGS_testing_image_file.c_str());

TEST(Image, TestLoadImage) {
  // Load an OpenCV image and convert it to floating point RGB.
  cv::Mat image1 =
      cv::imread(test_image.c_str(), CV_LOAD_IMAGE_COLOR);
  cv::cvtColor(image1, image1, CV_BGR2RGB);

  // Load the same image in our format.
  Image image2(test_image.c_str());

  size_t w = image1.cols;
  size_t h = image1.rows;

  EXPECT_EQ(w, image2.Width());
  EXPECT_EQ(h, image2.Height());

  for (size_t c = 0; c < w; ++c)
   for (size_t r = 0; r < h; ++r)
     EXPECT_EQ(image1.at<cv::Vec3b>(r, c), image2.at<cv::Vec3b>(r, c));
}

TEST(Image, TestResize) {
  Image image(test_image.c_str());

  image.Resize(256, 256);
  EXPECT_EQ(256, image.Width());
  EXPECT_EQ(256, image.Height());

  image.Resize(2.0);
  EXPECT_EQ(512, image.Width());
  EXPECT_EQ(512, image.Height());
}

TEST(Image, TestConvertColor) {
  Image image(test_image.c_str());
  EXPECT_EQ(3, image.Channels());

  image.ConvertToGrayscale();
  EXPECT_EQ(1, image.Channels());

  image.ConvertToRGB();
  EXPECT_EQ(3, image.Channels());

  for (size_t c = 0; c < image.Width(); ++c) {
    for (size_t r = 0; r < image.Height(); ++r) {
      cv::Vec3b pixel = image.at<cv::Vec3b>(r, c);
      EXPECT_EQ(pixel[0], pixel[1]);
      EXPECT_EQ(pixel[1], pixel[2]);
    }
  }
}

TEST(Image, TestCopy) {
  Image image1(test_image.c_str());
  Image image2(image1);
  Image image3 = image1;

  for (size_t c = 0; c < image1.Width(); ++c) {
    for (size_t r = 0; r < image1.Height(); ++r) {
      EXPECT_EQ(image1.at<cv::Vec3b>(r, c), image2.at<cv::Vec3b>(r, c));
      EXPECT_EQ(image2.at<cv::Vec3b>(r, c), image3.at<cv::Vec3b>(r, c));
    }
  }
}

TEST(Image, TestLoadFromOpenCVMat) {
  // Load an OpenCV image and convert it to floating point RGB.
  cv::Mat image1 = cv::imread(test_image.c_str(), CV_LOAD_IMAGE_COLOR);

  // Load the same image in our format.
  Image image2(test_image.c_str());

  // Copy the OpenCV image to make a third image.
  Image image3(image1);

  // Check for equality.
  for (size_t c = 0; c < image2.Width(); ++c)
    for (size_t r = 0; r < image2.Width(); ++r)
      EXPECT_EQ(image2.at<cv::Vec3b>(r, c), image3.at<cv::Vec3b>(r, c));
}

TEST(Image, TestEigen) {
  Image image(test_image.c_str());

  // Create an Eigen matrix from the image.
  MatrixXf eigen_mat;
  image.ToEigen(eigen_mat);

  // Get the image in OpenCV format.
  image.ConvertToGrayscale();
  cv::Mat cv_image;
  image.ToCV(cv_image);

  // Convert to floating point.
  cv_image.convertTo(cv_image, CV_32F, 1.f / 255.f);

  // Make sure that the two matrices are equivalent.
  for (size_t c = 0; c < image.Width(); ++c)
    for (size_t r = 0; r < image.Width(); ++r)
      EXPECT_EQ(cv_image.at<float>(r, c), eigen_mat(r, c));

  // Convert back from Eigen to OpenCV.
  cv::Mat cv_image2;
  EigenMatToOpenCV<float>(eigen_mat, cv_image2);

  // Make sure that the two matrices are equivalent.
  for (size_t c = 0; c < image.Width(); ++c)
    for (size_t r = 0; r < image.Width(); ++r)
      EXPECT_EQ(cv_image.at<float>(r, c), cv_image2.at<float>(r, c));
}

} //\namespace bsfm
