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

DEFINE_string(image_file, "lenna.png", "Name of the image used for testing.");

namespace bsfm {

const std::string test_image =
    strings::JoinFilepath(BSFM_TEST_DATA_DIR, FLAGS_image_file.c_str());

TEST(Image, TestLoadImage) {
  cv::Mat image1 =
      cv::imread(test_image.c_str(), CV_LOAD_IMAGE_COLOR);
  image1.convertTo(image1, CV_32FC3, 1.f / 255.f);

  cv::cvtColor(image1, image1, CV_BGR2RGB);

  Image image2(test_image.c_str());

  size_t w = image1.cols;
  size_t h = image1.rows;

  ASSERT_EQ(w, image2.Width());
  ASSERT_EQ(h, image2.Height());

  for (size_t c = 0; c < w; ++c)
   for (size_t r = 0; r < h; ++r)
     ASSERT_EQ(image1.at<cv::Vec3f>(r, c), image2.at<cv::Vec3f>(r, c));
}

TEST(Image, TestResize) {
  Image image1(test_image.c_str());

  image1.Resize(256, 256);
  ASSERT_EQ(256, image1.Width());
  ASSERT_EQ(256, image1.Height());

  image1.Resize(2.0);
  ASSERT_EQ(512, image1.Width());
  ASSERT_EQ(512, image1.Height());
}

TEST(Image, TestConvertColor) {

  Image image1(test_image.c_str());
  ASSERT_EQ(3, image1.Channels());

  image1.ConvertToGrayscale();
  ASSERT_EQ(1, image1.Channels());

  image1.ConvertToRGB();
  ASSERT_EQ(3, image1.Channels());

  for (size_t c = 0; c < image1.Width(); ++c) {
    for (size_t r = 0; r < image1.Height(); ++r) {
      cv::Vec3f pixel = image1.at<cv::Vec3f>(r, c);
      ASSERT_EQ(pixel[0], pixel[1]);
      ASSERT_EQ(pixel[1], pixel[2]);
    }
  }
}

TEST(Image, TestCopy) {

  Image image1(test_image.c_str());
  Image image2(image1);
  Image image3 = image1;

  for (size_t c = 0; c < image1.Width(); ++c) {
    for (size_t r = 0; r < image1.Height(); ++r) {
      ASSERT_EQ(image1.at<cv::Vec3f>(r, c), image2.at<cv::Vec3f>(r, c));
      ASSERT_EQ(image2.at<cv::Vec3f>(r, c), image3.at<cv::Vec3f>(r, c));
    }
  }
}

} //\namespace bsfm

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
