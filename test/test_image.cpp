/*
 * Copyright (C) 2015 - Erik Nelson
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 */

#include <image/image.h>

#include <gtest/gtest.h>
#include <gflags/gflags.h>

DEFINE_string(image_file, "", "Name of the image used for testing.");

namespace cider {

TEST(Image, TestLoadImage) {
  cimg_library::CImg<double> image1(FLAGS_image_file.c_str());
  Image<double> image2(FLAGS_image_file.c_str());

  size_t w = image1.width();
  size_t h = image1.height();

  for (size_t c = 0; c < w; ++c)
    for (size_t r = 0; r < h; ++r)
      ASSERT_EQ(image1(r, c), image2(r, c));
}

} //\namespace cider
