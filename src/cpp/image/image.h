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

#ifndef IMAGE_IMAGE_H
#define IMAGE_IMAGE_H

#include <string>

#include <CImg.h>

namespace bsfm {

template <typename T>
class Image {
 public:
  Image() {}
  ~Image() {}

  // Copy ctor.
  Image(const Image<T>& other);

  // Basic ctor.
  Image(size_t width, size_t height, size_t channels);

  // Load from file.
  explicit Image(const std::string& filename);

  // Save and load.
  void Read(const std::string& filename);
  void Write(const std::string& filename) const;

  // Basic information.
  size_t Width() const;
  size_t Height() const;
  size_t Cols() const { return Width(); }
  size_t Rows() const { return Height(); }
  size_t Channels() const;

  // Resize to a specific scale or to a specific width and height.
  void Resize(double scale);
  void Resize(size_t new_width, size_t new_height);

  // Access the pixel at (u, v), at a specific channel.
  T& operator()(size_t u, size_t v, size_t c = 0);
  const T& operator()(size_t u, size_t v, size_t c = 0) const;

 private:
  cimg_library::CImg<T> image_;
}; //\class Image

// ----------------------------- Implementation ----------------------------- //

// Deep copy ctor.
template <typename T>
Image<T>::Image(const Image<T>& other) {
  image_ = other.image_;
}

// Basic ctor.
template <typename T>
Image<T>::Image(size_t width, size_t height, size_t channels) {
  image_.resize(width, height, 1, channels);
}

// Load from file ctor.
template <typename T>
Image<T>::Image(const std::string& filename) {
  image_.load(filename.c_str());
}

template <typename T>
void Image<T>::Read(const std::string& filename) {
  image_.load(filename.c_str());
}

template <typename T>
void Image<T>::Write(const std::string& filename) const {
  image_.save(filename.c_str());
}

template <typename T>
size_t Image<T>::Width() const {
  return image_.width();
}

template <typename T>
size_t Image<T>::Height() const {
  return image_.height();
}

template <typename T>
size_t Image<T>::Channels() const {
  return image_.spectrum();
}

template <typename T>
void Image<T>::Resize(double scale) {
  image_.resize(static_cast<size_t>(Width() * scale),
                static_cast<size_t>(Height() * scale));
}

template <typename T>
void Image<T>::Resize(size_t new_width, size_t new_height) {
  image_.resize(new_width, new_height);
}

template <typename T>
T& Image<T>::operator()(size_t u, size_t v, size_t c) {
  return image_(u, v, 0, c);
}

template <typename T>
const T& Image<T>::operator()(size_t u, size_t v, size_t c) const {
  return image_(u, v, 0, c);
}

} //\namespace bsfm

#endif
