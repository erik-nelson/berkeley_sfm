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

#include "csv_writer.h"

namespace bsfm {
namespace file {

CsvWriter::CsvWriter() {
  file_.reset(new std::ofstream());
}

CsvWriter::CsvWriter(const std::string& filename) {
  file_.reset(new std::ofstream());
  file_->open(filename.c_str(), std::ios::app | std::ios::out);
}

CsvWriter::~CsvWriter() {}

bool CsvWriter::Open(const std::string& filename) {
  file_->open(filename.c_str(), std::ios::app | std::ios::out);
  return IsOpen();
}

bool CsvWriter::Close() {
  if (!IsOpen())
    return false;

  file_->close();
  return !IsOpen();
}

bool CsvWriter::IsOpen() const {
  return file_->is_open();
}

bool CsvWriter::WriteLine(const std::vector<int>& data, char delimiter) {
  if (!IsOpen()) {
    return false;
  }

  for (size_t ii = 0; ii < data.size() - 1; ++ii) {
    *file_ << data[ii] << delimiter;
  }
  *file_ << data.back() << std::endl;
  return true;
}

bool CsvWriter::WriteLine(const std::vector<double>& data, char delimiter) {
  if (!IsOpen()) {
    return false;
  }

  for (size_t ii = 0; ii < data.size() - 1; ++ii) {
    *file_ << data[ii] << delimiter;
  }
  *file_ << data.back() << std::endl;
  return true;
}

bool CsvWriter::WriteLine(const std::vector<std::string>& data, char delimiter) {
  if (!IsOpen()) {
    return false;
  }

  for (size_t ii = 0; ii < data.size() - 1; ++ii) {
    *file_ << data[ii] << delimiter;
  }
  *file_ << data.back() << std::endl;
  return true;
}

bool CsvWriter::WriteLine(const Eigen::VectorXd& data, char delimiter) {
  if (!IsOpen()) {
    return false;
  }

  for (size_t ii = 0; ii < data.size() - 1; ++ii) {
    *file_ << data(ii) << delimiter;
  }
  *file_ << data(data.size() - 1) << std::endl;
  return true;
}

bool CsvWriter::WriteLines(const std::vector<Eigen::VectorXd>& data, char delimiter) {
  for (const auto& vector : data)
    if (!WriteLine(vector, delimiter))
      return false;
  return true;
}

}  //\namespace file
}  //\namespace bsfm
