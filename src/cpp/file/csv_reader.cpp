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

#include <glog/logging.h>

#include "csv_reader.h"

#include "../strings/tokenize.h"

namespace bsfm {
namespace file {

CsvReader::CsvReader() {
  file_.reset(new std::ifstream());
}

CsvReader::CsvReader(const std::string& filename) {
  file_.reset(new std::ifstream());
  file_->open(filename.c_str(), std::ifstream::in);
}

CsvReader::~CsvReader() {}

bool CsvReader::Open(const std::string& filename) {
  file_->open(filename.c_str(), std::ifstream::in);
  return file_->is_open();
}

bool CsvReader::IsOpen() const {
  return file_->is_open();
}

bool CsvReader::HasMoreLines() const {
  return file_->peek() != std::ifstream::traits_type::eof();
}

bool CsvReader::ReadLine(CsvReader::Line* line, char delimiter) const {
  CHECK_NOTNULL(line)->clear();

  // Read a line into a string.
  std::string line_string;
  if (!std::getline(*file_, line_string)) {
    return false;
  }

  // Tokenize the string.
  CsvReader::Line tokenized_line;
  ::bsfm::strings::Tokenize(line_string, delimiter, line);

  return true;
}

bool CsvReader::ReadFile(CsvReader::File* file, char delimiter) const {
  CHECK_NOTNULL(file)->clear();

  std::string line;
  while (std::getline(*file_, line)) {
    // Tokenize the string.
    CsvReader::Line tokenized_line;
    ::bsfm::strings::Tokenize(line, delimiter, &tokenized_line);

    file->push_back(tokenized_line);
  }

  // Did we read the entire file?
  return !HasMoreLines();
}

}  //\namespace file
}  //\namespace bsfm
