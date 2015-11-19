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
// This file defines a Comma Separated Value (CSV) reader class, which loads
// CSV files, parses them, and returns their values.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_FILE_CSV_READER_H
#define BSFM_FILE_CSV_READER_H

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "../util/disallow_copy_and_assign.h"

namespace bsfm {
namespace file {

class CsvReader {
 public:
  typedef std::string Token;
  typedef std::vector<Token> Line;
  typedef std::vector<Line> File;

  CsvReader();
  CsvReader(const std::string& filename);

  ~CsvReader();

  // Open a file for reading.
  bool Open(const std::string& filename);

  // Check if the file is open.
  bool IsOpen() const;

  // Check if the file has more lines to read.
  bool HasMoreLines() const;

  // Read the next line, splitting all delimited values into tokens.
  bool ReadLine(Line* line, char delimiter = ',') const;

  // Read an entire file, splitting all delimited values into tokens.
  bool ReadFile(File* file, char delimiter = ',') const;

 private:
  DISALLOW_COPY_AND_ASSIGN(CsvReader)

  std::shared_ptr<std::ifstream> file_;
};  //\class CsvReader

}  //\namespace file
}  //\namespace bsfm

#endif
