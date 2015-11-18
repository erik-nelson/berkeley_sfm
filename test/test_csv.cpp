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

#include <file/csv_reader.h>
#include <strings/join_filepath.h>

#include <gflags/gflags.h>
#include <gtest/gtest.h>

DEFINE_string(csv_file, "test_csv.csv", "Name of the csv used for testing.");

namespace bsfm {
namespace file {

namespace {
const std::string csv_file = strings::JoinFilepath(
    BSFM_TEST_DATA_DIR, FLAGS_csv_file.c_str());
}

TEST(CsvReader, TestOpen) {
  CsvReader csv_reader;
  EXPECT_FALSE(csv_reader.IsOpen());
  EXPECT_FALSE(csv_reader.Open(""));
  EXPECT_FALSE(csv_reader.IsOpen());
  EXPECT_TRUE(csv_reader.Open(csv_file));
  EXPECT_TRUE(csv_reader.IsOpen());

  CsvReader csv_reader2("");
  EXPECT_FALSE(csv_reader2.IsOpen());
  EXPECT_TRUE(csv_reader2.Open(csv_file));
  EXPECT_TRUE(csv_reader2.IsOpen());

  CsvReader csv_reader3(csv_file);
  EXPECT_TRUE(csv_reader3.IsOpen());
}

TEST(CsvReader, TestReadLine) {
  CsvReader csv_reader(csv_file);
  std::vector<std::string> tokenized_line;

  int line_number = 0;
  EXPECT_TRUE(csv_reader.HasMoreLines());
  while (csv_reader.HasMoreLines()) {
    csv_reader.ReadLine(&tokenized_line);
    EXPECT_EQ(5, tokenized_line.size());

    for (int ii = 0; ii <= 4; ++ii) {
      EXPECT_EQ(1, tokenized_line[ii].size());
      const char expected = 'a' + (ii + line_number * 5);
      EXPECT_EQ(expected, tokenized_line[ii][0]);
    }

    if (line_number < 4) {
      EXPECT_TRUE(csv_reader.HasMoreLines());
    } else {
      EXPECT_FALSE(csv_reader.HasMoreLines());
    }

    line_number++;
  }
  EXPECT_FALSE(csv_reader.HasMoreLines());
}

TEST(CsvReader, TestReadFile) {
  CsvReader csv_reader(csv_file);
  std::vector<std::vector<std::string>> tokenized_lines;

  EXPECT_TRUE(csv_reader.HasMoreLines());
  EXPECT_TRUE(csv_reader.ReadFile(&tokenized_lines));
  EXPECT_FALSE(csv_reader.HasMoreLines());
  ASSERT_EQ(5, tokenized_lines.size());

  for (int line = 0; line < tokenized_lines.size(); ++line) {
    for (int token = 0; token < tokenized_lines[line].size(); ++token) {
      ASSERT_EQ(1, tokenized_lines[line][token].size());

      const char expected = 'a' + (token + line * 5);
      EXPECT_EQ(expected, tokenized_lines[line][token][0]);
    }
  }
}

}  //\namespace file
}  //\namespace bsfm
