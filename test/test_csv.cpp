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

#include <Eigen/Core>
#include <string>
#include <vector>

#include <file/csv_reader.h>
#include <file/csv_writer.h>
#include <strings/join_filepath.h>

#include <gflags/gflags.h>
#include <gtest/gtest.h>

DEFINE_string(csv_file, "test_csv.csv", "Name of the csv used for testing.");

namespace bsfm {
namespace file {

namespace {
const std::string csv_read_file = strings::JoinFilepath(
    BSFM_TEST_DATA_DIR, FLAGS_csv_file.c_str());
const std::string csv_write_file = strings::JoinFilepath(
    BSFM_TEST_DATA_DIR, "test_write.csv");
}

TEST(CsvReader, TestOpen) {
  CsvReader csv_reader;
  EXPECT_FALSE(csv_reader.IsOpen());
  EXPECT_FALSE(csv_reader.Open("fake_file"));
  EXPECT_FALSE(csv_reader.IsOpen());
  EXPECT_TRUE(csv_reader.Open(csv_read_file));
  EXPECT_TRUE(csv_reader.IsOpen());

  CsvReader csv_reader2("fake_file");
  EXPECT_FALSE(csv_reader2.IsOpen());
  EXPECT_TRUE(csv_reader2.Open(csv_read_file));
  EXPECT_TRUE(csv_reader2.IsOpen());

  CsvReader csv_reader3(csv_read_file);
  EXPECT_TRUE(csv_reader3.IsOpen());
}

TEST(CsvReader, TestReadLine) {
  CsvReader csv_reader(csv_read_file);
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
  CsvReader csv_reader(csv_read_file);
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

TEST(CsvWriter, TestOpen) {
  CsvWriter csv_writer;
  EXPECT_FALSE(csv_writer.IsOpen());
  EXPECT_FALSE(csv_writer.Close());
  EXPECT_TRUE(csv_writer.Open(csv_write_file));
  EXPECT_TRUE(csv_writer.IsOpen());

  CsvWriter csv_writer2(csv_write_file);
  EXPECT_TRUE(csv_writer2.IsOpen());
  EXPECT_TRUE(csv_writer.Close());
  EXPECT_FALSE(csv_writer.IsOpen());

  // Delete the file.
  std::remove(csv_write_file.c_str());
}

TEST(CsvWriter, TestWriteLine) {

  // Write a bunch of lines to a csv file.
  CsvWriter csv_writer(csv_write_file);
  EXPECT_TRUE(csv_writer.IsOpen());

  std::vector<int> line1 = {1, 2, 3, 4, 5};
  std::vector<double> line2 = {6.0, 7.0, 8.0, 9.0, 10.0};
  std::vector<std::string> line3 = {"a", "b", "c", "d", "e"};
  Eigen::Matrix<double, 5, 1> line4;
  line4 << 11.0, 12.0, 13.0, 14.0, 15.0;

  EXPECT_TRUE(csv_writer.WriteLine(line1));
  EXPECT_TRUE(csv_writer.WriteLine(line2));
  EXPECT_TRUE(csv_writer.WriteLine(line3));
  EXPECT_TRUE(csv_writer.WriteLine(line4));
  EXPECT_TRUE(csv_writer.Close());

  // Read back the lines from the csv file.
  CsvReader csv_reader(csv_write_file);
  EXPECT_TRUE(csv_reader.IsOpen());
  EXPECT_TRUE(csv_reader.HasMoreLines());

  // First line.
  std::vector<std::string> tokenized_line;
  EXPECT_TRUE(csv_reader.ReadLine(&tokenized_line));
  EXPECT_EQ(line1.size(), tokenized_line.size());
  for (size_t ii = 0; ii < tokenized_line.size(); ++ii) {
    EXPECT_EQ(line1[ii], std::stoi(tokenized_line[ii]));
  }

  // Second line.
  EXPECT_TRUE(csv_reader.ReadLine(&tokenized_line));
  EXPECT_EQ(line2.size(), tokenized_line.size());
  for (size_t ii = 0; ii < tokenized_line.size(); ++ii) {
    EXPECT_EQ(line2[ii], std::stod(tokenized_line[ii]));
  }

  // Third line.
  EXPECT_TRUE(csv_reader.ReadLine(&tokenized_line));
  EXPECT_EQ(line3.size(), tokenized_line.size());
  for (size_t ii = 0; ii < tokenized_line.size(); ++ii) {
    EXPECT_EQ(line3[ii], tokenized_line[ii].c_str());
  }

  // Fourth line.
  EXPECT_TRUE(csv_reader.ReadLine(&tokenized_line));
  EXPECT_EQ(line4.size(), tokenized_line.size());
  for (size_t ii = 0; ii < tokenized_line.size(); ++ii) {
    EXPECT_EQ(line4(ii), std::stod(tokenized_line[ii]));
  }

  // Delete the file.
  std::remove(csv_write_file.c_str());
}

TEST(CsvWriter, TestWriteLines) {

  // Write a bunch of Eigen vectors as lines to a csv file.
  CsvWriter csv_writer(csv_write_file);
  EXPECT_TRUE(csv_writer.IsOpen());

  // Make 1000 random 100-dimensional vectors.
  std::vector<Eigen::VectorXd> lines;
  for (int ii = 0; ii < 1000; ++ii) {
    lines.push_back(Eigen::Matrix<double, 100, 1>::Random());
  }

  // Store the vectors in the csv file.
  EXPECT_TRUE(csv_writer.WriteLines(lines));
  EXPECT_TRUE(csv_writer.Close());

  // Read back the lines from the csv file.
  CsvReader csv_reader(csv_write_file);
  EXPECT_TRUE(csv_reader.IsOpen());
  EXPECT_TRUE(csv_reader.HasMoreLines());

  // Make sure all lines match the original vector.
  int line_number = 0;
  std::vector<std::string> tokenized_line;
  while (csv_reader.HasMoreLines()) {
    EXPECT_TRUE(csv_reader.ReadLine(&tokenized_line));
    EXPECT_EQ(100, tokenized_line.size());
    for (size_t ii = 0; ii < tokenized_line.size(); ++ii) {
      EXPECT_NEAR(lines[line_number](ii),
                  std::stod(tokenized_line[ii]),
                  1e-4);
    }
    line_number++;
  }

  // Delete the file.
  std::remove(csv_write_file.c_str());
}

}  //\namespace file
}  //\namespace bsfm
