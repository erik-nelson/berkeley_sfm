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

#ifndef BSFM_STRINGS_TOKENIZE_H
#define BSFM_STRINGS_TOKENIZE_H

#include <string>
#include <vector>

namespace bsfm {
namespace strings {

// Tokenize a string. All chars in the delimiters string are considered.
inline void Tokenize(const std::string &string, const std::string &delimiters,
                     std::vector<std::string> *tokens) {
  size_t prev = 0, next = 0;
  while ((next = string.find_first_of(delimiters, prev)) != std::string::npos) {
    if (next - prev != 0) {
      tokens->push_back(string.substr(prev, next - prev));
    }
    prev = next + 1;
  }

  if (prev < string.size()) {
    tokens->push_back(string.substr(prev));
  }
}

// Tokenize using all elements in a vector of delimiters.
inline void Tokenize(const std::string &string,
                     const std::vector<char> &delimiters,
                     std::vector<std::string> *tokens) {
  std::string delimiters_string;
  for (const auto &element : delimiters)
    delimiters_string.push_back(element);

  Tokenize(string, delimiters_string, tokens);
}

// Tokenize with a single delimiter.
inline void Tokenize(const std::string &string, char delimiter,
                     std::vector<std::string> *tokens) {
  std::string delimiter_string = { delimiter };
  Tokenize(string, delimiter_string, tokens);
}

}  //\namespace strings
}  //\namespace bsfm

#endif
