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

#ifndef UTILS_STRINGS_TOKENIZE_H
#define UTILS_STRINGS_TOKENIZE_H

#include <string>
#include <vector>

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

} //\namespace strings

#endif
