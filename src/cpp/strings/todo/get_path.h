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

#ifndef UTILS_STRINGS_GET_PATH_H
#define UTILS_STRINGS_GET_PATH_H

#include <string>
#include <vector>

#include "join_filepath.h"
#include "prepend_slash.h"
#include "tokenize_file_string.h"

namespace strings {

// Given a path to a file (e.g. /home/erik/test.png), return the filepath up to
// bu not including the last '/'.
std::string GetPath(const std::string &path) {
  // If the input path is empty, return an empty string.
  if (path.empty())
    return std::string("");

  // If the input path ends in a '/', it already does not have a filename.
  if (path.back() == '/')
    return path;

  std::vector<std::string> path_tokens;
  TokenizeFileString(path, &path_tokens);

  // If there are no tokens and the path begins with '/', return '/'. Otherwise
  // the input name is a file and therefore has no path.
  if (path_tokens.size() == 0)
    return path[0] == '/' ? std::string("/") : std::string("");

  // Remove the last token from the tokenized path.
  path_tokens.erase(path_tokens.end());

  // Join the remaining tokens, add a prepending slash if one existed in the
  // input path, and return the result.
  std::string output_path = JoinFilepath(path_tokens);
  if (path[0] == '/')
    return PrependSlash(output_path);

  return output_path;
}

} //\namespace strings

#endif
