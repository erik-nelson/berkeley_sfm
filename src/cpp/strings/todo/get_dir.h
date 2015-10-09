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

#ifndef UTILS_STRINGS_GET_DIR_H
#define UTILS_STRINGS_GET_DIR_H

#include <string>
#include <vector>

#include "tokenize_file_string.h"

namespace strings {

// Given a path to a file (e.g. /home/erik/test.png), return local directory
// (e.g. "erik").
std::string GetDir(const std::string &path) {
  std::vector<std::string> path_tokens;
  TokenizeFileString(path, &path_tokens);

  // If the input path ends in a '/', it already does not have a filename. The
  // last element in the tokens vector is the local directory.
  // e.g. this/is/a/path/ --> return "path"
  if (path.back() == '/')
    return path_tokens.back();

  // Otherwise, there must be at least 2 tokens to have a local directory.
  // e.g. if the path is "/filename", there is no way to tell if it is a file or
  // directory. Assume it is a file.
  if (path_tokens.size() < 2)
    return std::string("");

  // If there are at least 2 tokens, return the second-to-last.
  // e.g. local_directory/filename.ext --> local_directory
  return path_tokens[path_tokens.size()-2];
}

} //\namespace strings

#endif
