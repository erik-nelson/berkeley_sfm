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

#ifndef UTILS_STRINGS_GET_FILENAME_NO_EXTENSION_H
#define UTILS_STRINGS_GET_FILENAME_NO_EXTENSION_H

#include <string>
#include <vector>

#include "remove_extension.h"
#include "tokenize_file_string.h"

namespace strings {

// Given a path to a file (e.g. /home/erik/test.png), return the last
// '/'-separated token with its extension removed (i.e. test). There are no
// checks to ensure that the resulting string is actually a file.
std::string GetFilenameNoExtension(const std::string &path) {
  // If the path's last character is '/', there is no filename.
  if (path.back() == '/')
    return std::string("");

  std::vector<std::string> path_tokens;
  TokenizeFileString(path, &path_tokens);

  std::string filename = path_tokens.back();
  RemoveExtension(&filename);
  return filename;
}

} //\namespace strings

#endif
