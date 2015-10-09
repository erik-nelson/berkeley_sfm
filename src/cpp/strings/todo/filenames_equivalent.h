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

#ifndef UTILS_STRINGS_FILENAMES_EQUIVALENT_H
#define UTILS_STRINGS_FILENAMES_EQUIVALENT_H

#include <string>

#include "get_filename.h"

namespace strings {

// Given 2 file paths, determine whether the filenames at the ends of the paths
// are equivalent.
// e.g. the following two paths would return true:
// path1 = /path/to/file.ext
// path2 = ../../to/file.ext
inline bool FilenamesEquivalent(const std::string &path1,
                                const std::string &path2) {
  return GetFilename(path1) == GetFilename(path2);
}

} //\namespace strings

#endif
