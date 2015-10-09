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

#ifndef UTILS_STRINGS_REMOVE_LAST_OF_H
#define UTILS_STRINGS_REMOVE_LAST_OF_H

#include <algorithm>
#include <string>

namespace strings {

// Removes last instance of needle from haystack. Returns true if needle was
// found in haystack.
inline bool RemoveLastOf(const std::string &needle, std::string *haystack) {
  size_t needle_pos = haystack->rfind(needle);
  if (needle_pos != std::string::npos) {
    haystack->erase(needle_pos, needle.size());
    return true;
  }

  return false;
}

inline bool RemoveLastOf(char needle, std::string *haystack) {
  const std::string needle_string = { needle };
  return RemoveLastOf(needle_string, haystack);
}

} //\namespace strings

#endif
