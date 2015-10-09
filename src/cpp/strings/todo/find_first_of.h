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

#ifndef UTILS_STRINGS_FIND_FIRST_OF_H
#define UTILS_STRINGS_FIND_FIRST_OF_H

#include <string>

namespace strings {

inline size_t FindFirstOf(const std::string &haystack,
                          const std::string &needle) {
  return haystack.find(needle);
}

inline size_t FindFirstOf(const std::string &haystack, char needle) {
  const std::string needle_string = { needle };
  return FindFirstOf(haystack, needle_string);
}

} //\namespace strings

#endif
