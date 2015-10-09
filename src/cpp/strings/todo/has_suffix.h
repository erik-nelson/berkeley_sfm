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

#ifndef UTILS_STRINGS_HAS_SUFFIX_H
#define UTILS_STRINGS_HAS_SUFFIX_H

#include <string>

namespace strings {

inline bool HasSuffix(const std::string& string, const std::string& suffix) {
  const size_t suffix_begin = string.size() - suffix.size();
  return string.size() >= suffix.size() &&
         string.compare(suffix_begin, suffix.size(), suffix) == 0;
}

} //\namespace strings

#endif
