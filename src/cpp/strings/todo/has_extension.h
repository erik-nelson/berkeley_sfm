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

#ifndef UTILS_STRINGS_HAS_EXTENSION_H
#define UTILS_STRINGS_HAS_EXTENSION_H

#include <string>

namespace strings {

inline bool HasExtension(const std::string &string,
                         const std::string &extension) {
  // - 1 because extension does not include the '.'
  if (extension.size() > string.size() - 1)
    return false;

  // Extension begins after the last '.', if one exists.
  const size_t last_period = string.find_last_of('.');
  if (last_period == string.npos)
    return false;

  // If the extension has non-zero size, '.' cannot be the last character in the
  // string.
  if (string.back() == '.' && extension.size() > 0)
    return false;

  // If the user prepends their extension query with '.', include the string's
  // last period in the comparison.
  if (extension[0] == '.')
    return string.compare(last_period, extension.size(), extension) == 0;

  return string.compare(last_period + 1, extension.size(), extension) == 0;
}

} //\namespace strings

#endif
