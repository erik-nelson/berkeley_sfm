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

#ifndef UTILS_STRINGS_GET_EXTENSION_H
#define UTILS_STRINGS_GET_EXTENSION_H

#include <string>

namespace strings {

inline std::string GetExtension(const std::string &string) {

  // Extension begins after the last '.', if one exists.
  const size_t last_period = string.find_last_of('.');
  if (last_period == string.npos)
    return std::string("");

  // Return an empty extension if the last charaacter in the string is a period.
  if (string.back() == '.')
    return std::string("");

  return string.substr(last_period + 1, string.size() - last_period - 1);
}

} //\namespace strings

#endif
