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

#ifndef UTILS_STRINGS_TRIM_WHITE_SPACE_H
#define UTILS_STRINGS_TRIM_WHITE_SPACE_H

#include <string>

namespace strings {

inline void TrimWhiteSpace(std::string *string) {
  // Trim white space from the back.
  size_t back_whitespace = string->size() - 1;
  for (; back_whitespace > 0; --back_whitespace) {
    if (string->at(back_whitespace) != ' ')
      break;
  }
  string->erase(back_whitespace + 1, string->size());

  // Trim white space from the front.
  size_t front_whitespace = 0;
  for (; front_whitespace < string->size(); ++front_whitespace) {
    if (string->at(front_whitespace) != ' ')
      break;
  }
  string->erase(0, front_whitespace);
}

} //\namespace strings

#endif

