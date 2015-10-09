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

#ifndef UTILS_STRINGS_REPLACE_ALL_H
#define UTILS_STRINGS_REPLACE_ALL_H

#include <string>

namespace strings {

inline void ReplaceAll(const std::string &replace_this,
                       const std::string &with_this, std::string *string) {
  string->replace(string->begin(), string->end(), replace_this.c_str(),
                  with_this.c_str());
}

inline void ReplaceAll(char replace_this, char with_this, std::string *string) {
  string->replace(string->begin(), string->end(), replace_this, with_this);
}

} //\namespace strings

#endif
