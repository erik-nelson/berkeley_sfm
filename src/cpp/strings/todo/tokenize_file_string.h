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

#ifndef UTILS_STRINGS_TOKENIZE_FILE_STRING_H
#define UTILS_STRINGS_TOKENIZE_FILE_STRING_H

#include <string>
#include <vector>

#include "tokenize.h"

namespace strings {

// Tokenize a file string using '/' as a delimiter. Returns true if the input
// string contained at least one instance of '/'. An output vector of the
// tokenized directories and filenames is stored in tokens.
inline void TokenizeFileString(const std::string &string,
                               std::vector<std::string> *tokens) {
  Tokenize(string, '/', tokens);
}

} //\namespace strings

#endif
