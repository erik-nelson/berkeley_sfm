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

#ifndef UTILS_STRINGS_CREATE_TIMESTAMPED_FILENAME_H
#define UTILS_STRINGS_CREATE_TIMESTAMPED_FILENAME_H

#include <time.h>
#include <string>

#include "join.h"

namespace strings {

// Creates a filename from base_name and extension that includes the current
// date and time. The date and time format is specified by format_string, with a
// default value of %Y%m%d_%H%M%s.
// e.g.
// base_name = filename
// extension = .ext
// format_string = %Y%m%d-%H%M%S
// output: filename_20150702_094750.ext
//
// If the system clock is not able to give us a time, default to
// basename.extension.
inline std::string CreateTimestampedFilename(const std::string &base_name,
                                             const std::string &extension,
                                             const char *format_string =
                                                 "%Y%m%d-%H%M%S") {
  std::string output_filename, date_time_string;

  // Get the current date.
  int size = 12;
  char *date_time = static_cast<char *>(malloc(size));
  date_time[0] = '\0';
  time_t now = time(NULL);

  // If time exists, format date string.
  if (now != -1) {
    // Make sure there is enough room in the buffer to get the full date and
    // time (specified by format_string). If there is not enough room, retry
    // until there is.
    int success = strftime(date_time, size - 1, format_string, gmtime(&now));
    while (!success) {
      free(date_time);
      size += 10;
      date_time = static_cast<char *>(malloc(size));
      date_time[0] = '\0';
      success = strftime(date_time, size - 1, format_string, gmtime(&now));
    }
    date_time_string = Join("_", std::string(date_time));
  }
  free(date_time);
  return Join(base_name, date_time_string, ".", extension);
}

} //\namespace strings

#endif
