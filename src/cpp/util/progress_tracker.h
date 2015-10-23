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

#ifndef BSFM_UTIL_PROGRESS_TRACKER_H
#define BSFM_UTIL_PROGRESS_TRACKER_H

#include <cmath>
#include <iomanip>
#include <sstream>
#include <string.h>

#include "timer.h"

namespace bsfm {
namespace util {

class ProgressTracker {
public:
 // Initialize a new progress tracker with a print message and a maximum number
 // of items that we are tracking towards. The tracker will print 100% when we
 // our progress reaches 'max'.
  ProgressTracker(const std::string& message, double max);
  ~ProgressTracker();

  // Update the progress tracker with our current value.
  std::string Update(double progress);

  // Reset out progress with a new maximum.
  void Reset(double max);

  // Getters.
  double GetMax() const;
  double GetProgress() const;
  double GetPercentage() const;
  double GetRemainingSeconds();

private:
  // Convert from an elapsed time to a remaining amount of time, based on how
  // long it has taken so far to get to our current progress.
  double ElapsedToRemaining(double elapsed) const;

  // Formatting and string functions.
  std::string FormatRemaining(double elapsed) const;
  std::string ProgressBarString() const;
  std::string PrintProgress(double elapsed);

  // Private members used for storing internal state, such as last message
  // length, time, etc.
  Timer timer_;
  std::string message_;

  double progress_;
  double max_;

  size_t last_message_length_;
  static const size_t bar_length_ = 40;

}; //\class ProgressTracker

} //\namespace util
} //\namespace bsfm

#endif
