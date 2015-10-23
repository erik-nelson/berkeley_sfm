/*
 * Copyright (c) 2015, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: Erik Nelson            ( eanelson@eecs.berkeley.edu )
 *          David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

#include "progress_bar.h"

namespace bsfm {
namespace util {

ProgressBar::ProgressBar(const std::string &message, double max)
    : message_(message), progress_(0), max_(max), last_message_length_(0) {
  timer_.Tic();
}

ProgressBar::~ProgressBar() {}

std::string ProgressBar::Update(double progress) {
  progress_ = progress;
  double elapsed = timer_.Toc();
  return PrintProgress(elapsed);
}

void ProgressBar::Reset(double max) {
  max_ = max;
  progress_ = 0;
  timer_.Tic();
}

double ProgressBar::GetMax() const {
  return max_;
}

double ProgressBar::GetProgress() const {
  return progress_;
}

double ProgressBar::GetPercentage() const {
  return progress_ / max_ * 100.;
}

double ProgressBar::GetRemainingSeconds() {
  return ElapsedToRemaining(timer_.Toc());
}

double ProgressBar::ElapsedToRemaining(double elapsed) const {
  if (progress_ == 0.0)
    return 0.0;

  return elapsed * (max_ / progress_ - 1.0);
}

std::string ProgressBar::FormatRemaining(double elapsed) const {
  // Elapsed is in seconds. Convert to HH:MM:SS.
  int hours = static_cast<int>(elapsed) / 3600;
  int minutes = (static_cast<int>(elapsed) - 3600 * hours) / 60;
  int seconds = static_cast<int>(elapsed) % 60;

  std::stringstream ss;
  ss << std::setfill('0') << std::setw(2) << hours << "h:";
  ss << std::setfill('0') << std::setw(2) << minutes << "m:";
  ss << std::setfill('0') << std::setw(2) << seconds << "s";

  return ss.str();
}

std::string ProgressBar::ProgressBarString() const {
  std::stringstream progress_bar;
  std::string equals(floor((bar_length_ - 3) * progress_ / max_), '=');
  progress_bar << "[" << equals << ">";

  std::string remainder(bar_length_ - 3 - equals.length(), ' ');
  progress_bar << remainder << "]";

  return progress_bar.str();
}

std::string ProgressBar::PrintProgress(double elapsed) {
  double remaining = ElapsedToRemaining(elapsed);
  std::string remaining_string = FormatRemaining(remaining);

  std::stringstream ss;
  ss << "   " << message_ << ": " << progress_ << "/" << max_ << " ("
     << std::fixed << std::setprecision(2) << GetPercentage()
     << "%%), remaining: " << remaining_string << ".";

  std::string progress_bar = ProgressBarString();

  size_t num_backspaces = bar_length_ + last_message_length_;
  printf("%s", std::string(num_backspaces, '\b').c_str());

  printf("%s", progress_bar.c_str());
  printf("%s", ss.str().c_str());

  last_message_length_ = ss.str().length();

  return ss.str();
}

}  //\namespace util
}  //\namespace bsfm
