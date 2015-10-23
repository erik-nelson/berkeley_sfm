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

#include "timer.h"

#include <stdio.h>

namespace bsfm {
namespace util {

Timer::Timer() {
  Tic();
}

Timer::~Timer() {}

 // Start the timer.
double Timer::Tic() {
  start_ = Clock::now();
  double start_double =
      duration_cast<nanoseconds>(start_.time_since_epoch()).count() * 1e-9;
  return start_double;
}

double Timer::Ptic(const std::string &prefix) {
  start_ = Clock::now();

  if (!prefix.empty()) {
    fputs(prefix.c_str(), stdout);
    printf(": ");
  }

  double start_double =
      duration_cast<nanoseconds>(start_.time_since_epoch()).count() * 1e-9;
  printf("Begin: %lf (s)\n", start_double);
  return start_double;
}

double Timer::Toc() {
  Clock::duration elapsed = Clock::now() - start_;
  double elapsed_double = duration_cast<nanoseconds>(elapsed).count() * 1e-9;
  return elapsed_double;
}

double Timer::Ptoc(const std::string &prefix) {
  if (!prefix.empty()) {
    fputs(prefix.c_str(), stdout);
    printf(": ");
  }

  Clock::duration elapsed = Clock::now() - start_;
  double elapsed_double = duration_cast<nanoseconds>(elapsed).count() * 1e-9;

  printf("Elapsed: %lf (s)\n", elapsed_double);
  return elapsed_double;
}

 }  //\namespace util
 }  //\namespace utils
