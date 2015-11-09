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

///////////////////////////////////////////////////////////////////////////////
//
// Defines a status class for success and error codes. The status class should
// be used as a return type, e.g. with the following function:
//
// Status GetElement(vector<int> ints, int index, int& element) {
//   if (index < ints.size()) {
//     element = ints[index];
//     return Status::OK();
//   } else {
//     return Status::OUT_OF_RANGE("Invalid index!");
//   }
// }
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BSFM_UTIL_STATUS_H
#define BSFM_UTIL_STATUS_H

#include <string>

namespace bsfm {

class Status {
 public:
  typedef enum {
    OK = 0,
    CANCELLED = 1,
    UNKNOWN = 2,
    INVALID_ARGUMENT = 3,
    DEADLINE_EXCEEDED = 4,
    NOT_FOUND = 5,
    ALREADY_EXISTS = 6,
    PERMISSION_DENIED = 7,
    RESOURCE_EXHAUSTED = 8,
    FAILED_PRECONDITION = 9,
    ABORTED = 10,
    OUT_OF_RANGE = 11,
    UNIMPLEMENTED = 12,
  } Code;

  Status() : code_(OK) {}
  Status(const Code& code) : code_(code) {}
  Status(const std::string& msg) : code_(UNKNOWN), msg_(msg) {}
  Status(const Code& code, const std::string& msg)
      : code_(code), msg_(msg) {}

  // Static functions to return each of the error codes.
  static Status Ok(const std::string& msg = std::string()) {
    return Status(OK, msg);
  }
  static Status Cancelled(const std::string& msg = std::string()) {
    return Status(CANCELLED, msg);
  }
  static Status Unknown(const std::string& msg = std::string()) {
    return Status(UNKNOWN, msg);
  }
  static Status InvalidArgument(const std::string& msg = std::string()) {
    return Status(INVALID_ARGUMENT, msg);
  }
  static Status DeadlineExceeded(const std::string& msg = std::string()) {
    return Status(DEADLINE_EXCEEDED, msg);
  }
  static Status NotFound(const std::string& msg = std::string()) {
    return Status(NOT_FOUND, msg);
  }
  static Status AlreadyExists(const std::string& msg = std::string()) {
    return Status(ALREADY_EXISTS, msg);
  }
  static Status PermissionDenied(const std::string& msg = std::string()) {
    return Status(PERMISSION_DENIED, msg);
  }
  static Status ResourceExhausted(const std::string& msg = std::string()) {
    return Status(RESOURCE_EXHAUSTED, msg);
  }
  static Status FailedPrecondition(const std::string& msg = std::string()) {
    return Status(FAILED_PRECONDITION, msg);
  }
  static Status Aborted(const std::string& msg = std::string()) {
    return Status(ABORTED, msg);
  }
  static Status OutOfRange(const std::string& msg = std::string()) {
    return Status(OUT_OF_RANGE, msg);
  }
  static Status Unimplemented(const std::string& msg = std::string()) {
    return Status(UNIMPLEMENTED, msg);
  }

  // Easy function to check for success.
  inline bool ok() { return code_ == OK; }

  // Getters.
  inline const Code& ErrorCode() { return code_; }
  inline const std::string& Message() { return msg_; }

 private:
  Code code_;
  std::string msg_;
};  //\class Status

}  //\namespace bsfm

#endif
