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

#ifndef __STRING_STRING_UTILS_H__
#define __STRING_STRING_UTILS_H__

#include <string>
#include <vector>

namespace string {

bool IsUpper(const std::string &string);

bool IsLower(const std::string &string);

bool IsNumber(const std::string &string);

bool IsAlphabetic(const std::string &string);

bool HasPrefix(const std::string &string, const std::string &prefix);

bool HasSuffix(const std::string &string, const std::string &suffix);

bool HasExtension(const std::string &string, const std::string &extension);

void ToUpper(std::string *string);

void ToLower(std::string *string);

bool ReplaceAll(std::string *string, const std::string &replace_this,
                const std::string &with_this);

bool ReplaceAllChar(std::string *string, char replace_this, char with_this);

bool RemoveNumbers(std::string *string);

// base = hello_world, extension = .png
// if include_date is true --> hello_world_2015_06_04_9-45-16.png
// otherwise               --> hello_world_9-45-16.png
std::string CreateTimestampedFilename(const std::string &base,
                                      const std::string &extension,
                                      const bool include_date = true);

// users/erik/yodawg.jpg --> erik
std::string GetDir(const std::string &string);

// users/erik/yodawg.jpg --> users/erik
std::string GetPath(const std::string &string);

// users/erik/yodawg.jpg --> users
std::string GetRootDir(const std::string &string);

// users/erik/yodawg.jpg --> yodawg.jpg
std::string GetFileName(const std::string &string);

// users/erik/yodawg.jpg --> yodawg
std::string GetFileNameNoExtension(const std::string &string);

// /users/erik/yodawg.jpg --> /users/erik/yodawg
// yodawg.jpg             --> yodawg
bool RemoveExtension(std::string *string);

bool RemovePrefix(std::string *string);

bool RemoveSuffix(std::string *string);

// /users/erik/yodawg.jpg --> jpg
std::string GetExtension(const std::string &string);

void ReplaceExtension(std::string *string, const std::string &new_extension);

std::string JoinFilePath(const std::vector<std::string> &strings);

bool IsAbsolutePath(const std::string &string);

bool Tokenize(const std::string &string, const std::vector<char> &delimiters,
              std::vector<std::string> *tokens);

bool TokenizeFileString(const std::string &file_string,
                        std::vector<std::string> *tokens);

void TrimWhiteSpace(std::string *string);

std::string StringPrintf(const std::string &string);

std::string StringScanf(const std::string &string);

size_t FindFirstOf(const std::string &string);

size_t FindLastOf(const std::string &string);

std::vector<size_t> FindAllOf(const std::string &string);

void RemoveFirstOf(std::string *string, const std::string &remove_this);

void RemoveLastOf(std::string *string, const std::string &remove_this);

void RemoveAll(std::string *string, const std::string &remove_this);

std::string Substring(const std::string &string);

// string = hello_world, after_this = hello_
// --> world
std::string After(const std::string &string, const std::string &after_this);

std::string Before(const std::string &string, const std::string &before_this);

void Append(std::string *base, const std::string &append);

void Append(std::string *first, const std::vector<std::string> &second);

bool TrimFromBackToLength(std::string *string, size_t length);

bool TrimFromFrontToLength(std::string *string, size_t length);

// bool TrimFromPosToLength(std::string *string, size_t start, size_t length);

bool TrimFromTo(std::string *string, size_t from, size_t to);

bool FileNamesEquivalent(const std::string &one, const std::string &two);

bool ExtensionsEquivalent(const std::string &one, const std::string &two);

bool FilePathsEquivalent(const std::string &one, const std::string &two);

void Reverse(std::string *string);
} //\namespace string

#endif
