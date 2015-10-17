# Berkeley S(tructure) F(rom) M(otion)

[![Build Status](https://travis-ci.org/erik-nelson/berkeley_sfm.svg?branch=master)](https://travis-ci.org/erik-nelson/berkeley_sfm)
[![License](https://img.shields.io/badge/license-BSD-blue.svg)](LICENSE)

Berkeley SFM is a structure from motion library created to explore new solutions to challenging 3D reconstruction problems (large scale, indoors, incremental, homogeneously textured, etc.). It is developed by Erik Nelson and David Fridovich-Keil, members of the Berkeley Vision and Learning Center ([BVLC](http://bvlc.eecs.berkeley.edu)).

## Overview
This repository is entirely written in C++, and is structured around the CMAKE cross-compilation paradigm. All source code is in the berkeley_sfm/src/cpp/ directory. We have also written extensive unit tests, which are automatically compiled and tested in continuous integration. The source code for these tests is stored in the berkeley_sfm/test directory, and the test executable will be compiled into berkeley_sfm/bin/run_tests.

## Build Instructions
We follow the standard CMAKE build scheme. Just download the repository and from the top directory type:

```bash
mkdir build && cd build && cmake .. && make
```

We have tested the build process on Mac OSX Yosemite and El Capitan, and on Ubuntu 14.04 Trusty. One difficulty we did encounter is the potential conflict between multiple copies of OpenCV on a system. For example, if you run Anaconda's (and/or Homebrew's) version of OpenCV it may not be compiled properly such that it will link to our repository. If you run into this problem, we suggest uninstalling all versions of OpenCV and rebuilding it from source with the default compiler on your system (not Homebrew's version of gcc/g++).
