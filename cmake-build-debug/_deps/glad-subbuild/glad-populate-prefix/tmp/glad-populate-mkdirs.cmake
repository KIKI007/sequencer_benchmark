# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/wangziqi/Documents/GitHub/sequencer_benchmark/cmake-build-debug/_deps/glad-src"
  "/Users/wangziqi/Documents/GitHub/sequencer_benchmark/cmake-build-debug/_deps/glad-build"
  "/Users/wangziqi/Documents/GitHub/sequencer_benchmark/cmake-build-debug/_deps/glad-subbuild/glad-populate-prefix"
  "/Users/wangziqi/Documents/GitHub/sequencer_benchmark/cmake-build-debug/_deps/glad-subbuild/glad-populate-prefix/tmp"
  "/Users/wangziqi/Documents/GitHub/sequencer_benchmark/cmake-build-debug/_deps/glad-subbuild/glad-populate-prefix/src/glad-populate-stamp"
  "/Users/wangziqi/Documents/GitHub/sequencer_benchmark/cmake-build-debug/_deps/glad-subbuild/glad-populate-prefix/src"
  "/Users/wangziqi/Documents/GitHub/sequencer_benchmark/cmake-build-debug/_deps/glad-subbuild/glad-populate-prefix/src/glad-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/wangziqi/Documents/GitHub/sequencer_benchmark/cmake-build-debug/_deps/glad-subbuild/glad-populate-prefix/src/glad-populate-stamp/${subDir}")
endforeach()
