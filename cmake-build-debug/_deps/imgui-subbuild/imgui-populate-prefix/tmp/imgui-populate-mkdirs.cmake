# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/wangziqi/Documents/GitHub/sequencer_benchmark/cmake-build-debug/_deps/imgui-src"
  "/Users/wangziqi/Documents/GitHub/sequencer_benchmark/cmake-build-debug/_deps/imgui-build"
  "/Users/wangziqi/Documents/GitHub/sequencer_benchmark/cmake-build-debug/_deps/imgui-subbuild/imgui-populate-prefix"
  "/Users/wangziqi/Documents/GitHub/sequencer_benchmark/cmake-build-debug/_deps/imgui-subbuild/imgui-populate-prefix/tmp"
  "/Users/wangziqi/Documents/GitHub/sequencer_benchmark/cmake-build-debug/_deps/imgui-subbuild/imgui-populate-prefix/src/imgui-populate-stamp"
  "/Users/wangziqi/Documents/GitHub/sequencer_benchmark/cmake-build-debug/_deps/imgui-subbuild/imgui-populate-prefix/src"
  "/Users/wangziqi/Documents/GitHub/sequencer_benchmark/cmake-build-debug/_deps/imgui-subbuild/imgui-populate-prefix/src/imgui-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/wangziqi/Documents/GitHub/sequencer_benchmark/cmake-build-debug/_deps/imgui-subbuild/imgui-populate-prefix/src/imgui-populate-stamp/${subDir}")
endforeach()
