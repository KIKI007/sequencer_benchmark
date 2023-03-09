# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/wangziqi/Documents/GitHub/sequencer_benchmark/cmake-build-debug/_deps/libigl_imgui_fonts-src"
  "/Users/wangziqi/Documents/GitHub/sequencer_benchmark/cmake-build-debug/_deps/libigl_imgui_fonts-build"
  "/Users/wangziqi/Documents/GitHub/sequencer_benchmark/cmake-build-debug/_deps/libigl_imgui_fonts-subbuild/libigl_imgui_fonts-populate-prefix"
  "/Users/wangziqi/Documents/GitHub/sequencer_benchmark/cmake-build-debug/_deps/libigl_imgui_fonts-subbuild/libigl_imgui_fonts-populate-prefix/tmp"
  "/Users/wangziqi/Documents/GitHub/sequencer_benchmark/cmake-build-debug/_deps/libigl_imgui_fonts-subbuild/libigl_imgui_fonts-populate-prefix/src/libigl_imgui_fonts-populate-stamp"
  "/Users/wangziqi/Documents/GitHub/sequencer_benchmark/cmake-build-debug/_deps/libigl_imgui_fonts-subbuild/libigl_imgui_fonts-populate-prefix/src"
  "/Users/wangziqi/Documents/GitHub/sequencer_benchmark/cmake-build-debug/_deps/libigl_imgui_fonts-subbuild/libigl_imgui_fonts-populate-prefix/src/libigl_imgui_fonts-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/wangziqi/Documents/GitHub/sequencer_benchmark/cmake-build-debug/_deps/libigl_imgui_fonts-subbuild/libigl_imgui_fonts-populate-prefix/src/libigl_imgui_fonts-populate-stamp/${subDir}")
endforeach()
