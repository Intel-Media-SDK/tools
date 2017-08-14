# Copyright (c) 2017 Intel Corporation
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

cmake_minimum_required(VERSION 2.8)

project(metrics_calc_lite)

OPTION(USE_IPP "Enable Intel(R) IPP optimizations" OFF)

include_directories(include)

if (NOT USE_IPP)
  add_definitions(-DNO_IPP)
  add_executable(metrics_calc_lite src/metrics_calc_lite_utils.cpp src/metrics_calc_lite.cpp)
else()
  if (NOT DEFINED ENV{IPP_ROOT})
    message(FATAL_ERROR "Env var IPP_ROOT should be set")
  endif()

  set(IPP_ROOT $ENV{IPP_ROOT})
  if (WIN32)
    if (CMAKE_GENERATOR MATCHES "(Win64|IA64)")
      set(IPP_ROOT ${IPP_ROOT}/em64t)
    else()
      set(IPP_ROOT ${IPP_ROOT}/ia32)
    endif()
    set(IPP_LIBS ippcoremt ippsmt ippimt ippcvmt ippccmt)
  else()
    set(IPP_ROOT ${IPP_ROOT}/em64t)
    set(IPP_LIBS ippi_l ipps_l ippcc_l ippcv_l ippcore_l)
  endif()

  set(IPP_FOUND OFF)
  find_path(IPP_INCLUDE ippcore.h PATHS ${IPP_ROOT}/include)
  find_library(IPP_LIBRARY ${IPP_LIBS} PATHS ${IPP_ROOT}/lib)
  if (NOT IPP_INCLUDE MATCHES NOTFOUND)
    if (NOT IPP_LIBRARY MATCHES NOTFOUND)
      set(IPP_FOUND ON)
      include_directories(${IPP_INCLUDE})
      link_directories(${IPP_ROOT}/lib)
      message(STATUS "IPP found in ${IPP_ROOT}")
    endif()
  endif()
  
  if (NOT IPP_FOUND)
    message(FATAL_ERROR "Can't find IPP")
  endif()

  add_executable(metrics_calc_lite src/metrics_calc_lite_utils.cpp src/metrics_calc_lite.cpp)
  target_link_libraries(metrics_calc_lite ${IPP_LIBS})
endif()

if (UNIX)
  install(TARGETS metrics_calc_lite RUNTIME DESTINATION bin)
endif()
