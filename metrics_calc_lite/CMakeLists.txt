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
OPTION(USE_IOMP "Enable Intel(R) OpenMP" OFF)

include_directories(include)

if (NOT USE_IPP)
  add_definitions(-DNO_IPP)
else()
  if (NOT DEFINED ENV{IPP_ROOT})
    message(FATAL_ERROR "Env var IPP_ROOT should be set")
  endif()

  set(IPP_ROOT $ENV{IPP_ROOT})
  string(REPLACE "\"" "" IPP_ROOT ${IPP_ROOT})

  if (WIN32)
    file(TO_CMAKE_PATH ${IPP_ROOT}/include IPP8_INC)
    file(TO_CMAKE_PATH ${IPP_ROOT}/lib/intel64 IPP8_LIB)
    set(IPP_LIBS ippcoremt ippsmt ippimt ippcvmt ippccmt)
  else()
    file(TO_CMAKE_PATH ${IPP_ROOT}/include IPP8_INC)
    file(TO_CMAKE_PATH ${IPP_ROOT}/lib/intel64 IPP8_LIB)
    file(TO_CMAKE_PATH ${IPP_ROOT}/em64t/include IPP_INC)
    file(TO_CMAKE_PATH ${IPP_ROOT}/em64t/lib IPP_LIB)
    set(IPP_LIBS ippi ipps ippcc ippcv ippcore)
  endif()
endif()

if (USE_IOMP)
  message("Intel OpenMP is used")
  add_definitions(-D_OPENMP)
  if (NOT DEFINED ENV{IOMP_ROOT})
    message(FATAL_ERROR "Env var IOMP_ROOT should be set")
  endif()

  set(IOMP_ROOT $ENV{IOMP_ROOT})
  string(REPLACE "\"" "" IOMP_ROOT ${IOMP_ROOT})

  if (WIN32)
    file(TO_CMAKE_PATH ${IOMP_ROOT}/compiler/lib/intel64_win IOMP_LIB_DIR)
    set(IOMP_LIB libiomp5md)
  endif()

  find_library(_LIB NAMES ${IOMP_LIB} PATHS ${IOMP_LIB_DIR})
  if (NOT _LIB MATCHES NOTFOUND)
    MESSAGE("IOMP ${IOMP_LIB} library found at ${_LIB}")
    link_directories(${IOMP_LIB_DIR})
  else ()
    message(FATAL_ERROR "Can't find ${IOMP_LIB} library!")
  endif()
  unset(_LIB CACHE )

endif()

add_executable(metrics_calc_lite src/metrics_calc_lite_utils.cpp src/metrics_calc_lite.cpp)

if (USE_IPP)
  find_path(IPP_INCLUDE NAMES ippcore.h PATHS ${IPP8_INC} ${IPP_INC})
  if (NOT IPP_INCLUDE MATCHES NOTFOUND)
    include_directories(${IPP_INCLUDE})
    message(STATUS "IPP headers found in ${IPP_INCLUDE}")
    file(STRINGS ${IPP_INCLUDE}/ippversion.h IPPVERSION)
    foreach(_LINE ${IPPVERSION})
      string(REGEX MATCH "IPP_VERSION_MAJOR.* ([0-9]+)" VERSION ${_LINE})
      if(CMAKE_MATCH_1)
        message("IPP library version major is ${CMAKE_MATCH_1}")
        if(CMAKE_MATCH_1 LESS 9)
          message("Legacy IPP detected, limited set of metrics will be supported")
          add_definitions(-DLEGACY_IPP)
        else()
          message("Modern IPP detected, all metrics will be supported")
        endif()
      endif()
    endforeach()
  else()
    message(FATAL_ERROR "Can't find IPP headers!")
  endif()

  foreach(LIB ${IPP_LIBS})
    find_library(_LIB NAMES ${LIB} ${LIB}_l PATHS ${IPP8_LIB} ${IPP_LIB})
    if (NOT _LIB MATCHES NOTFOUND)
      target_link_libraries(metrics_calc_lite ${_LIB})
      MESSAGE("IPP ${LIB} library found at ${_LIB}")
    else ()
      message(FATAL_ERROR "Can't find ${LIB} library!")
    endif()
    unset(_LIB CACHE )
  endforeach()
endif()

if (USE_IOMP)
  target_link_options(metrics_calc_lite PRIVATE /nodefaultlib:vcomp libiomp5md.lib)
endif()

if (WIN32)
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
endif()

if (UNIX)
  install(TARGETS metrics_calc_lite RUNTIME DESTINATION bin)
endif()
