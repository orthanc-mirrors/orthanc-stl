# SPDX-FileCopyrightText: 2023 Sebastien Jodogne, UCLouvain, Belgium
# SPDX-License-Identifier: GPL-3.0-or-later

# STL plugin for Orthanc
# Copyright (C) 2023 Sebastien Jodogne, UCLouvain, Belgium
#
# This program is free software: you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.


if (STATIC_BUILD OR NOT USE_SYSTEM_NIFTILIB)
  set(NIFTILIB_SOURCES_DIR ${CMAKE_BINARY_DIR}/nifti_clib-3.0.0)
  DownloadPackage(
    "ee40068103775a181522166e435ee82d"
    "https://third-party.orthanc-labs.com/nifti_clib-3.0.0.tar.gz"
    "${NIFTILIB_SOURCES_DIR}")

  include_directories(
    ${NIFTILIB_SOURCES_DIR}/niftilib
    ${NIFTILIB_SOURCES_DIR}/znzlib
    )

  add_definitions(
    -DHAVE_ZLIB=1
    )

  set(NIFTILIB_SOURCES
    ${NIFTILIB_SOURCES_DIR}/niftilib/nifti1_io.c
    ${NIFTILIB_SOURCES_DIR}/znzlib/znzlib.c
    )

else()
  find_path(NIFTILIB_INCLUDE_DIR
    NAMES nifti1.h
    PATHS
    /usr/include
    /usr/include/nifti
    /usr/local/include
    /usr/local/include/nifti
    )

  check_include_file(${NIFTILIB_INCLUDE_DIR}/nifti1.h HAVE_NIFTILIB_H)
  if (NOT HAVE_NIFTILIB_H)
    message(FATAL_ERROR "Please install the libnifti-dev package")
  endif()

  include_directories(${NIFTILIB_INCLUDE_DIR})

  link_libraries(niftiio znz)
endif()
