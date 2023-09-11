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


if (NOT STATIC_BUILD AND USE_SYSTEM_VTK)
  find_package(VTK REQUIRED COMPONENTS
    vtkCommonDataModel
    vtkFiltersCore
    vtkImagingCore
    )

  include_directories(${VTK_INCLUDE_DIRS})

  # Add a void target
  add_custom_target(VTK)

else()
  set(VTK_MAJOR 7)
  set(VTK_MINOR 1)
  set(VTK_REVISION 1)
  set(VTK_MD5 "daee43460f4e95547f0635240ffbc9cb")

  set(VTK_SOURCES_DIR "${CMAKE_BINARY_DIR}/VTK-${VTK_MAJOR}.${VTK_MINOR}.${VTK_REVISION}")

  DownloadPackage(
    "${VTK_MD5}"
    "https://orthanc.uclouvain.be/third-party-downloads/VTK-${VTK_MAJOR}.${VTK_MINOR}.${VTK_REVISION}.tar.gz"
    "${VTK_SOURCES_DIR}")

  if (CMAKE_TOOLCHAIN_FILE)
    # Take absolute path to the toolchain
    get_filename_component(TMP ${CMAKE_TOOLCHAIN_FILE} REALPATH BASE ${CMAKE_SOURCE_DIR}/..)
    list(APPEND VTKCMakeFlags
      -DCMAKE_TOOLCHAIN_FILE=${TMP}
      -DLSB_CC=${LSB_CC}
      -DLSB_CXX=${LSB_CXX}
      )
  endif()

  cmake_host_system_information(RESULT NumberOfPhysicalCores QUERY NUMBER_OF_PHYSICAL_CORES)

  include(ExternalProject)

  if (CMAKE_CROSSCOMPILING)
    # First, generate the "VTKCompileToolsConfig.cmake" file on the host
    # computer. This is necessary before cross-compiling. Explanations:
    # https://cmake.org/cmake/help/book/mastering-cmake/chapter/Cross%20Compiling%20With%20CMake.html#cross-compiling-a-complex-project-vtk
    externalproject_add(VTKCompileTools
      SOURCE_DIR "${VTK_SOURCES_DIR}"

      CMAKE_ARGS
      -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
      -DCMAKE_CXX_COMPILER=g++
      -DCMAKE_C_COMPILER=gcc

      BUILD_COMMAND
      ${CMAKE_COMMAND} --build <BINARY_DIR> --config <CONFIG> --parallel ${NumberOfPhysicalCores} -t
      VTKCompileToolsConfig.cmake

      INSTALL_COMMAND ""  # Skip the install step
      )

    ExternalProject_Get_Property(VTKCompileTools binary_dir)
    list(APPEND VTKCMakeFlags
      -DVTKCompileTools_DIR=${binary_dir}
      )

    list(APPEND VTKCMakeFlags
      -DCMAKE_REQUIRE_LARGE_FILE_SUPPORT=ON
      -DCMAKE_REQUIRE_LARGE_FILE_SUPPORT__TRYRUN_OUTPUT=""
      -DKWSYS_LFS_WORKS=ON
      -DKWSYS_LFS_WORKS__TRYRUN_OUTPUT=""
      -DFILE_OFFSET_BITS=64

      # This disables HDF5, which is not used in Orthanc
      -DVTK_USE_SYSTEM_HDF5=OFF

      # Some raw guesses to make CMake happy, could be incorrect!
      -DHAVE_IOEO_EXITCODE=0
      -DH5_FP_TO_INTEGER_OVERFLOW_WORKS_RUN=0
      -DH5_FP_TO_INTEGER_OVERFLOW_WORKS_RUN__TRYRUN_OUTPUT=
      -DH5_FP_TO_ULLONG_ACCURATE_RUN=0
      -DH5_FP_TO_ULLONG_ACCURATE_RUN__TRYRUN_OUTPUT=
      -DH5_FP_TO_ULLONG_RIGHT_MAXIMUM_RUN=0
      -DH5_FP_TO_ULLONG_RIGHT_MAXIMUM_RUN__TRYRUN_OUTPUT=
      -DH5_LDOUBLE_TO_INTEGER_WORKS_RUN=0
      -DH5_LDOUBLE_TO_INTEGER_WORKS_RUN__TRYRUN_OUTPUT=
      -DH5_LDOUBLE_TO_LLONG_ACCURATE_RUN=0
      -DH5_LDOUBLE_TO_LLONG_ACCURATE_RUN__TRYRUN_OUTPUT=
      -DH5_LDOUBLE_TO_UINT_ACCURATE_RUN=0
      -DH5_LDOUBLE_TO_UINT_ACCURATE_RUN__TRYRUN_OUTPUT=
      -DH5_LLONG_TO_LDOUBLE_CORRECT_RUN=0
      -DH5_LLONG_TO_LDOUBLE_CORRECT_RUN__TRYRUN_OUTPUT=
      -DH5_NO_ALIGNMENT_RESTRICTIONS_RUN=0
      -DH5_NO_ALIGNMENT_RESTRICTIONS_RUN__TRYRUN_OUTPUT=
      -DH5_ULLONG_TO_LDOUBLE_PRECISION_RUN=0
      -DH5_ULONG_TO_FLOAT_ACCURATE_RUN=0
      -DH5_ULONG_TO_FLOAT_ACCURATE_RUN__TRYRUN_OUTPUT=
      -DHDF5_PRINTF_LL_TEST_RUN=0
      -DHDF5_PRINTF_LL_TEST_RUN__TRYRUN_OUTPUT=
      )
  else()
    add_custom_target(VTKCompileTools)  # Empty target
  endif()

  if (CMAKE_COMPILER_IS_GNUCXX OR
      CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    # The following flag is necessary to statically link VTK within
    # the shared library containing the plugin
    set(Flags "-fPIC")
  else()
    set(Flags "")
  endif()

  externalproject_add(VTK
    SOURCE_DIR "${VTK_SOURCES_DIR}"

    CMAKE_ARGS
    ${VTKCMakeFlags}
    -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
    -DBUILD_SHARED_LIBS=OFF
    -DVTK_Group_Rendering=OFF

    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    "-DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS} ${Flags}"
    -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
    "-DCMAKE_C_FLAGS=${CMAKE_C_FLAGS} ${Flags}"
    -DCMAKE_OSX_DEPLOYMENT_TARGET=${CMAKE_OSX_DEPLOYMENT_TARGET}
    -DCMAKE_OSX_ARCHITECTURES=${CMAKE_OSX_ARCHITECTURES}

    BUILD_COMMAND
    ${CMAKE_COMMAND} --build <BINARY_DIR> --config <CONFIG> --parallel ${NumberOfPhysicalCores} -t
    vtkCommonCore
    vtkCommonDataModel
    vtkCommonMath
    vtkCommonMisc
    vtkCommonSystem
    vtkCommonTransforms
    vtkFiltersCore
    vtkFiltersPoints
    vtkImagingCore

    EXCLUDE_FROM_ALL TRUE

    INSTALL_COMMAND ""  # Skip the install step

    DEPENDS VTKCompileTools
    )

  if(MSVC)
    set(Suffix ".lib")
    set(Prefix "")
  else()
    set(Suffix ".a")
    list(GET CMAKE_FIND_LIBRARY_PREFIXES 0 Prefix)
  endif()

  foreach(module IN ITEMS
      # WARNING: The order of the modules below *is* important!
      ImagingCore
      FiltersCore
      CommonExecutionModel
      CommonDataModel
      CommonTransforms
      CommonMath
      CommonMisc
      CommonSystem
      CommonCore
      sys
      )
    list(APPEND VTK_LIBRARIES
      ${Prefix}vtk${module}-${VTK_MAJOR}.${VTK_MINOR}${Suffix}
      )
  endforeach()

  ExternalProject_Get_Property(VTK binary_dir)
  ExternalProject_Get_Property(VTK source_dir)

  link_directories(${binary_dir}/lib)

  foreach(dir IN ITEMS
      Common/Core
      Common/DataModel
      Common/ExecutionModel
      Common/Misc
      Filters/Core
      IO/Image
      Imaging/Core
      Utilities/KWIML
      )
    include_directories(
      ${source_dir}/${dir}
      ${binary_dir}/${dir}
      )
  endforeach()

endif()
