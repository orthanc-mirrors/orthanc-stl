# SPDX-FileCopyrightText: 2023-2024 Sebastien Jodogne, UCLouvain, Belgium
# SPDX-License-Identifier: GPL-3.0-or-later


# STL plugin for Orthanc
# Copyright (C) 2023-2024 Sebastien Jodogne, UCLouvain, Belgium
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


# Native static compilation:
#   $ cmake .. -DCMAKE_BUILD_TYPE=Debug -DSTATIC_BUILD=ON && make -j4

# LSB compilation:
#   $ LSB_CC=gcc-4.8 LSB_CXX=g++-4.8 cmake .. -DCMAKE_BUILD_TYPE=Release -DSTATIC_BUILD=ON -DUSE_LEGACY_JSONCPP=ON -DUSE_LEGACY_BOOST=ON -DCMAKE_TOOLCHAIN_FILE=${HOME}/Subversion/orthanc/OrthancFramework/Resources/Toolchains/LinuxStandardBaseToolchain.cmake && make -j4

# MinGW compilation:
#   $ cmake .. -DCMAKE_BUILD_TYPE=Release -DSTATIC_BUILD=ON -DUSE_LEGACY_JSONCPP=ON -DUSE_LEGACY_BOOST=ON -DCMAKE_TOOLCHAIN_FILE=${HOME}/Subversion/orthanc/OrthancFramework/Resources/Toolchains/MinGW-W64-Toolchain32.cmake && make -j4


# We request version 3.12 of CMake, as it is the first version to
# support "--parallel" in "cmake --build":
# https://cmake.org/cmake/help/latest/manual/cmake.1.html#build-a-project
cmake_minimum_required(VERSION 3.12)
project(OrthancSTL)

set(ORTHANC_STL_VERSION "mainline")

if (ORTHANC_STL_VERSION STREQUAL "mainline")
  set(ORTHANC_FRAMEWORK_DEFAULT_VERSION "mainline")
  set(ORTHANC_FRAMEWORK_DEFAULT_SOURCE "hg")
else()
  set(ORTHANC_FRAMEWORK_DEFAULT_VERSION "1.12.0")
  set(ORTHANC_FRAMEWORK_DEFAULT_SOURCE "web")
endif()


#####################################################################
## Parameters of the build
#####################################################################

# Generic parameters
SET(STATIC_BUILD OFF CACHE BOOL "Static build of the third-party libraries (necessary for Windows)")
SET(ALLOW_DOWNLOADS OFF CACHE BOOL "Allow CMake to download packages")
set(ORTHANC_FRAMEWORK_SOURCE "${ORTHANC_FRAMEWORK_DEFAULT_SOURCE}" CACHE STRING "Source of the Orthanc framework (can be \"system\", \"hg\", \"archive\", \"web\" or \"path\")")
set(ORTHANC_FRAMEWORK_VERSION "${ORTHANC_FRAMEWORK_DEFAULT_VERSION}" CACHE STRING "Version of the Orthanc framework")
set(ORTHANC_FRAMEWORK_ARCHIVE "" CACHE STRING "Path to the Orthanc archive, if ORTHANC_FRAMEWORK_SOURCE is \"archive\"")
set(ORTHANC_FRAMEWORK_ROOT "" CACHE STRING "Path to the Orthanc source directory, if ORTHANC_FRAMEWORK_SOURCE is \"path\"")

# Advanced parameters to fine-tune linking against system libraries
SET(USE_SYSTEM_ORTHANC_SDK ON CACHE BOOL "Use the system version of the Orthanc plugin SDK")

# Parameters to fine-tune linking against system libraries
set(USE_SYSTEM_NIFTILIB ON CACHE BOOL "Use the system version of niftilib")
set(USE_SYSTEM_VTK ON CACHE BOOL "Use the system version of VTK")

# Option "VTK_BUILD_TYPE" is necessary for multi-configuration
# compilers (such as XCode or MSVC), for "externalproject_add()" to
# choose the correct build type (which is not known when running
# "cmake" to configure the project)
set(VTK_BUILD_TYPE "Release" CACHE STRING "Build type to be used for static builds of VTK (should be \"Debug\" or \"Release\")")


#####################################################################
## Download and setup the Orthanc framework
#####################################################################

include(${CMAKE_SOURCE_DIR}/Resources/Orthanc/CMake/DownloadOrthancFramework.cmake)

if (ORTHANC_FRAMEWORK_SOURCE STREQUAL "system")
  if (ORTHANC_FRAMEWORK_USE_SHARED)
    include(FindBoost)
    find_package(Boost COMPONENTS system thread)
    
    if (NOT Boost_FOUND)
      message(FATAL_ERROR "Unable to locate Boost on this system")
    endif()
    
    link_libraries(${Boost_LIBRARIES} jsoncpp)
  endif()
  
  link_libraries(${ORTHANC_FRAMEWORK_LIBRARIES})
  
else()
  include(${ORTHANC_FRAMEWORK_ROOT}/../Resources/CMake/OrthancFrameworkParameters.cmake)

  set(ENABLE_LOCALE ON)
  set(ENABLE_ZLIB ON)
  set(ENABLE_DCMTK ON)

  set(ENABLE_MODULE_JOBS OFF CACHE INTERNAL "")

  include(${ORTHANC_FRAMEWORK_ROOT}/../Resources/CMake/OrthancFrameworkConfiguration.cmake)
  include_directories(${ORTHANC_FRAMEWORK_ROOT})
endif()

include(${CMAKE_SOURCE_DIR}/Resources/Orthanc/Plugins/OrthancPluginsExports.cmake)


#####################################################################
## Find the Orthanc SDK
#####################################################################

if (STATIC_BUILD OR NOT USE_SYSTEM_ORTHANC_SDK)
  include_directories(${CMAKE_SOURCE_DIR}/Resources/Orthanc/Sdk-1.0.0)
else ()
  CHECK_INCLUDE_FILE_CXX(orthanc/OrthancCPlugin.h HAVE_ORTHANC_H)
  if (NOT HAVE_ORTHANC_H)
    message(FATAL_ERROR "Please install the headers of the Orthanc plugins SDK")
  endif()
endif()


#####################################################################
## Platform-specific configuration
#####################################################################

include(${CMAKE_SOURCE_DIR}/Resources/CMake/LibVtkConfiguration.cmake)
include(${CMAKE_SOURCE_DIR}/Resources/CMake/LibNiftiConfiguration.cmake)

add_definitions(
  -DHAS_ORTHANC_EXCEPTION=1
  -DORTHANC_ENABLE_LOGGING_PLUGIN=1
  -DORTHANC_FRAMEWORK_BUILDING_PLUGIN=1
  -DORTHANC_STL_VERSION="${ORTHANC_STL_VERSION}"
  -DMETADATA_VERSION=${METADATA_VERSION}
  )

if (${CMAKE_SYSTEM_NAME} STREQUAL "Linux" OR
    ${CMAKE_SYSTEM_NAME} STREQUAL "kFreeBSD" OR
    ${CMAKE_SYSTEM_NAME} STREQUAL "FreeBSD")
  link_libraries(rt)

elseif (${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
  execute_process(
    COMMAND 
    ${PYTHON_EXECUTABLE} ${ORTHANC_FRAMEWORK_ROOT}/../Resources/WindowsResources.py
    ${ORTHANC_STL_VERSION} "OrthancSTL" OrthancSTL.dll "STL plugin for Orthanc"
    ERROR_VARIABLE Failure
    OUTPUT_FILE ${AUTOGENERATED_DIR}/Version.rc
    )

  if (Failure)
    message(FATAL_ERROR "Error while computing the version information: ${Failure}")
  endif()

  list(APPEND AUTOGENERATED_SOURCES  ${AUTOGENERATED_DIR}/Version.rc)
endif()

if (APPLE)
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -framework CoreFoundation")
  SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -framework CoreFoundation")
endif()


#####################################################################
## Create the autogenerated files
#####################################################################

EmbedResources(
  THREE_HTML         ${CMAKE_SOURCE_DIR}/WebApplications/three.html
  THREE_JS           ${CMAKE_SOURCE_DIR}/WebApplications/three.js
  O3DV_HTML          ${CMAKE_SOURCE_DIR}/WebApplications/o3dv.html
  O3DV_JS            ${CMAKE_SOURCE_DIR}/WebApplications/o3dv.js
  ORTHANC_EXPLORER   ${CMAKE_SOURCE_DIR}/Sources/OrthancExplorer.js
  )

add_custom_command(
  OUTPUT
  ${AUTOGENERATED_DIR}/StaticAssets.cpp
  COMMAND
  ${PYTHON_EXECUTABLE}
  ${CMAKE_SOURCE_DIR}/Resources/EmbedStaticAssets.py
  ${CMAKE_SOURCE_DIR}/JavaScriptLibraries/dist
  ${AUTOGENERATED_DIR}/StaticAssets.cpp
  DEPENDS
  ${CMAKE_SOURCE_DIR}/JavaScriptLibraries/dist
  ${CMAKE_SOURCE_DIR}/Resources/EmbedStaticAssets.py
  )

list(APPEND AUTOGENERATED_SOURCES 
  ${AUTOGENERATED_DIR}/StaticAssets.cpp
  )

add_custom_target(
  AutogeneratedTarget
  DEPENDS 
  ${AUTOGENERATED_SOURCES}
  )


#####################################################################
## Create the plugin
#####################################################################

add_library(OrthancSTL SHARED
  Sources/Extent2D.cpp
  Sources/Plugin.cpp
  Sources/STLToolbox.cpp
  Sources/StructurePolygon.cpp
  Sources/StructureSet.cpp
  Sources/StructureSetGeometry.cpp
  Sources/VTKToolbox.cpp
  Sources/Vector3D.cpp

  ${AUTOGENERATED_SOURCES}
  ${CMAKE_SOURCE_DIR}/Resources/Orthanc/Plugins/OrthancPluginCppWrapper.cpp
  ${ORTHANC_CORE_SOURCES_DEPENDENCIES}
  ${ORTHANC_CORE_SOURCES_INTERNAL}
  ${ORTHANC_DICOM_SOURCES_DEPENDENCIES}
  ${ORTHANC_DICOM_SOURCES_INTERNAL}
  ${ORTHANC_STL_SOURCES}
  ${NIFTILIB_SOURCES}
  )

if (COMMAND DefineSourceBasenameForTarget)
  DefineSourceBasenameForTarget(OrthancSTL)
endif()

add_dependencies(OrthancSTL AutogeneratedTarget VTK)

message("Setting the version of the library to ${ORTHANC_STL_VERSION}")

set_target_properties(OrthancSTL PROPERTIES 
  VERSION ${ORTHANC_STL_VERSION} 
  SOVERSION ${ORTHANC_STL_VERSION})

target_link_libraries(OrthancSTL ${DCMTK_LIBRARIES} ${VTK_LIBRARIES})

install(
  TARGETS OrthancSTL
  RUNTIME DESTINATION lib    # Destination for Windows
  LIBRARY DESTINATION share/orthanc/plugins    # Destination for Linux
  )
