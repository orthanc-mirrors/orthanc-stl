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


DownloadPackage(
  "bf61d139b592eb5b724f0f8a3f53bb04"
  "https://orthanc.uclouvain.be/downloads/third-party-downloads/STL/nexus-4.3.zip"
  "${CMAKE_BINARY_DIR}/nexus-4.3")

set(NEXUS_VIEWER_DIR  ${CMAKE_CURRENT_BINARY_DIR}/nexus)
file(MAKE_DIRECTORY ${NEXUS_VIEWER_DIR})

file(COPY
  ${CMAKE_BINARY_DIR}/nexus-4.3/html/js
  ${CMAKE_BINARY_DIR}/nexus-4.3/html/threejs.html
  DESTINATION
  ${NEXUS_VIEWER_DIR}
  )

execute_process(
  COMMAND ${PATCH_EXECUTABLE} -p0 -N -i
  ${CMAKE_CURRENT_LIST_DIR}/NexusViewer-4.3.patch
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  RESULT_VARIABLE Failure
  )

list(APPEND STATIC_ASSETS_PREFIXES "nexus")
list(APPEND STATIC_ASSETS_CONTENT  ${NEXUS_VIEWER_DIR})
