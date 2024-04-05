#!/bin/bash

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



# This command-line script uses the "npm" tool to populate the
# "JavaScriptLibraries/dist" folder. It uses Docker to this end, in
# order to be usable on our CIS.

set -ex


##
## Prepare a Docker container with npm
##

if [ -t 1 ]; then
    # TTY is available => use interactive mode
    DOCKER_FLAGS='-i'
fi

ROOT_DIR=`dirname $(readlink -f $0)`/..
IMAGE=orthanc-stl-node

if [ -e "${ROOT_DIR}/JavaScriptLibraries/dist/" ]; then
    echo "Target folder is already existing, aborting"
    exit -1
fi

mkdir -p ${ROOT_DIR}/JavaScriptLibraries/dist/

( cd ${ROOT_DIR}/Resources/CreateJavaScriptLibraries && \
      docker build --no-cache -t ${IMAGE} . )


##
## Building O3DV (Online 3D Viewer)
## https://github.com/kovacsv/Online3DViewer
##

O3DV=Online3DViewer-0.12.0

echo "Creating the distribution of O3DV from $O3DV"

if [ ! -f "${ROOT_DIR}/JavaScriptLibraries/${O3DV}.tar.gz" ]; then
    mkdir -p "${ROOT_DIR}/JavaScriptLibraries"
    ( cd ${ROOT_DIR}/JavaScriptLibraries && \
          wget https://orthanc.uclouvain.be/downloads/third-party-downloads/STL/${O3DV}.tar.gz )
fi

docker run -t ${DOCKER_FLAGS} --rm \
       --user $(id -u):$(id -g) \
       -v ${ROOT_DIR}/Resources/CreateJavaScriptLibraries/build-o3dv.sh:/source/build-o3dv.sh:ro \
       -v ${ROOT_DIR}/JavaScriptLibraries/${O3DV}.tar.gz:/source/${O3DV}.tar.gz:ro \
       -v ${ROOT_DIR}/JavaScriptLibraries/dist/:/target:rw \
       ${IMAGE} \
       bash /source/build-o3dv.sh ${O3DV}


##
## Building Three.js
##

THREE=three.js-r154-sources

echo "Creating the distribution of Three.js from $THREE"

if [ ! -f "${ROOT_DIR}/JavaScriptLibraries/${THREE}.tar.gz" ]; then
    mkdir -p "${ROOT_DIR}/JavaScriptLibraries"
    ( cd ${ROOT_DIR}/JavaScriptLibraries && \
          wget https://orthanc.uclouvain.be/downloads/third-party-downloads/${THREE}.tar.gz )
fi

docker run -t ${DOCKER_FLAGS} --rm \
       --user $(id -u):$(id -g) \
       -v ${ROOT_DIR}/Resources/CreateJavaScriptLibraries/build-three.sh:/source/build-three.sh:ro \
       -v ${ROOT_DIR}/JavaScriptLibraries/${THREE}.tar.gz:/source/${THREE}.tar.gz:ro \
       -v ${ROOT_DIR}/JavaScriptLibraries/dist/:/target:rw \
       ${IMAGE} \
       bash /source/build-three.sh ${THREE}
