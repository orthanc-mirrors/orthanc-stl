#!/bin/bash

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



# This command-line script uses the "npm" tool to populate the "dist"
# folder of Three.js. It uses Docker to this end, in order to be
# usable on our CIS.

set -ex

if [ "$1" = "" ]; then
    PACKAGE=three.js-r154-sources
else
    PACKAGE=$1
fi

if [ -t 1 ]; then
    # TTY is available => use interactive mode
    DOCKER_FLAGS='-i'
fi

ROOT_DIR=`dirname $(readlink -f $0)`/..
IMAGE=orthanc-stl-node

echo "Creating the distribution of Three.js from $PACKAGE"

if [ -e "${ROOT_DIR}/Three/dist/" ]; then
    echo "Target folder is already existing, aborting"
    exit -1
fi

if [ ! -f "${ROOT_DIR}/Three/${PACKAGE}.tar.gz" ]; then
    mkdir -p "${ROOT_DIR}/Three"
    ( cd ${ROOT_DIR}/Three && \
          wget https://orthanc.uclouvain.be/downloads/third-party-downloads/${PACKAGE}.tar.gz )
fi

mkdir -p ${ROOT_DIR}/Three/dist/

( cd ${ROOT_DIR}/Resources/CreateThreeDist && \
      docker build --no-cache -t ${IMAGE} . )

docker run -t ${DOCKER_FLAGS} --rm \
       --user $(id -u):$(id -g) \
       -v ${ROOT_DIR}/Resources/CreateThreeDist/build.sh:/source/build.sh:ro \
       -v ${ROOT_DIR}/Three/${PACKAGE}.tar.gz:/source/${PACKAGE}.tar.gz:ro \
       -v ${ROOT_DIR}/Three/dist/:/target:rw \
       ${IMAGE} \
       bash /source/build.sh ${PACKAGE}
