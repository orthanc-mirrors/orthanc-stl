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

set -ex

if [ "$1" = "" ]; then
    echo "Please provide the source package of Three.js"
    exit -1
fi

cd /tmp/
tar xvf /source/$1.tar.gz

cd /tmp/$1
npm install --cache /tmp/npm-cache
npm run build --cache /tmp/npm-cache

cp -r /tmp/$1/build/three.module.min.js /target/

cp /tmp/$1/editor/js/libs/es-module-shims.js /target/
cp /tmp/$1/examples/jsm/controls/OrbitControls.js /target/
cp /tmp/$1/examples/jsm/loaders/STLLoader.js /target/
