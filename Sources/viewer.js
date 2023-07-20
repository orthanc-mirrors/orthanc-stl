/**
 * SPDX-FileCopyrightText: 2023 Sebastien Jodogne, UCLouvain, Belgium
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/**
 * STL plugin for Orthanc
 * Copyright (C) 2023 Sebastien Jodogne, UCLouvain, Belgium
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 **/


import * as THREE from 'three';

import { OrbitControls } from './OrbitControls.js';
import { STLLoader } from './STLLoader.js';


// http://stackoverflow.com/a/21903119/881731
function GetUrlParameter(sParam) 
{
  var sPageURL = decodeURIComponent(window.location.search.substring(1));
  var sURLVariables = sPageURL.split('&');

  for (var i = 0; i < sURLVariables.length; i++) {
    var sParameterName = sURLVariables[i].split('=');

    if (sParameterName[0] === sParam) {
      return sParameterName[1] === undefined ? '' : sParameterName[1];
    }
  }

  return '';
};

var instanceId = GetUrlParameter('instance');


const scene = new THREE.Scene();

const renderer = new THREE.WebGLRenderer();
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

const material = new THREE.MeshPhongMaterial();

const light = new THREE.AmbientLight(0x555555);
scene.add(light);

const light2 = new THREE.PointLight(0xaaaaaa, 2);
light2.position.set(10, 10, 10);
scene.add(light2);
  
const loader = new STLLoader()
loader.load(
  //'../../instances/' + instanceId + '/content/0042-0011',
  '../../instances/' + instanceId + '/stl',
  function (geometry) {
    const frustumSize = 200;

    geometry.computeBoundingBox();
    geometry.translate(-(geometry.boundingBox.min.x + geometry.boundingBox.max.x) / 2.0,
                       -(geometry.boundingBox.min.y + geometry.boundingBox.max.y) / 2.0,
                       -(geometry.boundingBox.min.z + geometry.boundingBox.max.z) / 2.0);

    var maxSize = Math.max(geometry.boundingBox.max.x - geometry.boundingBox.min.x,
                           geometry.boundingBox.max.y - geometry.boundingBox.min.y,
                           geometry.boundingBox.max.z - geometry.boundingBox.min.z);

    geometry.scale((frustumSize / 2.0) / maxSize,
                   (frustumSize / 2.0) / maxSize,
                   (frustumSize / 2.0) / maxSize);

    const mesh = new THREE.Mesh(geometry, material);
    scene.add(mesh);

    const aspect = window.innerWidth / window.innerHeight;
    const camera = new THREE.OrthographicCamera(
      frustumSize * aspect / - 2, frustumSize * aspect / 2,
      frustumSize / 2, frustumSize / - 2, 1, frustumSize);

    camera.position.z = 100;

    const controls = new OrbitControls(camera, renderer.domElement);

    //controls.update() must be called after any manual changes to the camera's transform
    controls.update();

    function animate() {
      requestAnimationFrame(animate);

      // required if controls.enableDamping or controls.autoRotate are set to true
      controls.update();
      light2.position.copy(camera.position);

      renderer.render(scene, camera);
    }
    
    animate();

    function onWindowResize() {
      const aspect = window.innerWidth / window.innerHeight;
      camera.left = - frustumSize * aspect / 2;
      camera.right = frustumSize * aspect / 2;
      camera.top = frustumSize / 2;
      camera.bottom = - frustumSize / 2;
      camera.updateProjectionMatrix();
      renderer.setSize(window.innerWidth, window.innerHeight);
    }

    window.addEventListener('resize', onWindowResize );
  },
  function (xhr) {
  },
  function (error) {
    console.log(error);
  }
);
