/**
 * SPDX-FileCopyrightText: 2023-2025 Sebastien Jodogne, ICTEAM UCLouvain, Belgium
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/**
 * STL plugin for Orthanc
 * Copyright (C) 2023-2025 Sebastien Jodogne, ICTEAM UCLouvain, Belgium
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
var token = GetUrlParameter('token');

async function loadStl(viewer, instanceId) {
  
  var headers = {};
  if (token) {
      headers['Authorization'] = 'Bearer ' + token
  }

  const response = await fetch('../../instances/' + instanceId + '/stl', {
    headers: headers
  });

  const blob = await response.blob();
  const file = new File([blob], 'model.stl');

  viewer.LoadModelFromInputFiles([
    new OV.InputFile('model.stl', OV.FileSource.File, file),
  ]);
}

window.addEventListener ('load', () => {
  let parentDiv = document.getElementById ('viewer');

  let viewer = new OV.EmbeddedViewer (parentDiv, {
    /*backgroundColor : new OV.RGBAColor (255, 255, 255, 255),
    defaultColor : new OV.RGBColor (200, 200, 200),
    edgeSettings : new OV.EdgeSettings (false, new OV.RGBColor (0, 0, 0), 1)*/

    backgroundColor : new OV.RGBAColor (0, 0, 0, 255),
    defaultColor : new OV.RGBColor (200, 200, 200),
    edgeSettings : new OV.EdgeSettings (false, new OV.RGBColor (255, 255, 255), 1)
  });

  loadStl(viewer, instanceId);
});
