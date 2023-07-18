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


const SOP_CLASS_UID_STL = '1.2.840.10008.5.1.4.1.1.104.3';

function AddOpenStlViewerButton(instanceId, id, parent) {
  var b = $('<a>')
      .attr('id', id)
      .attr('data-role', 'button')
      .attr('href', '#')
      .attr('data-icon', 'search')
      .attr('data-theme', 'e')
      .text('STL viewer')
      .button();

  b.insertAfter($('#' + parent));
  b.click(function() {
    if ($.mobile.pageData) {
      window.open('../stl/app/viewer.html?instance=' + instanceId);
    }
  });
}


$('#series').live('pagebeforeshow', function() {
  var seriesId = $.mobile.pageData.uuid;

  $('#stl-button-series').remove();

  // Test whether this is a whole-slide image by check the SOP Class
  // UID of one instance of the series
  GetResource('/series/' + seriesId, function(series) {
    if (series['Instances'].length == 1) {
      var instanceId = series['Instances'][0];

      $.ajax({
        url: '/instances/' + instanceId + '/metadata/SopClassUid',
        success: function(sopClassUid) {
          if (sopClassUid == SOP_CLASS_UID_STL) {
            // This is an "Encapsulated STL Storage" IOD, register the button
            AddOpenStlViewerButton(instanceId, 'stl-button-series', 'series-info');
          }
        }
      });
    }
  });
});


$('#instance').live('pagebeforeshow', function() {
  var instanceId = $.mobile.pageData.uuid;

  $('#stl-button-instance').remove();

  // Test whether this is a whole-slide image by check the SOP Class
  // UID of one instance of the series
  $.ajax({
    url: '/instances/' + instanceId + '/metadata/SopClassUid',
    success: function(sopClassUid) {
      if (sopClassUid == SOP_CLASS_UID_STL) {
        // This is an "Encapsulated STL Storage" IOD, register the button
        AddOpenStlViewerButton(instanceId, 'stl-button-instance', 'instance-info');
      }
    }
  });
});
