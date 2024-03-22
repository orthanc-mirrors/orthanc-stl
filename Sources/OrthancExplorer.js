/**
 * SPDX-FileCopyrightText: 2023-2024 Sebastien Jodogne, UCLouvain, Belgium
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/**
 * STL plugin for Orthanc
 * Copyright (C) 2023-2024 Sebastien Jodogne, UCLouvain, Belgium
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


const STL_PLUGIN_SOP_CLASS_UID_STL = '1.2.840.10008.5.1.4.1.1.104.3';
const STL_PLUGIN_SOP_CLASS_UID_RT_STRUCT = '1.2.840.10008.5.1.4.1.1.481.3';

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


function AddGenerateFromRtStructButton(instanceId, id, parent) {
  if (${HAS_CREATE_DICOM_STL}) {

    var b = $('<a>')
        .attr('id', id)
        .attr('data-role', 'button')
        .attr('href', '#')
        .attr('data-icon', 'search')
        .attr('data-theme', 'e')
        .text('Generate 3D model')
        .button();

    b.insertAfter($('#' + parent));
    b.click(function() {

      $.ajax({
        url: '../stl/rt-struct/' + instanceId,
        dataType: 'json',
        success: function(s) {

          var options = $('<ul>')
              .attr('data-divider-theme', 'd')
              .attr('data-role', 'listview');

          var select = $('<select>')
              .attr('id', id + '-structure')
              .attr('data-theme', 'a');

          for (i = 0; i < s.length; i++) {
            select.append($('<option>').attr('value', s[i]).text(s[i]));
          }

          options.append($('<li>').text('Choose the structure:'));
          options.append($('<li>').append(select));
          options.append($('<li>').text('Resolution:'));
          options.append($('<li>').append($('<select>')
                                          .attr('id', id + '-resolution')
                                          .attr('data-theme', 'a')
                                          .append($('<option>').attr('value', '256').text('256'))
                                          .append($('<option>').attr('value', '128').text('128'))
                                          .append($('<option>').attr('value', '512').text('512'))));
          options.append($('<li>')
                         .append($('<input>')
                                 .attr('id', id + '-smooth')
                                 .attr('type', 'checkbox')
                                 .attr('data-theme', 'a')
                                 .attr('checked', ''))
                         .append($('<label>')
                                 .attr('for', id + '-smooth')
                                 .text('Smooth volume')));

          options.append($('<li>').append(
            $('<a>')
              .attr('href', '#')
              .attr('rel', 'close').attr('data-theme', 'b')
              .text('Generate')
              .click(function(e) {
                e.preventDefault();

                var structure = $('#' + id + '-structure').val();
                var resolution = $('#' + id + '-resolution').val();
                var smooth = $('#' + id + '-smooth').is(':checked');

                $.ajax({
                  url: '../stl/encode-rtstruct',
                  type: 'POST',
                  data: JSON.stringify({
                    'Instance' : instanceId,
                    'RoiNames' : [ structure ],
                    'Smooth' : smooth,
                    'Resolution' : parseInt(resolution, 10)
                  }),
                  dataType: 'json',
                  success: function(s) {
                    $.mobile.changePage('#series?uuid=' + s.ParentSeries, {
                      allowSamePageTransition: true
                    });
                  },
                  error: function() {
                    alert('Error while generating the 3D model');
                  }
                });

              })));

          // Launch the dialog
          $('#dialog').simpledialog2({
            mode: 'blank',
            animate: false,
            headerText: 'Generate 3D model',
            headerClose: true,
            forceInput: false,
            width: '100%',
            blankContent: options
          });

        }
      });
    });
  }
}


$('#series').live('pagebeforeshow', function() {
  var seriesId = $.mobile.pageData.uuid;

  $('#stl-viewer-series').remove();
  $('#stl-generate-rtstruct-series').remove();

  GetResource('/series/' + seriesId, function(series) {
    if (series['Instances'].length == 1) {
      var instanceId = series['Instances'][0];

      $.ajax({
        url: '/instances/' + instanceId + '/metadata/SopClassUid',
        success: function(sopClassUid) {

          if (sopClassUid == STL_PLUGIN_SOP_CLASS_UID_STL) {
            // This is an "Encapsulated STL Storage" IOD, register the button
            AddOpenStlViewerButton(instanceId, 'stl-viewer-series', 'series-info');
          }
          else if (sopClassUid == STL_PLUGIN_SOP_CLASS_UID_RT_STRUCT) {
            AddGenerateFromRtStructButton(instanceId, 'stl-generate-rtstruct-series', 'series-info');
          }

        }
      });
    }
  });
});


$('#instance').live('pagebeforeshow', function() {
  var instanceId = $.mobile.pageData.uuid;

  $('#stl-viewer-instance').remove();
  $('#stl-generate-rtstruct-instance').remove();

  $.ajax({
    url: '/instances/' + instanceId + '/metadata/SopClassUid',
    success: function(sopClassUid) {

      if (sopClassUid == STL_PLUGIN_SOP_CLASS_UID_STL) {
        // This is an "Encapsulated STL Storage" IOD, register the button
        AddOpenStlViewerButton(instanceId, 'stl-viewer-instance', 'instance-info');
      }
      else if (sopClassUid == STL_PLUGIN_SOP_CLASS_UID_RT_STRUCT) {
        AddGenerateFromRtStructButton(instanceId, 'stl-generate-rtstruct-instance', 'instance-info');
      }

    }
  });
});


$('#study').live('pagebeforeshow', function() {
  if (${HAS_CREATE_DICOM_STL}) {
    var studyId = $.mobile.pageData.uuid;

    $('#stl-attach-nifti-study').remove();

    var b = $('<a>')
        .attr('id', 'stl-attach-nifti-study')
        .attr('data-role', 'button')
        .attr('href', '#')
        .attr('data-icon', 'search')
        .attr('data-theme', 'e')
        .text('Attach NIfTI 3D model')
        .button();

    b.insertAfter($('#study-info'));
    b.click(function() {

      var options = $('<ul>')
          .attr('data-divider-theme', 'd')
          .attr('data-role', 'listview');

      var upload = $('<input>')
          .attr('type', 'file')
          .attr('id', 'stl-attach-nifti-study-upload')
          .attr('data-theme', 'a');

      options.append($('<li>').text('Choose the NIfTI file:'));
      options.append($('<li>').append(upload));
      options.append($('<li>').text('Resolution:'));
      options.append($('<li>').append($('<select>')
                                      .attr('id', 'stl-attach-nifti-study-resolution')
                                      .attr('data-theme', 'a')
                                      .append($('<option>').attr('value', '256').text('256'))
                                      .append($('<option>').attr('value', '128').text('128'))
                                      .append($('<option>').attr('value', '512').text('512'))));
      options.append($('<li>')
                     .append($('<input>')
                             .attr('id', 'stl-attach-nifti-study-smooth')
                             .attr('type', 'checkbox')
                             .attr('data-theme', 'a')
                             .attr('checked', ''))
                     .append($('<label>')
                             .attr('for', 'stl-attach-nifti-study-smooth')
                             .text('Smooth volume')));

      options.append($('<li>').append(
        $('<a>')
          .attr('href', '#')
          .attr('rel', 'close').attr('data-theme', 'b')
          .text('Generate')
          .click(function(e) {
            e.preventDefault();

            var fileInput = document.getElementById('stl-attach-nifti-study-upload');
            var resolution = $('#stl-attach-nifti-study-resolution').val();
            var smooth = $('#stl-attach-nifti-study-smooth').is(':checked');

            if (fileInput.files.length == 0) {
              alert('No NIfTI file was selected');
              return;
            }

            reader = new FileReader();
            reader.onload = function() {

              // https://github.com/axios/axios/issues/513
              var nifti = reader.result;
              var niftiBase64 = btoa(new Uint8Array(nifti).reduce((data, byte) => data + String.fromCharCode(byte), ''));

              $.ajax({
                url: '../stl/encode-nifti',
                type: 'POST',
                data: JSON.stringify({
                  'Nifti' : 'data:application/octet-stream;base64,' + niftiBase64,
                  'ParentStudy' : studyId,
                  'Smooth' : smooth,
                  'Resolution' : parseInt(resolution, 10)
                }),
                dataType: 'json',
                success: function(s) {
                  $.mobile.changePage('#series?uuid=' + s.ParentSeries, {
                    allowSamePageTransition: true
                  });
                },
                error: function() {
                  alert('Error while generating the 3D model');
                }
              });

            };

            reader.readAsArrayBuffer(fileInput.files[0]);
          })));

      // Launch the dialog
      $('#dialog').simpledialog2({
        mode: 'blank',
        animate: false,
        headerText: 'Generate 3D model',
        headerClose: true,
        forceInput: false,
        width: '100%',
        blankContent: options
      });

    });
  }
});
