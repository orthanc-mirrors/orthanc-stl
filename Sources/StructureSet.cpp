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


#include "StructureSet.h"

#include "STLToolbox.h"

#include <OrthancException.h>

#include <dcmtk/dcmdata/dcdeftag.h>
#include <dcmtk/dcmdata/dcfilefo.h>


StructureSet::StructureSet(Orthanc::ParsedDicomFile& dicom) :
  hasFrameOfReferenceUid_(false)
{
  DcmDataset& dataset = *dicom.GetDcmtkObject().getDataset();
  patientId_ = STLToolbox::GetStringValue(dataset, DCM_PatientID);
  studyInstanceUid_ = STLToolbox::GetStringValue(dataset, DCM_StudyInstanceUID);
  seriesInstanceUid_ = STLToolbox::GetStringValue(dataset, DCM_SeriesInstanceUID);
  sopInstanceUid_ = STLToolbox::GetStringValue(dataset, DCM_SOPInstanceUID);

  DcmSequenceOfItems* frame = NULL;
  if (!dataset.findAndGetSequence(DCM_ReferencedFrameOfReferenceSequence, frame).good() ||
      frame == NULL)
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_BadFileFormat);
  }

  if (frame->card() == 1)
  {
    const char* v = NULL;
    if (frame->getItem(0)->findAndGetString(DCM_FrameOfReferenceUID, v).good() &&
        v != NULL)
    {
      hasFrameOfReferenceUid_ = true;
      frameOfReferenceUid_.assign(v);
    }
  }

  DcmSequenceOfItems* rois = NULL;
  if (!dataset.findAndGetSequence(DCM_ROIContourSequence, rois).good() ||
      rois == NULL)
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_BadFileFormat);
  }

  std::vector<DcmSequenceOfItems*> contours(rois->card());
  size_t countPolygons = 0;

  for (unsigned long i = 0; i < rois->card(); i++)
  {
    DcmSequenceOfItems* contour = NULL;
    if (!rois->getItem(i)->findAndGetSequence(DCM_ContourSequence, contour).good() ||
        contour == NULL)
    {
      throw Orthanc::OrthancException(Orthanc::ErrorCode_BadFileFormat);
    }
    else
    {
      contours[i] = contour;
      countPolygons += contour->card();
    }
  }

  polygons_.resize(countPolygons);

  size_t pos = 0;
  for (unsigned long i = 0; i < contours.size(); i++)
  {
    for (unsigned long j = 0; j < contours[i]->card(); j++, pos++)
    {
      polygons_[pos] = new StructurePolygon(dicom, i, j);
    }
  }

  assert(pos == countPolygons);
}


StructureSet::~StructureSet()
{
  for (size_t i = 0; i < polygons_.size(); i++)
  {
    assert(polygons_[i] != NULL);
    delete polygons_[i];
  }
}


std::string StructureSet::HashStudy() const
{
  Orthanc::DicomInstanceHasher hasher(patientId_, studyInstanceUid_, seriesInstanceUid_, sopInstanceUid_);
  return hasher.HashStudy();
}


const StructurePolygon& StructureSet::GetPolygon(size_t i) const
{
  if (i >= polygons_.size())
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_ParameterOutOfRange);
  }
  else
  {
    assert(polygons_[i] != NULL);
    return *polygons_[i];
  }
}


const std::string& StructureSet::GetFrameOfReferenceUid() const
{
  if (hasFrameOfReferenceUid_)
  {
    return frameOfReferenceUid_;
  }
  else
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_BadSequenceOfCalls);
  }
}


void StructureSet::ListStructuresNames(std::set<std::string>& target,
                                       Orthanc::ParsedDicomFile& source)
{
  target.clear();

  DcmSequenceOfItems* sequence = NULL;
  if (!source.GetDcmtkObject().getDataset()->findAndGetSequence(DCM_StructureSetROISequence, sequence).good() ||
      sequence == NULL)
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_BadFileFormat);
  }

  for (unsigned long i = 0; i < sequence->card(); i++)
  {
    DcmItem* item = sequence->getItem(i);
    if (item == NULL)
    {
      throw Orthanc::OrthancException(Orthanc::ErrorCode_BadFileFormat);
    }
    else
    {
      target.insert(STLToolbox::GetStringValue(*item, DCM_ROIName));
    }
  }
}
