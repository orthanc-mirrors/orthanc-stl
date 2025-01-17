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


#include "StructurePolygon.h"

#include "STLToolbox.h"

#include <OrthancException.h>
#include <SerializationToolbox.h>

#include <dcmtk/dcmdata/dcfilefo.h>
#include <dcmtk/dcmdata/dcdeftag.h>


StructurePolygon::StructurePolygon(Orthanc::ParsedDicomFile& dicom,
                                   unsigned long roiIndex,
                                   unsigned long contourIndex)
{
  DcmDataset& dataset = *dicom.GetDcmtkObject().getDataset();

  DcmItem* structure = NULL;
  DcmItem* roi = NULL;
  DcmItem* contour = NULL;
  DcmSequenceOfItems* referenced = NULL;

  if (!dataset.findAndGetSequenceItem(DCM_StructureSetROISequence, structure, roiIndex).good() ||
      structure == NULL ||
      !dataset.findAndGetSequenceItem(DCM_ROIContourSequence, roi, roiIndex).good() ||
      roi == NULL ||
      !roi->findAndGetSequenceItem(DCM_ContourSequence, contour, contourIndex).good() ||
      contour == NULL ||
      !contour->findAndGetSequence(DCM_ContourImageSequence, referenced).good() ||
      referenced == NULL ||
      referenced->card() != 1)
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_BadFileFormat);
  }

  roiName_ = STLToolbox::GetStringValue(*structure, DCM_ROIName);
  referencedSopInstanceUid_ = STLToolbox::GetStringValue(*referenced->getItem(0), DCM_ReferencedSOPInstanceUID);

  if (STLToolbox::GetStringValue(*contour, DCM_ContourGeometricType) != "CLOSED_PLANAR")
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_BadFileFormat);
  }

  {
    std::vector<std::string> tokens;
    Orthanc::Toolbox::TokenizeString(tokens, STLToolbox::GetStringValue(*roi, DCM_ROIDisplayColor), '\\');

    uint32_t r, g, b;
    if (tokens.size() != 3 ||
        !Orthanc::SerializationToolbox::ParseFirstUnsignedInteger32(r, tokens[0]) ||
        !Orthanc::SerializationToolbox::ParseFirstUnsignedInteger32(g, tokens[1]) ||
        !Orthanc::SerializationToolbox::ParseFirstUnsignedInteger32(b, tokens[2]) ||
        r > 255 ||
        g > 255 ||
        b > 255)
    {
      throw Orthanc::OrthancException(Orthanc::ErrorCode_BadFileFormat);
    }

    red_ = r;
    green_ = g;
    blue_ = b;
  }

  {
    std::vector<std::string> tokens;
    Orthanc::Toolbox::TokenizeString(tokens, STLToolbox::GetStringValue(*contour, DCM_ContourData), '\\');

    const std::string s = STLToolbox::GetStringValue(*contour, DCM_NumberOfContourPoints);

    uint32_t countPoints;
    if (!Orthanc::SerializationToolbox::ParseUnsignedInteger32(countPoints, s) ||
        tokens.size() != 3 * countPoints)
    {
      throw Orthanc::OrthancException(Orthanc::ErrorCode_BadFileFormat);
    }

    points_.reserve(countPoints);

    for (size_t i = 0; i < tokens.size(); i += 3)
    {
      double x, y, z;
      if (!STLToolbox::MyParseDouble(x, tokens[i]) ||
          !STLToolbox::MyParseDouble(y, tokens[i + 1]) ||
          !STLToolbox::MyParseDouble(z, tokens[i + 2]))
      {
        throw Orthanc::OrthancException(Orthanc::ErrorCode_BadFileFormat);
      }

      points_.push_back(Vector3D(x, y, z));
    }

    assert(points_.size() == countPoints);
  }
}


const Vector3D& StructurePolygon::GetPoint(size_t i) const
{
  if (i >= points_.size())
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_ParameterOutOfRange);
  }
  else
  {
    return points_[i];
  }
}


bool StructurePolygon::IsCoplanar(Vector3D& normal) const
{
  if (points_.size() < 3)
  {
    return false;
  }

  bool hasNormal = false;

  for (size_t i = 0; i < points_.size(); i++)
  {
    normal = Vector3D::CrossProduct(Vector3D(points_[1], points_[0]),
                                    Vector3D(points_[2], points_[0]));
    if (!STLToolbox::IsNear(normal.ComputeNorm(), 0))
    {
      normal.Normalize();
      hasNormal = true;
    }
  }

  if (!hasNormal)
  {
    return false;
  }

  double a = Vector3D::DotProduct(points_[0], normal);

  for (size_t i = 1; i < points_.size(); i++)
  {
    double b = Vector3D::DotProduct(points_[i], normal);
    if (!STLToolbox::IsNear(a, b))
    {
      return false;
    }
  }

  return true;
}


void StructurePolygon::Add(Extent2D& extent,
                           const Vector3D& axisX,
                           const Vector3D& axisY) const
{
  assert(STLToolbox::IsNear(1, axisX.ComputeNorm()));
  assert(STLToolbox::IsNear(1, axisY.ComputeNorm()));

  for (size_t i = 0; i < points_.size(); i++)
  {
    extent.Add(Vector3D::DotProduct(axisX, points_[i]),
               Vector3D::DotProduct(axisY, points_[i]));
  }
}
