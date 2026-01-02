/**
 * SPDX-FileCopyrightText: 2023-2026 Sebastien Jodogne, ICTEAM UCLouvain, Belgium
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/**
 * STL plugin for Orthanc
 * Copyright (C) 2023-2026 Sebastien Jodogne, ICTEAM UCLouvain, Belgium
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


#pragma once

#include "Extent2D.h"
#include "Vector3D.h"

#include <DicomParsing/ParsedDicomFile.h>


class StructurePolygon : public boost::noncopyable
{
private:
  std::string            roiName_;
  std::string            referencedSopInstanceUid_;
  uint8_t                red_;
  uint8_t                green_;
  uint8_t                blue_;
  std::vector<Vector3D>  points_;

public:
  StructurePolygon(Orthanc::ParsedDicomFile& dicom,
                   unsigned long roiIndex,
                   unsigned long contourIndex);

  const std::string& GetRoiName() const
  {
    return roiName_;
  }

  const std::string& GetReferencedSopInstanceUid() const
  {
    return referencedSopInstanceUid_;
  }

  size_t GetPointsCount() const
  {
    return points_.size();
  }

  const Vector3D& GetPoint(size_t i) const;

  bool IsCoplanar(Vector3D& normal) const;

  void Add(Extent2D& extent,
           const Vector3D& axisX,
           const Vector3D& axisY) const;
};
