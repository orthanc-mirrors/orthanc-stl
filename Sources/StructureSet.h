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


#pragma once

#include "StructurePolygon.h"

class StructureSet : public boost::noncopyable
{
private:
  std::vector<StructurePolygon*>  polygons_;
  std::string                     patientId_;
  std::string                     studyInstanceUid_;
  std::string                     seriesInstanceUid_;
  std::string                     sopInstanceUid_;
  bool                            hasFrameOfReferenceUid_;
  std::string                     frameOfReferenceUid_;

public:
  explicit StructureSet(Orthanc::ParsedDicomFile& dicom);

  ~StructureSet();

  const std::string& GetPatientId() const
  {
    return patientId_;
  }

  const std::string& GetStudyInstanceUid() const
  {
    return studyInstanceUid_;
  }

  const std::string& GetSeriesInstanceUid() const
  {
    return seriesInstanceUid_;
  }

  const std::string& GetSopInstanceUid() const
  {
    return sopInstanceUid_;
  }

  std::string HashStudy() const;

  size_t GetPolygonsCount() const
  {
    return polygons_.size();
  }

  const StructurePolygon& GetPolygon(size_t i) const;

  bool HasFrameOfReferenceUid() const
  {
    return hasFrameOfReferenceUid_;
  }

  const std::string& GetFrameOfReferenceUid() const;

  // This static method is faster than constructing the full "StructureSet" object
  static void ListStructuresNames(std::set<std::string>& target,
                                  Orthanc::ParsedDicomFile& source);
};
