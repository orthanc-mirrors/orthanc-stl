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

#include "StructureSet.h"


class StructureSetGeometry : public boost::noncopyable
{
private:
  Vector3D  slicesNormal_;
  double    slicesSpacing_;
  double    minProjectionAlongNormal_;
  double    maxProjectionAlongNormal_;
  size_t    slicesCount_;

  bool LookupProjectionIndex(size_t& index,
                             double z) const;

public:
  StructureSetGeometry(const StructureSet& structures,
                       bool strict);
  
  const Vector3D& GetSlicesNormal() const
  {
    return slicesNormal_;
  }

  double GetSlicesSpacing() const
  {
    return slicesSpacing_;
  }

  double GetMinProjectionAlongNormal() const
  {
    return minProjectionAlongNormal_;
  }

  double GetMaxProjectionAlongNormal() const
  {
    return maxProjectionAlongNormal_;
  }

  bool ProjectAlongNormal(double& z,
                          const StructurePolygon& polygon) const;

  size_t GetSlicesCount() const
  {
    return slicesCount_;
  }

  bool LookupSliceIndex(size_t& slice,
                        const StructurePolygon& polygon) const;
};
