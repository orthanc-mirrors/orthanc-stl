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


#include "StructureSetGeometry.h"

#include "STLToolbox.h"

#include <OrthancException.h>

#include <list>


bool StructureSetGeometry::LookupProjectionIndex(size_t& index,
                                                 double z) const
{
  if (slicesCount_ == 0)
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_InternalError);
  }

  if (z < minProjectionAlongNormal_ ||
      z > maxProjectionAlongNormal_)
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_ParameterOutOfRange);
  }

  assert(slicesSpacing_ > 0 &&
         minProjectionAlongNormal_ < maxProjectionAlongNormal_);

  double d = (z - minProjectionAlongNormal_) / slicesSpacing_;

  if (STLToolbox::IsNear(d, round(d)))
  {
    if (d < 0.0 ||
        d > static_cast<double>(slicesCount_) - 1.0)
    {
      throw Orthanc::OrthancException(Orthanc::ErrorCode_InternalError);
    }
    else
    {
      index = static_cast<size_t>(round(d));
      return true;
    }
  }
  else
  {
    return false;
  }
}


StructureSetGeometry::StructureSetGeometry(const StructureSet& structures,
                                           bool strict)
{
  bool isValid = false;

  std::vector<double> projections;
  projections.reserve(structures.GetPolygonsCount());

  for (size_t i = 0; i < structures.GetPolygonsCount(); i++)
  {
    Vector3D normal;
    if (structures.GetPolygon(i).IsCoplanar(normal))
    {
      // Initialize the normal of the whole volume, if need be
      if (!isValid)
      {
        isValid = true;
        slicesNormal_ = normal;
      }

      if (Vector3D::AreParallel(normal, slicesNormal_))
      {
        // This is a valid slice (it is parallel to the normal)
        const Vector3D& point = structures.GetPolygon(i).GetPoint(0);
        projections.push_back(Vector3D::DotProduct(point, slicesNormal_));
      }
      else
      {
        // RT-STRUCT with non-parallel slices
        throw Orthanc::OrthancException(Orthanc::ErrorCode_NotImplemented);
      }
    }
    else
    {
      // Ignore slices that are not coplanar
    }
  }

  if (projections.empty())
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_NotImplemented,
                                    "Structure set without a valid geometry");
  }

  // Only keep unique projections

  std::sort(projections.begin(), projections.end());
  STLToolbox::RemoveDuplicateValues(projections);
  assert(!projections.empty());

  if (projections.size() == 1)
  {
    // Volume with one single slice
    minProjectionAlongNormal_ = projections[0];
    maxProjectionAlongNormal_ = projections[0];
    slicesSpacing_ = 1;   // Arbitrary value
    slicesCount_ = 1;
    return;
  }


  // Compute the most probable spacing between the slices

  {
    std::vector<double> spacings;
    spacings.resize(projections.size() - 1);

    for (size_t i = 0; i < spacings.size(); i++)
    {
      spacings[i] = projections[i + 1] - projections[i];
      assert(spacings[i] > 0);
    }

    std::sort(spacings.begin(), spacings.end());
    STLToolbox::RemoveDuplicateValues(spacings);

    if (spacings.empty())
    {
      throw Orthanc::OrthancException(Orthanc::ErrorCode_InternalError);
    }

    slicesSpacing_ = spacings[spacings.size() / 10];  // Take the 90% percentile of smallest spacings
    assert(slicesSpacing_ > 0);
  }


  // Find the projection along the normal with the largest support

  bool first = true;
  size_t bestSupport = 0;       // Explicit initialization to avoid valgrind warnings on old compilers
  double bestProjection = 0.0;  // Explicit initialization to make old compilers happy

  std::list<size_t> candidates;
  for (size_t i = 0; i < projections.size(); i++)
  {
    candidates.push_back(i);
  }

  while (!candidates.empty())
  {
    std::list<size_t> next;

    size_t countSupport = 0;

    std::list<size_t>::const_iterator it = candidates.begin();
    size_t reference = *it;
    ++it;

    while (it != candidates.end())
    {
      double d = (projections[*it] - projections[reference]) / slicesSpacing_;
      if (STLToolbox::IsNear(d, round(d)))
      {
        countSupport ++;
      }
      else
      {
        next.push_back(*it);
      }

      ++it;
    }

    if (first ||
        countSupport > bestSupport)
    {
      first = false;
      bestSupport = countSupport;
      bestProjection = projections[reference];
    }

    if (strict &&
        !next.empty())
    {
      throw Orthanc::OrthancException(Orthanc::ErrorCode_BadFileFormat,
                                      "Structure set with multiple support, which is not allowed in Strict mode");
    }

    candidates.swap(next);
  }

  if (first)
  {
    // Should never happen
    throw Orthanc::OrthancException(Orthanc::ErrorCode_InternalError);
  }


  // Compute the range of the projections

  minProjectionAlongNormal_ = bestProjection;
  maxProjectionAlongNormal_ = bestProjection;

  for (size_t i = 0; i < projections.size(); i++)
  {
    double d = (projections[i] - bestProjection) / slicesSpacing_;
    if (STLToolbox::IsNear(d, round(d)))
    {
      minProjectionAlongNormal_ = std::min(minProjectionAlongNormal_, projections[i]);
      maxProjectionAlongNormal_ = std::max(maxProjectionAlongNormal_, projections[i]);
    }
  }

  double d = (maxProjectionAlongNormal_ - minProjectionAlongNormal_) / slicesSpacing_;
  if (STLToolbox::IsNear(d, round(d)))
  {
    slicesCount_ = static_cast<size_t>(round(d)) + 1;
  }
  else
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_InternalError);
  }


  // Sanity check

  size_t a, b;
  if (!LookupProjectionIndex(a, minProjectionAlongNormal_) ||
      !LookupProjectionIndex(b, maxProjectionAlongNormal_) ||
      a != 0 ||
      b + 1 != slicesCount_)
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_InternalError);
  }
}


bool StructureSetGeometry::ProjectAlongNormal(double& z,
                                              const StructurePolygon& polygon) const
{
  Vector3D normal;
  if (polygon.IsCoplanar(normal) &&
      Vector3D::AreParallel(normal, slicesNormal_))
  {
    z = Vector3D::DotProduct(polygon.GetPoint(0), slicesNormal_);
    return true;
  }
  else
  {
    return false;
  }
}


bool StructureSetGeometry::LookupSliceIndex(size_t& slice,
                                            const StructurePolygon& polygon) const
{
  double z;
  return (ProjectAlongNormal(z, polygon) &&
          LookupProjectionIndex(slice, z));
}
