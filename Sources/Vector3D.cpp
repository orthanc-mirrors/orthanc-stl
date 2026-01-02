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


#include "Vector3D.h"

#include "STLToolbox.h"

#include <OrthancException.h>

#include <cmath>


Vector3D::Vector3D() :
  x_(0),
  y_(0),
  z_(0)
{
}


Vector3D::Vector3D(double x,
                   double y,
                   double z) :
  x_(x),
  y_(y),
  z_(z)
{
}


Vector3D::Vector3D(const Vector3D& from,
                   const Vector3D& to) :
  x_(to.x_ - from.x_),
  y_(to.y_ - from.y_),
  z_(to.z_ - from.z_)
{
}


double Vector3D::ComputeSquaredNorm() const
{
  return x_ * x_ + y_ * y_ + z_ * z_;
}


double Vector3D::ComputeNorm() const
{
  return sqrt(ComputeSquaredNorm());
}


void Vector3D::Normalize()
{
  double norm = ComputeNorm();
  if (!STLToolbox::IsNear(norm, 0))
  {
    x_ /= norm;
    y_ /= norm;
    z_ /= norm;
  }
}


Vector3D Vector3D::CrossProduct(const Vector3D& u,
                                const Vector3D& v)
{
  return Vector3D(u.GetY() * v.GetZ() - u.GetZ() * v.GetY(),
                  u.GetZ() * v.GetX() - u.GetX() * v.GetZ(),
                  u.GetX() * v.GetY() - u.GetY() * v.GetX());
}


double Vector3D::DotProduct(const Vector3D& a,
                            const Vector3D& b)
{
  return a.GetX() * b.GetX() + a.GetY() * b.GetY() + a.GetZ() * b.GetZ();
}


bool Vector3D::AreParallel(const Vector3D& a,
                           const Vector3D& b)
{
  if (STLToolbox::IsNear(a.ComputeSquaredNorm(), 1) &&
      STLToolbox::IsNear(b.ComputeSquaredNorm(), 1))
  {
    return STLToolbox::IsNear(std::abs(Vector3D::DotProduct(a, b)), 1);
  }
  else
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_ParameterOutOfRange,
                                    "Only applicable to normalized vectors");
  }
}
