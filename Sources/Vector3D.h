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


class Vector3D
{
private:
  double x_;
  double y_;
  double z_;

public:
  Vector3D();

  Vector3D(double x,
           double y,
           double z);

  Vector3D(const Vector3D& from,
           const Vector3D& to);

  double GetX() const
  {
    return x_;
  }

  double GetY() const
  {
    return y_;
  }

  double GetZ() const
  {
    return z_;
  }

  double ComputeSquaredNorm() const;

  double ComputeNorm() const;

  void Normalize();

  static Vector3D CrossProduct(const Vector3D& u,
                               const Vector3D& v);

  static double DotProduct(const Vector3D& a,
                           const Vector3D& b);

  static bool AreParallel(const Vector3D& a,
                          const Vector3D& b);
};
