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

#include <boost/noncopyable.hpp>


class Extent2D : public boost::noncopyable
{
private:
  bool    isEmpty_;
  double  x1_;
  double  y1_;
  double  x2_;
  double  y2_;

  void CheckNotEmpty() const;

public:
  Extent2D();

  bool IsEmpty() const
  {
    return isEmpty_;
  }

  double GetMinX() const;

  double GetMaxX() const;

  double GetMinY() const;

  double GetMaxY() const;

  double GetWidth() const;

  double GetHeight() const;

  void Add(double x,
           double y);
};
