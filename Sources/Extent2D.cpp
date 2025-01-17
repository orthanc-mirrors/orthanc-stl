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


#include "Extent2D.h"

#include <OrthancException.h>


void Extent2D::CheckNotEmpty() const
{
  if (isEmpty_)
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_BadSequenceOfCalls);
  }
}


Extent2D::Extent2D() :
  isEmpty_(true),
  x1_(0),
  y1_(0),
  x2_(0),
  y2_(0)
{
}


double Extent2D::GetMinX() const
{
  CheckNotEmpty();
  return x1_;
}


double Extent2D::GetMaxX() const
{
  CheckNotEmpty();
  return x2_;
}


double Extent2D::GetMinY() const
{
  CheckNotEmpty();
  return y1_;
}


double Extent2D::GetMaxY() const
{
  CheckNotEmpty();
  return y2_;
}


double Extent2D::GetWidth() const
{
  CheckNotEmpty();
  return x2_ - x1_;
}


double Extent2D::GetHeight() const
{
  CheckNotEmpty();
  return y2_ - y1_;
}


void Extent2D::Add(double x,
                   double y)
{
  if (isEmpty_)
  {
    x1_ = x2_ = x;
    y1_ = y2_ = y;
    isEmpty_ = false;
  }
  else
  {
    x1_ = std::min(x1_, x);
    x2_ = std::max(x2_, x);
    y1_ = std::min(y1_, y);
    y2_ = std::max(y2_, y);
  }
}
