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


#include "Toolbox.h"

#include <algorithm>
#include <cmath>


namespace Toolbox
{
  bool IsNear(double a,
              double b)
  {
    return std::abs(a - b) < 10.0 * std::numeric_limits<double>::epsilon();
  }


  bool MyParseDouble(double& value,
                     const std::string& s)
  {
#if 1
    char* end = NULL;
    value = strtod(s.c_str(), &end);
    return (end == s.c_str() + s.size());
#else
    return Orthanc::SerializationToolbox::ParseDouble(value, s);
#endif
  }


  namespace
  {
    struct IsNearPredicate
    {
      bool operator() (const double& a,
                       const double& b)
      {
        return Toolbox::IsNear(a, b);
      }
    };
  }


  void RemoveDuplicateValues(std::vector<double>& v)
  {
    IsNearPredicate predicate;
    std::vector<double>::iterator last = std::unique(v.begin(), v.end(), predicate);
    v.erase(last, v.end());
  }
}
