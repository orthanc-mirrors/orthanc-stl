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

#include <orthanc/OrthancCPlugin.h>

#include <boost/noncopyable.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <map>
#include <string>


/**
 * As the static assets are gzipped by the "EmbedStaticAssets.py"
 * script, we use a cache to maintain the uncompressed assets in order
 * to avoid multiple gzip decodings.
 **/
class ResourcesCache : public boost::noncopyable
{
public:
  class IHandler : public boost::noncopyable
  {
  public:
    virtual ~IHandler()
    {
    }

    virtual void Apply(const std::string& resource) = 0;
  };

private:
  typedef std::map<std::string, std::string*>  Content;

  boost::shared_mutex  mutex_;
  Content              content_;

  class RestOutputHandler;
  class StoreResourceIntoString;

public:
  ~ResourcesCache();

  void Apply(IHandler& handler,
             const std::string& path);

  void Answer(OrthancPluginRestOutput* output,
              const std::string& path);

  void ReadResource(std::string& target,
                    const std::string& path);
};
