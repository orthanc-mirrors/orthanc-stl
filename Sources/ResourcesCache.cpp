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


#include "ResourcesCache.h"

#include "../Resources/Orthanc/Plugins/OrthancPluginCppWrapper.h"

#include <Compatibility.h>
#include <SystemToolbox.h>


// Forward declaration
void ReadStaticAsset(std::string& target,
                     const std::string& path);


class ResourcesCache::RestOutputHandler : public IHandler
{
private:
  OrthancPluginRestOutput* output_;
  std::string              mime_;

public:
  RestOutputHandler(OrthancPluginRestOutput* output,
                    const std::string& mime) :
    output_(output),
    mime_(mime)
  {
  }

  virtual void Apply(const std::string& resource) ORTHANC_OVERRIDE
  {
    OrthancPluginAnswerBuffer(OrthancPlugins::GetGlobalContext(), output_,
                              resource.c_str(), resource.size(), mime_.c_str());
  }
};


class ResourcesCache::StoreResourceIntoString : public IHandler
{
private:
  std::string& target_;

public:
  StoreResourceIntoString(std::string& target) :
    target_(target)
  {
  }

  virtual void Apply(const std::string& resource) ORTHANC_OVERRIDE
  {
    target_ = resource;
  }
};


ResourcesCache::~ResourcesCache()
{
  for (Content::iterator it = content_.begin(); it != content_.end(); ++it)
  {
    assert(it->second != NULL);
    delete it->second;
  }
}


void ResourcesCache::Apply(IHandler& handler,
                           const std::string& path)
{
  {
    // Check whether the cache already contains the resource
    boost::shared_lock<boost::shared_mutex> lock(mutex_);

    Content::const_iterator found = content_.find(path);

    if (found != content_.end())
    {
      assert(found->second != NULL);
      handler.Apply(*found->second);
      return;
    }
  }

  // This resource has not been cached yet

  std::unique_ptr<std::string> item(new std::string);
  ReadStaticAsset(*item, path);
  handler.Apply(*item);

  {
    // Store the resource into the cache
    boost::unique_lock<boost::shared_mutex> lock(mutex_);

    if (content_.find(path) == content_.end())
    {
      content_[path] = item.release();
    }
  }
}


void ResourcesCache::Answer(OrthancPluginRestOutput* output,
                            const std::string& path)
{
  const std::string mime = Orthanc::EnumerationToString(Orthanc::SystemToolbox::AutodetectMimeType(path));

  RestOutputHandler handler(output, mime);
  Apply(handler, path);
}


void ResourcesCache::ReadResource(std::string& target,
                                  const std::string& path)
{
  StoreResourceIntoString handler(target);
  Apply(handler, path);
}
