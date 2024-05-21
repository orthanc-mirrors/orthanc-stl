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


#if !defined(ORTHANC_ENABLE_NEXUS)
#  error Macro ORTHANC_ENABLE_NEXUS must be defined
#endif

#include "StructureSetGeometry.h"
#include "STLToolbox.h"
#include "VTKToolbox.h"

#include <EmbeddedResources.h>

#include "../Resources/Orthanc/Plugins/OrthancPluginCppWrapper.h"

#include <DicomParsing/FromDcmtkBridge.h>
#include <Images/ImageProcessing.h>
#include <Logging.h>
#include <SerializationToolbox.h>
#include <SystemToolbox.h>

#include <vtkNew.h>

#include <boost/algorithm/string.hpp>
#include <boost/thread/shared_mutex.hpp>


#if ORTHANC_ENABLE_NEXUS == 1
#  include <Cache/MemoryStringCache.h>
#endif

#define ORTHANC_PLUGIN_NAME  "stl"


#if ORTHANC_ENABLE_NEXUS == 1
static const char* const ORTHANC_STL_PRIVATE_CREATOR = "OrthancSTL";
static const char* const ORTHANC_STL_MANUFACTURER = "ORTHANC^STL";
static const uint16_t ORTHANC_STL_PRIVATE_GROUP = 0x4205u;
static const uint16_t ORTHANC_STL_CREATOR_ELEMENT = 0x0010u;
static const uint16_t ORTHANC_STL_NEXUS_ELEMENT = 0x1001u;
static const Orthanc::DicomTag DICOM_TAG_CREATOR_VERSION_UID(0x0008, 0x9123);

/**
 * Each version of the STL plugin must provide a different value for
 * the CreatorVersionUID (0008,9123) tag. A new UID can be generated
 * by typing:
 *
 * $ python -c 'import pydicom; print(pydicom.uid.generate_uid())'
 *
 **/
static const char* const ORTHANC_STL_CREATOR_VERSION_UID_MAINLINE = "1.2.826.0.1.3680043.8.498.90514926286349109728701975613711986292";

static const char* const GetCreatorVersionUid(const std::string& version)
{
  if (version == "mainline")
  {
    return ORTHANC_STL_CREATOR_VERSION_UID_MAINLINE;
  }
  else
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_InternalError);
  }
}

static void FillOrthancExplorerCreatorVersionUid(std::map<std::string, std::string>& dictionary)
{
  dictionary["ORTHANC_STL_CREATOR_VERSION_UID_MAINLINE"] = ORTHANC_STL_CREATOR_VERSION_UID_MAINLINE;
}

#endif


// Forward declaration
void ReadStaticAsset(std::string& target,
                     const std::string& path);


/**
 * As the static assets are gzipped by the "EmbedStaticAssets.py"
 * script, we use a cache to maintain the uncompressed assets in order
 * to avoid multiple gzip decodings.
 **/
class ResourcesCache : public boost::noncopyable
{
private:
  typedef std::map<std::string, std::string*>  Content;
  
  boost::shared_mutex  mutex_;
  Content              content_;

public:
  ~ResourcesCache()
  {
    for (Content::iterator it = content_.begin(); it != content_.end(); ++it)
    {
      assert(it->second != NULL);
      delete it->second;
    }
  }

  void Answer(OrthancPluginRestOutput* output,
              const std::string& path)
  {
    const std::string mime = Orthanc::EnumerationToString(Orthanc::SystemToolbox::AutodetectMimeType(path));

    {
      // Check whether the cache already contains the resource
      boost::shared_lock<boost::shared_mutex> lock(mutex_);

      Content::const_iterator found = content_.find(path);
    
      if (found != content_.end())
      {
        assert(found->second != NULL);
        OrthancPluginAnswerBuffer(OrthancPlugins::GetGlobalContext(), output, found->second->c_str(), found->second->size(), mime.c_str());
        return;
      }
    }

    // This resource has not been cached yet

    std::unique_ptr<std::string> item(new std::string);
    ReadStaticAsset(*item, path);
    OrthancPluginAnswerBuffer(OrthancPlugins::GetGlobalContext(), output, item->c_str(), item->size(), mime.c_str());

    {
      // Store the resource into the cache
      boost::unique_lock<boost::shared_mutex> lock(mutex_);

      if (content_.find(path) == content_.end())
      {
        content_[path] = item.release();
      }
    }
  }
};


static ResourcesCache cache_;

void ServeFile(OrthancPluginRestOutput* output,
               const char* url,
               const OrthancPluginHttpRequest* request)
{
  if (request->method != OrthancPluginHttpMethod_Get)
  {
    OrthancPluginSendMethodNotAllowed(OrthancPlugins::GetGlobalContext(), output, "GET");
    return;
  }

  const std::string file = request->groups[0];

  if (boost::starts_with(file, "libs/"))
  {
    cache_.Answer(output, file.substr(5));
  }
  else
  {
    Orthanc::EmbeddedResources::FileResourceId resourceId;
    Orthanc::MimeType mimeType;

    if (file == "three.html")
    {
      resourceId = Orthanc::EmbeddedResources::THREE_HTML;
      mimeType = Orthanc::MimeType_Html;
    }
    else if (file == "three.js")
    {
      resourceId = Orthanc::EmbeddedResources::THREE_JS;
      mimeType = Orthanc::MimeType_JavaScript;
    }
    else if (file == "o3dv.html")
    {
      resourceId = Orthanc::EmbeddedResources::O3DV_HTML;
      mimeType = Orthanc::MimeType_Html;
    }
    else if (file == "o3dv.js")
    {
      resourceId = Orthanc::EmbeddedResources::O3DV_JS;
      mimeType = Orthanc::MimeType_JavaScript;
    }
    else
    {
      OrthancPluginSendHttpStatusCode(OrthancPlugins::GetGlobalContext(), output, 404);
      return;
    }

    std::string s;
    Orthanc::EmbeddedResources::GetFileResource(s, resourceId);
    OrthancPluginAnswerBuffer(OrthancPlugins::GetGlobalContext(), output, s.c_str(), s.size(), Orthanc::EnumerationToString(mimeType));
  }
}




#include <dcmtk/dcmdata/dcfilefo.h>
#include <dcmtk/dcmdata/dcsequen.h>
#include <dcmtk/dcmdata/dcuid.h>


namespace
{
  class XorFiller : public Orthanc::ImageProcessing::IPolygonFiller
  {
  private:
    Orthanc::ImageAccessor& target_;

  public:
    explicit XorFiller(Orthanc::ImageAccessor& target) :
      target_(target)
    {
    }

    virtual void Fill(int y,
                      int x1,
                      int x2) ORTHANC_OVERRIDE
    {
      assert(x1 <= x2);

      if (y >= 0 &&
          y < static_cast<int>(target_.GetHeight()))
      {
        x1 = std::max(x1, 0);
        x2 = std::min(x2, static_cast<int>(target_.GetWidth()) - 1);

        uint8_t* p = reinterpret_cast<uint8_t*>(target_.GetRow(y)) + x1;

        for (int i = x1; i <= x2; i++, p++)
        {
          *p = (*p ^ 0xff);
        }
      }
    }
  };
}


static Orthanc::ParsedDicomFile* LoadInstance(const std::string& instanceId)
{
  std::string dicom;

  if (!OrthancPlugins::RestApiGetString(dicom, "/instances/" + instanceId + "/file", false))
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_UnknownResource);
  }
  else
  {
    return new Orthanc::ParsedDicomFile(dicom);
  }
}


static void GetReferencedVolumeAxes(Vector3D& axisX,
                                    Vector3D& axisY,
                                    const StructureSet& structureSet)
{
  // This is a rough guess for the X/Y axes
  axisX = Vector3D(1, 0, 0);
  axisY = Vector3D(0, 1, 0);

  // Look for one instance from the referenced volume
  const std::string& sopInstanceUid = structureSet.GetPolygon(0).GetReferencedSopInstanceUid();

  Json::Value response;
  if (OrthancPlugins::RestApiPost(response, "/tools/lookup", sopInstanceUid, false))
  {
    if (response.type() != Json::arrayValue)
    {
      throw Orthanc::OrthancException(Orthanc::ErrorCode_NetworkProtocol);
    }

    bool first = true;

    for (Json::Value::ArrayIndex i = 0; i < response.size(); i++)
    {
      if (response[i].type() != Json::objectValue)
      {
        throw Orthanc::OrthancException(Orthanc::ErrorCode_NetworkProtocol);
      }

      if (Orthanc::SerializationToolbox::ReadString(response[i], "Type") == "Instance")
      {
        if (first)
        {
          const std::string& instanceId = Orthanc::SerializationToolbox::ReadString(response[i], "ID");
          std::unique_ptr<Orthanc::ParsedDicomFile> reference(LoadInstance(instanceId));

          std::string imageOrientation;
          if (reference->GetTagValue(imageOrientation, Orthanc::DICOM_TAG_IMAGE_ORIENTATION_PATIENT))
          {
            std::vector<std::string> items;
            Orthanc::Toolbox::TokenizeString(items, imageOrientation, '\\');

            double x1, x2, x3, y1, y2, y3;

            if (items.size() == 6 &&
                STLToolbox::MyParseDouble(x1, items[0]) &&
                STLToolbox::MyParseDouble(x2, items[1]) &&
                STLToolbox::MyParseDouble(x3, items[2]) &&
                STLToolbox::MyParseDouble(y1, items[3]) &&
                STLToolbox::MyParseDouble(y2, items[4]) &&
                STLToolbox::MyParseDouble(y3, items[5]))
            {
              axisX = Vector3D(x1, x2, x3);
              axisY = Vector3D(y1, y2, y3);
            }
          }
        }
        else
        {
          throw Orthanc::OrthancException(Orthanc::ErrorCode_InternalError,
                                          "Multiple instances with the same SOP Instance UID");
        }
      }
    }
  }
}


static bool EncodeStructureSetMesh(std::string& stl,
                                   const StructureSet& structureSet,
                                   const StructureSetGeometry& geometry,
                                   const std::set<std::string>& roiNames,
                                   unsigned int resolution,
                                   bool smooth)
{
  if (resolution < 1 ||
      structureSet.GetPolygonsCount() < 1)
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_ParameterOutOfRange);
  }

  if (!STLToolbox::IsNear(1, geometry.GetSlicesNormal().ComputeNorm()))
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_InternalError);
  }

  Vector3D axisX, axisY;
  GetReferencedVolumeAxes(axisX, axisY, structureSet);

  Vector3D axisZ = Vector3D::CrossProduct(axisX, axisY);

  if (!STLToolbox::IsNear(1, axisX.ComputeNorm()) ||
      !STLToolbox::IsNear(1, axisY.ComputeNorm()) ||
      !Vector3D::AreParallel(axisZ, geometry.GetSlicesNormal()))
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_InternalError);
  }

  Extent2D extent;
  for (size_t i = 0; i < structureSet.GetPolygonsCount(); i++)
  {
    structureSet.GetPolygon(i).Add(extent, axisX, axisY);
  }

  const int depth = geometry.GetSlicesCount();

  vtkNew<vtkImageData> volume;
  volume->SetDimensions(resolution, resolution, depth);
  volume->AllocateScalars(VTK_UNSIGNED_CHAR, 1);

  assert(sizeof(unsigned char) == 1);
  memset(volume->GetScalarPointer(), 0, resolution * resolution * depth);

  const double pixelSpacingX = extent.GetWidth() / static_cast<double>(resolution);
  const double pixelSpacingY = extent.GetHeight() / static_cast<double>(resolution);
  const double pixelSpacingZ = geometry.GetSlicesSpacing();

  bool first = true;

  for (size_t i = 0; i < structureSet.GetPolygonsCount(); i++)
  {
    const StructurePolygon& polygon = structureSet.GetPolygon(i);
    if (roiNames.find(polygon.GetRoiName()) != roiNames.end())
    {
      // This polygon corresponds to a ROI of interest

      size_t z;
      if (geometry.LookupSliceIndex(z, polygon))
      {
        std::vector<Orthanc::ImageProcessing::ImagePoint> points;
        points.reserve(polygon.GetPointsCount());

        for (size_t j = 0; j < polygon.GetPointsCount(); j++)
        {
          const Vector3D& point = polygon.GetPoint(j);
          double x = (Vector3D::DotProduct(point, axisX) - extent.GetMinX()) / extent.GetWidth() * static_cast<double>(resolution);
          double y = (Vector3D::DotProduct(point, axisY) - extent.GetMinY()) / extent.GetHeight() * static_cast<double>(resolution);
          points.push_back(Orthanc::ImageProcessing::ImagePoint(static_cast<int32_t>(std::floor(x)),
                                                                static_cast<int32_t>(std::floor(y))));

          if (first)
          {
            first = false;
            volume->SetOrigin(point.GetX() - x * pixelSpacingX,
                              point.GetY() - y * pixelSpacingY,
                              point.GetZ() - z * pixelSpacingZ);
          }
        }

        Orthanc::ImageAccessor slice;
        slice.AssignWritable(Orthanc::PixelFormat_Grayscale8, resolution, resolution, resolution /* pitch */,
                             reinterpret_cast<uint8_t*>(volume->GetScalarPointer()) + z * resolution * resolution);

        XorFiller filler(slice);
        Orthanc::ImageProcessing::FillPolygon(filler, points);
      }
    }
  }

  volume->SetSpacing(pixelSpacingX, pixelSpacingY, pixelSpacingZ);

  return VTKToolbox::EncodeVolume(stl, volume.Get(), resolution, smooth);
}


void ListStructures(OrthancPluginRestOutput* output,
                    const char* url,
                    const OrthancPluginHttpRequest* request)
{
  if (request->method != OrthancPluginHttpMethod_Get)
  {
    OrthancPluginSendMethodNotAllowed(OrthancPlugins::GetGlobalContext(), output, "GET");
    return;
  }

  const std::string instanceId(request->groups[0]);

  std::unique_ptr<Orthanc::ParsedDicomFile> dicom(LoadInstance(instanceId));

  std::set<std::string> names;
  StructureSet::ListStructuresNames(names, *dicom);

  Json::Value answer = Json::arrayValue;

  for (std::set<std::string>::const_iterator it = names.begin(); it != names.end(); ++it)
  {
    answer.append(*it);
  }

  std::string s = answer.toStyledString();
  OrthancPluginAnswerBuffer(OrthancPlugins::GetGlobalContext(), output, s.c_str(), s.size(), Orthanc::MIME_JSON);
}


static void AddDefaultTagValue(Json::Value& target,
                               const Orthanc::DicomTag& tag,
                               const std::string& value)
{
  if (!target.isMember(tag.Format()))
  {
    target[tag.Format()] = value;
  }
}


static void AddDefaultTagValue(Json::Value& target,
                               const DcmTagKey& tag,
                               const std::string& value)
{
  AddDefaultTagValue(target, Orthanc::DicomTag(tag.getGroup(), tag.getElement()), value);
}


static void CallCreateDicom(Json::Value& answer,
                            const std::string& stl,
                            const Json::Value& body,
                            const std::string& parentStudy,
                            const std::string& defaultSeriesDescription,
                            const std::string& defaultFrameOfReferenceUid,
                            const std::string& defaultTitle)
{
  static const char* const KEY_TAGS = "Tags";

  Json::Value normalized = Json::objectValue;

  if (body.isMember(KEY_TAGS))
  {
    const Json::Value& tags = body[KEY_TAGS];

    if (tags.type() != Json::objectValue)
    {
      throw Orthanc::OrthancException(Orthanc::ErrorCode_BadRequest, "Tags must be provided as a JSON object");
    }

    std::vector<std::string> keys = tags.getMemberNames();
    for (size_t i = 0; i < keys.size(); i++)
    {
      const Orthanc::DicomTag tag = Orthanc::FromDcmtkBridge::ParseTag(keys[i]);
      normalized[tag.Format()] = tags[keys[i]];
    }
  }

  if (!normalized.isMember(Orthanc::DICOM_TAG_SERIES_DESCRIPTION.Format()))
  {
    normalized[Orthanc::DICOM_TAG_SERIES_DESCRIPTION.Format()] = defaultSeriesDescription;
  }

  AddDefaultTagValue(normalized, Orthanc::DICOM_TAG_SERIES_NUMBER, "1");
  AddDefaultTagValue(normalized, Orthanc::DICOM_TAG_FRAME_OF_REFERENCE_UID, defaultFrameOfReferenceUid);
  AddDefaultTagValue(normalized, Orthanc::DICOM_TAG_INSTANCE_NUMBER, "1");

  AddDefaultTagValue(normalized, DCM_BurnedInAnnotation, "NO");
  AddDefaultTagValue(normalized, DCM_DeviceSerialNumber, ORTHANC_STL_VERSION);
  AddDefaultTagValue(normalized, DCM_DocumentTitle, defaultTitle);
  AddDefaultTagValue(normalized, DCM_Manufacturer, "Orthanc STL plugin");
  AddDefaultTagValue(normalized, DCM_ManufacturerModelName, "Orthanc STL plugin");
  AddDefaultTagValue(normalized, DCM_PositionReferenceIndicator, "");
  AddDefaultTagValue(normalized, DCM_SoftwareVersions, ORTHANC_STL_VERSION);
  AddDefaultTagValue(normalized, DCM_ConceptNameCodeSequence, "");

  std::string date, time;
  Orthanc::SystemToolbox::GetNowDicom(date, time, true /* use UTC time (not local time) */);
  AddDefaultTagValue(normalized, DCM_AcquisitionDateTime, date + time);

  const Orthanc::DicomTag MEASUREMENT_UNITS_CODE_SEQUENCE(DCM_MeasurementUnitsCodeSequence.getGroup(),
                                                          DCM_MeasurementUnitsCodeSequence.getElement());

  if (!normalized.isMember(MEASUREMENT_UNITS_CODE_SEQUENCE.Format()))
  {
    Json::Value item;
    item["CodeValue"] = "mm";
    item["CodingSchemeDesignator"] = "UCUM";
    item["CodeMeaning"] = defaultTitle;

    normalized[MEASUREMENT_UNITS_CODE_SEQUENCE.Format()].append(item);
  }

  std::string content;
  Orthanc::Toolbox::EncodeDataUriScheme(content, Orthanc::MIME_STL, stl);

  Json::Value create;
  create["Content"] = content;
  create["Parent"] = parentStudy;
  create["Tags"] = normalized;

  if (!OrthancPlugins::RestApiPost(answer, "/tools/create-dicom", create.toStyledString(), false))
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_BadRequest, "Cannot create DICOM from STL");
  }
}


void EncodeStructureSet(OrthancPluginRestOutput* output,
                        const char* url,
                        const OrthancPluginHttpRequest* request)
{
  static const char* const KEY_INSTANCE = "Instance";
  static const char* const KEY_RESOLUTION = "Resolution";
  static const char* const KEY_ROI_NAMES = "RoiNames";
  static const char* const KEY_SMOOTH = "Smooth";
  static const char* const KEY_STRICT = "Strict";

  if (request->method != OrthancPluginHttpMethod_Post)
  {
    OrthancPluginSendMethodNotAllowed(OrthancPlugins::GetGlobalContext(), output, "POST");
    return;
  }

  Json::Value body;
  if (!Orthanc::Toolbox::ReadJson(body, request->body, request->bodySize))
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_BadRequest);
  }

  const std::string instanceId = Orthanc::SerializationToolbox::ReadString(body, KEY_INSTANCE);
  const bool smooth = (body.isMember(KEY_SMOOTH) ?
                       Orthanc::SerializationToolbox::ReadBoolean(body, KEY_SMOOTH) :
                       true /* smooth by default */);
  const unsigned int resolution = (body.isMember(KEY_RESOLUTION) ?
                                   Orthanc::SerializationToolbox::ReadUnsignedInteger(body, KEY_RESOLUTION) :
                                   256 /* default value */);
  const bool strict = (body.isMember(KEY_STRICT) ?
                       Orthanc::SerializationToolbox::ReadBoolean(body, KEY_STRICT) :
                       true /* strict by default */);

  std::set<std::string> roiNames;
  Orthanc::SerializationToolbox::ReadSetOfStrings(roiNames, body, KEY_ROI_NAMES);

  std::unique_ptr<Orthanc::ParsedDicomFile> dicom(LoadInstance(instanceId));

  StructureSet structureSet(*dicom);

  StructureSetGeometry geometry(structureSet, strict);

  std::string stl;
  if (!EncodeStructureSetMesh(stl, structureSet, geometry, roiNames, resolution, smooth))
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_BadFileFormat, "Cannot encode STL from RT-STRUCT");
  }
  else
  {
    std::string seriesDescription;

    if (dicom->GetTagValue(seriesDescription, Orthanc::DICOM_TAG_SERIES_DESCRIPTION))
    {
      seriesDescription += ": ";
    }
    else
    {
      seriesDescription.clear();
    }

    bool first = true;
    for (std::set<std::string>::const_iterator it = roiNames.begin(); it != roiNames.end(); ++it)
    {
      if (first)
      {
        first = false;
      }
      else
      {
        seriesDescription += ", ";
      }

      seriesDescription += *it;
    }

    std::string frameOfReferenceUid;
    if (structureSet.HasFrameOfReferenceUid())
    {
      frameOfReferenceUid = structureSet.GetFrameOfReferenceUid();
    }
    else
    {
      frameOfReferenceUid = Orthanc::FromDcmtkBridge::GenerateUniqueIdentifier(Orthanc::ResourceType_Instance);
    }

    Json::Value answer;
    CallCreateDicom(answer, stl, body, structureSet.HashStudy(), seriesDescription,
                    frameOfReferenceUid, "STL model generated from DICOM RT-STRUCT");

    std::string s = answer.toStyledString();
    OrthancPluginAnswerBuffer(OrthancPlugins::GetGlobalContext(), output, s.c_str(), s.size(), Orthanc::MIME_JSON);
  }
}


void ExtractStl(OrthancPluginRestOutput* output,
                const char* url,
                const OrthancPluginHttpRequest* request)
{
  if (request->method != OrthancPluginHttpMethod_Get)
  {
    OrthancPluginSendMethodNotAllowed(OrthancPlugins::GetGlobalContext(), output, "GET");
    return;
  }

  const std::string instanceId(request->groups[0]);

  std::unique_ptr<Orthanc::ParsedDicomFile> dicom(LoadInstance(instanceId));
  DcmDataset& dataset = *dicom->GetDcmtkObject().getDataset();

  std::string stl;
  if (STLToolbox::GetStringValue(dataset, DCM_MIMETypeOfEncapsulatedDocument) != Orthanc::MIME_STL ||
      STLToolbox::GetStringValue(dataset, DCM_SOPClassUID) != UID_EncapsulatedSTLStorage ||
      !dicom->GetTagValue(stl, Orthanc::DICOM_TAG_ENCAPSULATED_DOCUMENT))
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_BadRequest, "DICOM instance not encapsulating a STL model: " + instanceId);
  }
  else
  {
    OrthancPluginAnswerBuffer(OrthancPlugins::GetGlobalContext(), output,
                              stl.empty() ? NULL : stl.c_str(), stl.size(), Orthanc::MIME_STL);
  }
}



void EncodeNifti(OrthancPluginRestOutput* output,
                 const char* url,
                 const OrthancPluginHttpRequest* request)
{
  static const char* const KEY_NIFTI = "Nifti";
  static const char* const KEY_RESOLUTION = "Resolution";
  static const char* const KEY_PARENT_STUDY = "ParentStudy";
  static const char* const KEY_SMOOTH = "Smooth";

  if (request->method != OrthancPluginHttpMethod_Post)
  {
    OrthancPluginSendMethodNotAllowed(OrthancPlugins::GetGlobalContext(), output, "POST");
    return;
  }

  Json::Value body;
  if (!Orthanc::Toolbox::ReadJson(body, request->body, request->bodySize))
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_BadRequest);
  }

  std::string mime, nifti;
  if (!Orthanc::Toolbox::DecodeDataUriScheme(mime, nifti, Orthanc::SerializationToolbox::ReadString(body, KEY_NIFTI)))
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_BadRequest, "Missing the \"Nifti\" argument containing the NIfTI file");
  }

  const std::string parentStudy = Orthanc::SerializationToolbox::ReadString(body, KEY_PARENT_STUDY);
  const bool smooth = (body.isMember(KEY_SMOOTH) ?
                       Orthanc::SerializationToolbox::ReadBoolean(body, KEY_SMOOTH) :
                       true /* smooth by default */);
  const unsigned int resolution = (body.isMember(KEY_RESOLUTION) ?
                                   Orthanc::SerializationToolbox::ReadUnsignedInteger(body, KEY_RESOLUTION) :
                                   256 /* default value */);

  vtkNew<vtkImageData> volume;
  VTKToolbox::LoadNifti(volume.Get(), nifti);

  std::string stl;
  if (!VTKToolbox::EncodeVolume(stl, volume.Get(), resolution, smooth))
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_BadFileFormat, "Cannot encode STL from NIfTI");
  }
  else
  {
    const std::string title = "STL model generated from NIfTI";

    const std::string frameOfReferenceUid = Orthanc::FromDcmtkBridge::GenerateUniqueIdentifier(Orthanc::ResourceType_Instance);

    Json::Value answer;
    CallCreateDicom(answer, stl, body, parentStudy, title, frameOfReferenceUid, title);

    std::string s = answer.toStyledString();
    OrthancPluginAnswerBuffer(OrthancPlugins::GetGlobalContext(), output, s.c_str(), s.size(), Orthanc::MIME_JSON);
  }
}


#if ORTHANC_ENABLE_NEXUS == 1

void ServeNexusAssets(OrthancPluginRestOutput* output,
                      const char* url,
                      const OrthancPluginHttpRequest* request)
{
  if (request->method != OrthancPluginHttpMethod_Get)
  {
    OrthancPluginSendMethodNotAllowed(OrthancPlugins::GetGlobalContext(), output, "GET");
    return;
  }

  const std::string file = request->groups[0];

  Orthanc::EmbeddedResources::FileResourceId resourceId;
  Orthanc::MimeType mimeType;

  if (file == "threejs.html")
  {
    resourceId = Orthanc::EmbeddedResources::NEXUS_HTML;
    mimeType = Orthanc::MimeType_Html;
  }
  else if (file == "js/meco.js")
  {
    resourceId = Orthanc::EmbeddedResources::NEXUS_MECO_JS;
    mimeType = Orthanc::MimeType_JavaScript;
  }
  else if (file == "js/nexus.js")
  {
    resourceId = Orthanc::EmbeddedResources::NEXUS_JS;
    mimeType = Orthanc::MimeType_JavaScript;
  }
  else if (file == "js/nexus_three.js")
  {
    resourceId = Orthanc::EmbeddedResources::NEXUS_THREE_JS;
    mimeType = Orthanc::MimeType_JavaScript;
  }
  else if (file == "js/TrackballControls.js")
  {
    resourceId = Orthanc::EmbeddedResources::NEXUS_TRACKBALL_JS;
    mimeType = Orthanc::MimeType_JavaScript;
  }
  else
  {
    OrthancPluginSendHttpStatusCode(OrthancPlugins::GetGlobalContext(), output, 404);
    return;
  }

  std::string s;
  Orthanc::EmbeddedResources::GetFileResource(s, resourceId);
  OrthancPluginAnswerBuffer(OrthancPlugins::GetGlobalContext(), output, s.c_str(), s.size(), Orthanc::EnumerationToString(mimeType));
}


static Orthanc::MemoryStringCache nexusCache_;

void ExtractNexusModel(OrthancPluginRestOutput* output,
                       const char* url,
                       const OrthancPluginHttpRequest* request)
{
  OrthancPluginContext* context = OrthancPlugins::GetGlobalContext();

  if (request->method != OrthancPluginHttpMethod_Get)
  {
    OrthancPluginSendMethodNotAllowed(context, output, "GET");
    return;
  }

  const std::string instanceId = request->groups[0];

  std::string range;

  for (uint32_t i = 0; i < request->headersCount; i++)
  {
    if (std::string(request->headersKeys[i]) == "range")
    {
      range = request->headersValues[i];
    }
  }

  static const std::string BYTES = "bytes=";

  if (!boost::starts_with(range, BYTES))
  {
    OrthancPluginSendHttpStatusCode(context, output, 416);  // Range not satisfiable
    return;
  }

  std::vector<std::string> tokens;
  Orthanc::Toolbox::TokenizeString(tokens, range.substr(BYTES.length()), '-');

  uint64_t start, end;

  if (tokens.size() != 2 ||
      !Orthanc::SerializationToolbox::ParseUnsignedInteger64(start, tokens[0]) ||
      !Orthanc::SerializationToolbox::ParseUnsignedInteger64(end, tokens[1]) ||
      start < 0 ||
      start > end)
  {
    OrthancPluginSendHttpStatusCode(context, output, 416);  // Range not satisfiable
    return;
  }

  uint64_t modelSize;
  std::string part;

#if 0
  {
    // Use no cache
    std::string model;
    if (!OrthancPlugins::RestApiGetString(model, "/instances/" + instanceId + "/content/4205-1001", false))
    {
      OrthancPluginSendHttpStatusCode(context, output, 404);
      return;
    }

    modelSize = model.size();

    if (end >= modelSize)
    {
      OrthancPluginSendHttpStatusCode(context, output, 416);  // Range not satisfiable
      return;
    }

    part = model.substr(start, end - start + 1);
  }
#else
  {
    Orthanc::MemoryStringCache::Accessor accessor(nexusCache_);

    std::string model;
    if (!accessor.Fetch(model, instanceId))
    {
      if (OrthancPlugins::RestApiGetString(model, "/instances/" + instanceId + "/content/4205-1001", false))
      {
        accessor.Add(instanceId, model);
      }
      else
      {
        OrthancPluginSendHttpStatusCode(context, output, 404);
        return;
      }
    }

    modelSize = model.size();

    if (end >= modelSize)
    {
      OrthancPluginSendHttpStatusCode(context, output, 416);  // Range not satisfiable
      return;
    }

    part = model.substr(start, end - start + 1);
  }
#endif

  std::string s = ("bytes " + boost::lexical_cast<std::string>(start) + "-" +
                   boost::lexical_cast<std::string>(end) + "/" +
                   boost::lexical_cast<std::string>(modelSize));
  OrthancPluginSetHttpHeader(context, output, "Content-Range", s.c_str());

  s = boost::lexical_cast<std::string>(part.size());
  OrthancPluginSetHttpHeader(context, output, "Content-Length", s.c_str());
  OrthancPluginSetHttpHeader(context, output, "Content-Type", "application/octet-stream");

  OrthancPluginSendHttpStatus(context, output, 206 /* partial content */, part.c_str(), part.size());
}


void DicomizeNexusModel(OrthancPluginRestOutput* output,
                        const char* url,
                        const OrthancPluginHttpRequest* request)
{
  static const char* KEY_CONTENT = "Content";
  static const char* KEY_PARENT = "Parent";
  static const char* KEY_TAGS = "Tags";
  static const char* KEY_PRIVATE_CREATOR = "PrivateCreator";

  OrthancPluginContext* context = OrthancPlugins::GetGlobalContext();

  if (request->method != OrthancPluginHttpMethod_Post)
  {
    OrthancPluginSendMethodNotAllowed(context, output, "POST");
    return;
  }

  Json::Value body;
  if (!Orthanc::Toolbox::ReadJson(body, request->body, request->bodySize) ||
      body.type() != Json::objectValue ||
      !body.isMember(KEY_TAGS) ||
      body[KEY_TAGS].type() != Json::objectValue)
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_BadRequest);
  }

  if (!body.isMember(KEY_CONTENT) ||
      body[KEY_CONTENT].type() != Json::stringValue)
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_BadRequest,
                                    "POST body missing string field \"" + std::string(KEY_CONTENT) + "\"");
  }

  std::string decoded;
  Orthanc::Toolbox::DecodeBase64(decoded, body[KEY_CONTENT].asString());

  if (decoded.size() < 4 ||
      decoded[0] != 0x20 ||
      decoded[1] != 0x73 ||
      decoded[2] != 0x78 ||
      decoded[3] != 0x4e)
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_BadFileFormat,
                                    "This is not a valid Nexus file, its magic header is incorrect");
  }

  Json::Value creationBody = Json::objectValue;

  creationBody[KEY_TAGS] = body[KEY_TAGS];
  creationBody[KEY_TAGS][Orthanc::DICOM_TAG_MANUFACTURER.Format()] = ORTHANC_STL_MANUFACTURER;
  creationBody[KEY_TAGS][Orthanc::DICOM_TAG_SOP_CLASS_UID.Format()] = "1.2.840.10008.5.1.4.1.1.66";
  creationBody[KEY_TAGS][Orthanc::DICOM_TAG_MODALITY.Format()] = "OT";
  creationBody[KEY_TAGS][DICOM_TAG_CREATOR_VERSION_UID.Format()] = GetCreatorVersionUid(ORTHANC_STL_VERSION);
  creationBody[KEY_TAGS][Orthanc::DicomTag(ORTHANC_STL_PRIVATE_GROUP, ORTHANC_STL_CREATOR_ELEMENT).Format()] = ORTHANC_STL_PRIVATE_CREATOR;
  creationBody[KEY_TAGS][Orthanc::DicomTag(ORTHANC_STL_PRIVATE_GROUP, ORTHANC_STL_NEXUS_ELEMENT).Format()] =
    "data:application/octet-stream;base64," + body[KEY_CONTENT].asString();
  creationBody[KEY_PRIVATE_CREATOR] = ORTHANC_STL_PRIVATE_CREATOR;

  if (body.isMember(KEY_PARENT))
  {
    creationBody[KEY_PARENT] = body[KEY_PARENT];
  }

  std::string bodyString;
  Orthanc::Toolbox::WriteFastJson(bodyString, creationBody);

  std::string result;
  if (OrthancPlugins::RestApiPost(result, "/tools/create-dicom",
                                  bodyString.empty() ? NULL : bodyString.c_str(),
                                  bodyString.size(), false))
  {
    OrthancPluginAnswerBuffer(context, output, result.empty() ? NULL : result.c_str(),
                              result.size(), "application/json");
  }
  else
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_BadRequest);
  }
}

#endif


extern "C"
{
  ORTHANC_PLUGINS_API int32_t OrthancPluginInitialize(OrthancPluginContext* context)
  {
    OrthancPlugins::SetGlobalContext(context, ORTHANC_PLUGIN_NAME);

    /* Check the version of the Orthanc core */
    if (OrthancPluginCheckVersion(OrthancPlugins::GetGlobalContext()) == 0)
    {
      char info[1024];
      sprintf(info, "Your version of Orthanc (%s) must be above %d.%d.%d to run this plugin",
              OrthancPlugins::GetGlobalContext()->orthancVersion,
              ORTHANC_PLUGINS_MINIMAL_MAJOR_NUMBER,
              ORTHANC_PLUGINS_MINIMAL_MINOR_NUMBER,
              ORTHANC_PLUGINS_MINIMAL_REVISION_NUMBER);
      OrthancPluginLogError(OrthancPlugins::GetGlobalContext(), info);
      return -1;
    }

#if ORTHANC_FRAMEWORK_VERSION_IS_ABOVE(1, 12, 4)
    Orthanc::Logging::InitializePluginContext(context, ORTHANC_PLUGIN_NAME);
#elif ORTHANC_FRAMEWORK_VERSION_IS_ABOVE(1, 7, 2)
    Orthanc::Logging::InitializePluginContext(context);
#else
    Orthanc::Logging::Initialize(context);
#endif

    Orthanc::InitializeFramework("", false);

    const bool hasCreateDicomStl = OrthancPlugins::CheckMinimalOrthancVersion(1, 12, 1);

    if (!hasCreateDicomStl)
    {
      LOG(WARNING) << "Your version of Orthanc (" << std::string(context->orthancVersion)
                   << ") is insufficient to create DICOM STL, it should be above 1.12.1";
    }    

    OrthancPlugins::SetDescription(ORTHANC_PLUGIN_NAME, "STL plugin for Orthanc.");

    OrthancPlugins::RegisterRestCallback<ServeFile>("/stl/app/(.*)", true);
    OrthancPlugins::RegisterRestCallback<ExtractStl>("/instances/([0-9a-f-]+)/stl", true);
    OrthancPlugins::RegisterRestCallback<ListStructures>("/stl/rt-struct/([0-9a-f-]+)", true);

    if (hasCreateDicomStl)
    {
      OrthancPlugins::RegisterRestCallback<EncodeStructureSet>("/stl/encode-rtstruct", true);
      OrthancPlugins::RegisterRestCallback<EncodeNifti>("/stl/encode-nifti", true);
    }

    OrthancPlugins::OrthancConfiguration globalConfiguration;
    OrthancPlugins::OrthancConfiguration configuration;
    globalConfiguration.GetSection(configuration, "STL");

#if ORTHANC_ENABLE_NEXUS == 1
    const bool enableNexus = configuration.GetBooleanValue("EnableNexus", false);

    if (enableNexus)
    {
      LOG(INFO) << "Support for Nexus is enabled";
      nexusCache_.SetMaximumSize(512 * 1024 * 1024);  // Cache of 512MB for Nexus
      OrthancPlugins::RegisterRestCallback<ExtractNexusModel>("/instances/([0-9a-f-]+)/nexus", true);
      OrthancPlugins::RegisterRestCallback<ServeNexusAssets>("/stl/nexus/(.*)", true);

      const bool hasCreateNexus_ = OrthancPlugins::CheckMinimalOrthancVersion(1, 9, 4);

      if (hasCreateNexus_)
      {
        OrthancPlugins::RegisterRestCallback<DicomizeNexusModel>("/stl/create-nexus", true);

        if (OrthancPluginRegisterPrivateDictionaryTag(
              context, ORTHANC_STL_PRIVATE_GROUP, ORTHANC_STL_CREATOR_ELEMENT, OrthancPluginValueRepresentation_LO,
              "PrivateCreator", 1, 1, ORTHANC_STL_PRIVATE_CREATOR) != OrthancPluginErrorCode_Success ||
            OrthancPluginRegisterPrivateDictionaryTag(
              context, ORTHANC_STL_PRIVATE_GROUP, ORTHANC_STL_NEXUS_ELEMENT, OrthancPluginValueRepresentation_OB,
              "NexusData", 1, 1, ORTHANC_STL_PRIVATE_CREATOR) != OrthancPluginErrorCode_Success)
        {
          LOG(ERROR) << "Cannot register the private DICOM tags for handling Nexus";
        }
      }
      else
      {
        LOG(WARNING) << "Your version of Orthanc (" << std::string(context->orthancVersion)
                     << ") is insufficient to create DICOM-ize Nexus models, it should be above 1.9.4";
      }
    }
    else
    {
      LOG(INFO) << "Support for Nexus is disabled";
    }
#else
    const bool enableNexus = false;
#endif

    // Extend the default Orthanc Explorer with custom JavaScript for STL
    std::string explorer;

    {
      Orthanc::EmbeddedResources::GetFileResource(explorer, Orthanc::EmbeddedResources::ORTHANC_EXPLORER);

      std::map<std::string, std::string> dictionary;
      dictionary["HAS_CREATE_DICOM_STL"] = (hasCreateDicomStl ? "true" : "false");
      dictionary["SHOW_NIFTI_BUTTON"] = (configuration.GetBooleanValue("EnableNIfTI", false) ? "true" : "false");
      dictionary["IS_NEXUS_ENABLED"] = (enableNexus ? "true" : "false");
      FillOrthancExplorerCreatorVersionUid(dictionary);

      explorer = Orthanc::Toolbox::SubstituteVariables(explorer, dictionary);

      OrthancPlugins::ExtendOrthancExplorer(ORTHANC_PLUGIN_NAME, explorer);
    }

    return 0;
  }


  ORTHANC_PLUGINS_API void OrthancPluginFinalize()
  {
    Orthanc::FinalizeFramework();
  }


  ORTHANC_PLUGINS_API const char* OrthancPluginGetName()
  {
    return ORTHANC_PLUGIN_NAME;
  }


  ORTHANC_PLUGINS_API const char* OrthancPluginGetVersion()
  {
    return ORTHANC_STL_VERSION;
  }
}
