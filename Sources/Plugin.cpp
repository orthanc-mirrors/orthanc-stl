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

#define ORTHANC_PLUGIN_NAME  "stl"


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
static bool hasCreateDicomStl_;

void ServeFile(OrthancPluginRestOutput* output,
               const char* url,
               const OrthancPluginHttpRequest* request)
{
  if (request->method != OrthancPluginHttpMethod_Get)
  {
    OrthancPluginSendMethodNotAllowed(OrthancPlugins::GetGlobalContext(), output, "GET");
    return;
  }

  std::string file = request->groups[0];

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

    hasCreateDicomStl_ = OrthancPlugins::CheckMinimalOrthancVersion(1, 12, 1);
    LOG(WARNING) << "Hello!";

    if (!hasCreateDicomStl_)
    {
      LOG(WARNING) << "Your version of Orthanc (" << std::string(context->orthancVersion)
                   << ") is insufficient to create DICOM STL, it should be above 1.12.1";
    }    

    OrthancPlugins::SetDescription(ORTHANC_PLUGIN_NAME, "STL plugin for Orthanc.");

    OrthancPlugins::RegisterRestCallback<ServeFile>("/stl/app/(.*)", true);
    OrthancPlugins::RegisterRestCallback<ExtractStl>("/instances/([0-9a-f-]+)/stl", true);
    OrthancPlugins::RegisterRestCallback<ListStructures>("/stl/rt-struct/([0-9a-f-]+)", true);

    if (hasCreateDicomStl_)
    {
      OrthancPlugins::RegisterRestCallback<EncodeStructureSet>("/stl/encode-rtstruct", true);
      OrthancPlugins::RegisterRestCallback<EncodeNifti>("/stl/encode-nifti", true);
    }

    OrthancPlugins::OrthancConfiguration globalConfiguration;
    OrthancPlugins::OrthancConfiguration configuration;
    globalConfiguration.GetSection(configuration, "STL");

    // Extend the default Orthanc Explorer with custom JavaScript for STL
    std::string explorer;

    {
      Orthanc::EmbeddedResources::GetFileResource(explorer, Orthanc::EmbeddedResources::ORTHANC_EXPLORER);

      std::map<std::string, std::string> dictionary;
      dictionary["HAS_CREATE_DICOM_STL"] = (hasCreateDicomStl_ ? "true" : "false");
      dictionary["SHOW_NIFTI_BUTTON"] = (configuration.GetBooleanValue("EnableNIfTI", false) ? "true" : "false");
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
