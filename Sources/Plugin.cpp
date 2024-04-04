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


#include "../Resources/Orthanc/Plugins/OrthancPluginCppWrapper.h"

#include <EmbeddedResources.h>

#include <DicomFormat/DicomInstanceHasher.h>
#include <Compression/GzipCompressor.h>
#include <ChunkedBuffer.h>
#include <DicomParsing/FromDcmtkBridge.h>
#include <DicomParsing/ParsedDicomFile.h>
#include <Images/ImageProcessing.h>
#include <Logging.h>
#include <OrthancFramework.h>
#include <SerializationToolbox.h>
#include <SystemToolbox.h>

#include <vtkImageConstantPad.h>
#include <vtkImageData.h>
#include <vtkImageResize.h>
#include <vtkMarchingCubes.h>
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkPolyDataNormals.h>
#include <vtkSmoothPolyDataFilter.h>
#include <vtkTriangle.h>

#include <boost/thread/shared_mutex.hpp>

#include <nifti1_io.h>

#define ORTHANC_PLUGIN_NAME  "stl"


// Forward declaration
void ReadStaticAsset(std::string& target,
                     const std::string& path);


/**
 * As the Three.js static assets are gzipped by the
 * "EmbedStaticAssets.py" script, we use a cache to maintain the
 * uncompressed assets in order to avoid multiple gzip decodings.
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

  if (file == "viewer.html")
  {
    std::string s;
    Orthanc::EmbeddedResources::GetFileResource(s, Orthanc::EmbeddedResources::VIEWER_HTML);
    OrthancPluginAnswerBuffer(OrthancPlugins::GetGlobalContext(), output, s.c_str(), s.size(), Orthanc::EnumerationToString(Orthanc::MimeType_Html));
  }
  else if (file == "viewer.js")
  {
    std::string s;
    Orthanc::EmbeddedResources::GetFileResource(s, Orthanc::EmbeddedResources::VIEWER_JS);
    OrthancPluginAnswerBuffer(OrthancPlugins::GetGlobalContext(), output, s.c_str(), s.size(), Orthanc::EnumerationToString(Orthanc::MimeType_JavaScript));
  }
  else
  {
    cache_.Answer(output, file);
  }
}




#include <dcmtk/dcmdata/dcdeftag.h>
#include <dcmtk/dcmdata/dcfilefo.h>
#include <dcmtk/dcmdata/dcitem.h>
#include <dcmtk/dcmdata/dcsequen.h>
#include <dcmtk/dcmdata/dcuid.h>

class Extent2D : public boost::noncopyable
{
private:
  bool    isEmpty_;
  double  x1_;
  double  y1_;
  double  x2_;
  double  y2_;

  void CheckNotEmpty() const
  {
    if (isEmpty_)
    {
      throw Orthanc::OrthancException(Orthanc::ErrorCode_BadSequenceOfCalls);
    }
  }

public:
  Extent2D() :
    isEmpty_(true),
    x1_(0),
    y1_(0),
    x2_(0),
    y2_(0)
  {
  }

  bool IsEmpty() const
  {
    return isEmpty_;
  }

  double GetMinX() const
  {
    CheckNotEmpty();
    return x1_;
  }

  double GetMaxX() const
  {
    CheckNotEmpty();
    return x2_;
  }

  double GetMinY() const
  {
    CheckNotEmpty();
    return y1_;
  }

  double GetMaxY() const
  {
    CheckNotEmpty();
    return y2_;
  }

  double GetWidth() const
  {
    CheckNotEmpty();
    return x2_ - x1_;
  }

  double GetHeight() const
  {
    CheckNotEmpty();
    return y2_ - y1_;
  }

  void Add(double x,
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
};

static std::string GetStringValue(DcmItem& item,
                                  const DcmTagKey& key)
{
  const char* s = NULL;
  if (!item.findAndGetString(key, s).good() ||
      s == NULL)
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_BadFileFormat);
  }
  else
  {
    return Orthanc::Toolbox::StripSpaces(s);
  }
}


static void ListStructuresNames(std::set<std::string>& target,
                                Orthanc::ParsedDicomFile& source)
{
  target.clear();

  DcmSequenceOfItems* sequence = NULL;
  if (!source.GetDcmtkObject().getDataset()->findAndGetSequence(DCM_StructureSetROISequence, sequence).good() ||
      sequence == NULL)
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_BadFileFormat);
  }

  for (unsigned long i = 0; i < sequence->card(); i++)
  {
    DcmItem* item = sequence->getItem(i);
    if (item == NULL)
    {
      throw Orthanc::OrthancException(Orthanc::ErrorCode_BadFileFormat);
    }
    else
    {
      target.insert(GetStringValue(*item, DCM_ROIName));
    }
  }
}


static bool IsNear(double a,
                   double b)
{
  return std::abs(a - b) < 10.0 * std::numeric_limits<double>::epsilon();
}


class Vector3D
{
private:
  double x_;
  double y_;
  double z_;

public:
  Vector3D() :
    x_(0),
    y_(0),
    z_(0)
  {
  }

  Vector3D(double x,
           double y,
           double z) :
    x_(x),
    y_(y),
    z_(z)
  {
  }

  Vector3D(const Vector3D& from,
           const Vector3D& to) :
    x_(to.x_ - from.x_),
    y_(to.y_ - from.y_),
    z_(to.z_ - from.z_)
  {
  }

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

  double ComputeSquaredNorm() const
  {
    return x_ * x_ + y_ * y_ + z_ * z_;
  }

  double ComputeNorm() const
  {
    return sqrt(ComputeSquaredNorm());
  }

  void Normalize()
  {
    double norm = ComputeNorm();
    if (!IsNear(norm, 0))
    {
      x_ /= norm;
      y_ /= norm;
      z_ /= norm;
    }
  }

  static Vector3D CrossProduct(const Vector3D& u,
                               const Vector3D& v)
  {
    return Vector3D(u.GetY() * v.GetZ() - u.GetZ() * v.GetY(),
                    u.GetZ() * v.GetX() - u.GetX() * v.GetZ(),
                    u.GetX() * v.GetY() - u.GetY() * v.GetX());
  }

  static double DotProduct(const Vector3D& a,
                           const Vector3D& b)
  {
    return a.GetX() * b.GetX() + a.GetY() * b.GetY() + a.GetZ() * b.GetZ();
  }

  static bool AreParallel(const Vector3D& a,
                          const Vector3D& b)
  {
    if (IsNear(a.ComputeSquaredNorm(), 1) &&
        IsNear(b.ComputeSquaredNorm(), 1))
    {
      return IsNear(std::abs(Vector3D::DotProduct(a, b)), 1);
    }
    else
    {
      throw Orthanc::OrthancException(Orthanc::ErrorCode_ParameterOutOfRange,
                                      "Only applicable to normalized vectors");
    }
  }
};



static bool MyParseDouble(double& value,
                          const std::string& s)
{
#if 1
  // This version is much faster, as "ParseDouble()" internally uses
  // "boost::lexical_cast<double>()"
  char* end = NULL;
  value = strtod(s.c_str(), &end);
  return (end == s.c_str() + s.size());
#else
  return Orthanc::SerializationToolbox::ParseDouble(value, s);
#endif
}


class StructurePolygon : public boost::noncopyable
{
private:
  std::string            roiName_;
  std::string            referencedSopInstanceUid_;
  uint8_t                red_;
  uint8_t                green_;
  uint8_t                blue_;
  std::vector<Vector3D>  points_;

public:
  StructurePolygon(Orthanc::ParsedDicomFile& dicom,
                   unsigned long roiIndex,
                   unsigned long contourIndex)
  {
    DcmDataset& dataset = *dicom.GetDcmtkObject().getDataset();

    DcmItem* structure = NULL;
    DcmItem* roi = NULL;
    DcmItem* contour = NULL;
    DcmSequenceOfItems* referenced = NULL;

    if (!dataset.findAndGetSequenceItem(DCM_StructureSetROISequence, structure, roiIndex).good() ||
        structure == NULL ||
        !dataset.findAndGetSequenceItem(DCM_ROIContourSequence, roi, roiIndex).good() ||
        roi == NULL ||
        !roi->findAndGetSequenceItem(DCM_ContourSequence, contour, contourIndex).good() ||
        contour == NULL ||
        !contour->findAndGetSequence(DCM_ContourImageSequence, referenced).good() ||
        referenced == NULL ||
        referenced->card() != 1)
    {
      throw Orthanc::OrthancException(Orthanc::ErrorCode_BadFileFormat);
    }

    roiName_ = GetStringValue(*structure, DCM_ROIName);
    referencedSopInstanceUid_ = GetStringValue(*referenced->getItem(0), DCM_ReferencedSOPInstanceUID);

    if (GetStringValue(*contour, DCM_ContourGeometricType) != "CLOSED_PLANAR")
    {
      throw Orthanc::OrthancException(Orthanc::ErrorCode_BadFileFormat);
    }

    {
      std::vector<std::string> tokens;
      Orthanc::Toolbox::TokenizeString(tokens, GetStringValue(*roi, DCM_ROIDisplayColor), '\\');

      uint32_t r, g, b;
      if (tokens.size() != 3 ||
          !Orthanc::SerializationToolbox::ParseFirstUnsignedInteger32(r, tokens[0]) ||
          !Orthanc::SerializationToolbox::ParseFirstUnsignedInteger32(g, tokens[1]) ||
          !Orthanc::SerializationToolbox::ParseFirstUnsignedInteger32(b, tokens[2]) ||
          r > 255 ||
          g > 255 ||
          b > 255)
      {
        throw Orthanc::OrthancException(Orthanc::ErrorCode_BadFileFormat);
      }

      red_ = r;
      green_ = g;
      blue_ = b;
    }

    {
      std::vector<std::string> tokens;
      Orthanc::Toolbox::TokenizeString(tokens, GetStringValue(*contour, DCM_ContourData), '\\');

      const std::string s = GetStringValue(*contour, DCM_NumberOfContourPoints);

      uint32_t countPoints;
      if (!Orthanc::SerializationToolbox::ParseUnsignedInteger32(countPoints, s) ||
          tokens.size() != 3 * countPoints)
      {
        throw Orthanc::OrthancException(Orthanc::ErrorCode_BadFileFormat);
      }

      points_.reserve(countPoints);

      for (size_t i = 0; i < tokens.size(); i += 3)
      {
        double x, y, z;
        if (!MyParseDouble(x, tokens[i]) ||
            !MyParseDouble(y, tokens[i + 1]) ||
            !MyParseDouble(z, tokens[i + 2]))
        {
          throw Orthanc::OrthancException(Orthanc::ErrorCode_BadFileFormat);
        }

        points_.push_back(Vector3D(x, y, z));
      }

      assert(points_.size() == countPoints);
    }
  }

  const std::string& GetRoiName() const
  {
    return roiName_;
  }

  const std::string& GetReferencedSopInstanceUid() const
  {
    return referencedSopInstanceUid_;
  }

  size_t GetPointsCount() const
  {
    return points_.size();
  }

  const Vector3D& GetPoint(size_t i) const
  {
    if (i >= points_.size())
    {
      throw Orthanc::OrthancException(Orthanc::ErrorCode_ParameterOutOfRange);
    }
    else
    {
      return points_[i];
    }
  }

  bool IsCoplanar(Vector3D& normal) const
  {
    if (points_.size() < 3)
    {
      return false;
    }

    bool hasNormal = false;

    for (size_t i = 0; i < points_.size(); i++)
    {
      normal = Vector3D::CrossProduct(Vector3D(points_[1], points_[0]),
                                      Vector3D(points_[2], points_[0]));
      if (!IsNear(normal.ComputeNorm(), 0))
      {
        normal.Normalize();
        hasNormal = true;
      }
    }

    if (!hasNormal)
    {
      return false;
    }

    double a = Vector3D::DotProduct(points_[0], normal);

    for (size_t i = 1; i < points_.size(); i++)
    {
      double b = Vector3D::DotProduct(points_[i], normal);
      if (!IsNear(a, b))
      {
        return false;
      }
    }

    return true;
  }

  void Add(Extent2D& extent,
           const Vector3D& axisX,
           const Vector3D& axisY) const
  {
    assert(IsNear(1, axisX.ComputeNorm()));
    assert(IsNear(1, axisY.ComputeNorm()));

    for (size_t i = 0; i < points_.size(); i++)
    {
      extent.Add(Vector3D::DotProduct(axisX, points_[i]),
                 Vector3D::DotProduct(axisY, points_[i]));
    }
  }
};



struct IsNearPredicate
{
  bool operator() (const double& a,
                   const double& b)
  {
    return IsNear(a, b);
  }
};

static void RemoveDuplicateValues(std::vector<double>& v)
{
  IsNearPredicate predicate;
  std::vector<double>::iterator last = std::unique(v.begin(), v.end(), predicate);
  v.erase(last, v.end());
}


class StructureSet : public boost::noncopyable
{
private:
  std::vector<StructurePolygon*>  polygons_;
  std::string                     patientId_;
  std::string                     studyInstanceUid_;
  std::string                     seriesInstanceUid_;
  std::string                     sopInstanceUid_;
  bool                            hasFrameOfReferenceUid_;
  std::string                     frameOfReferenceUid_;

public:
  explicit StructureSet(Orthanc::ParsedDicomFile& dicom) :
    hasFrameOfReferenceUid_(false)
  {
    DcmDataset& dataset = *dicom.GetDcmtkObject().getDataset();
    patientId_ = GetStringValue(dataset, DCM_PatientID);
    studyInstanceUid_ = GetStringValue(dataset, DCM_StudyInstanceUID);
    seriesInstanceUid_ = GetStringValue(dataset, DCM_SeriesInstanceUID);
    sopInstanceUid_ = GetStringValue(dataset, DCM_SOPInstanceUID);

    DcmSequenceOfItems* frame = NULL;
    if (!dataset.findAndGetSequence(DCM_ReferencedFrameOfReferenceSequence, frame).good() ||
        frame == NULL)
    {
      throw Orthanc::OrthancException(Orthanc::ErrorCode_BadFileFormat);
    }

    if (frame->card() == 1)
    {
      const char* v = NULL;
      if (frame->getItem(0)->findAndGetString(DCM_FrameOfReferenceUID, v).good() &&
          v != NULL)
      {
        hasFrameOfReferenceUid_ = true;
        frameOfReferenceUid_.assign(v);
      }
    }

    DcmSequenceOfItems* rois = NULL;
    if (!dataset.findAndGetSequence(DCM_ROIContourSequence, rois).good() ||
        rois == NULL)
    {
      throw Orthanc::OrthancException(Orthanc::ErrorCode_BadFileFormat);
    }

    std::vector<DcmSequenceOfItems*> contours(rois->card());
    size_t countPolygons = 0;

    for (unsigned long i = 0; i < rois->card(); i++)
    {
      DcmSequenceOfItems* contour = NULL;
      if (!rois->getItem(i)->findAndGetSequence(DCM_ContourSequence, contour).good() ||
          contour == NULL)
      {
        throw Orthanc::OrthancException(Orthanc::ErrorCode_BadFileFormat);
      }
      else
      {
        contours[i] = contour;
        countPolygons += contour->card();
      }
    }

    polygons_.resize(countPolygons);

    size_t pos = 0;
    for (unsigned long i = 0; i < contours.size(); i++)
    {
      for (unsigned long j = 0; j < contours[i]->card(); j++, pos++)
      {
        polygons_[pos] = new StructurePolygon(dicom, i, j);
      }
    }

    assert(pos == countPolygons);
  }

  ~StructureSet()
  {
    for (size_t i = 0; i < polygons_.size(); i++)
    {
      assert(polygons_[i] != NULL);
      delete polygons_[i];
    }
  }

  const std::string& GetPatientId() const
  {
    return patientId_;
  }

  const std::string& GetStudyInstanceUid() const
  {
    return studyInstanceUid_;
  }

  const std::string& GetSeriesInstanceUid() const
  {
    return seriesInstanceUid_;
  }

  const std::string& GetSopInstanceUid() const
  {
    return sopInstanceUid_;
  }

  std::string HashStudy() const
  {
    Orthanc::DicomInstanceHasher hasher(patientId_, studyInstanceUid_, seriesInstanceUid_, sopInstanceUid_);
    return hasher.HashStudy();
  }

  size_t GetPolygonsCount() const
  {
    return polygons_.size();
  }

  const StructurePolygon& GetPolygon(size_t i) const
  {
    if (i >= polygons_.size())
    {
      throw Orthanc::OrthancException(Orthanc::ErrorCode_ParameterOutOfRange);
    }
    else
    {
      assert(polygons_[i] != NULL);
      return *polygons_[i];
    }
  }

  bool HasFrameOfReferenceUid() const
  {
    return hasFrameOfReferenceUid_;
  }

  const std::string& GetFrameOfReferenceUid() const
  {
    if (hasFrameOfReferenceUid_)
    {
      return frameOfReferenceUid_;
    }
    else
    {
      throw Orthanc::OrthancException(Orthanc::ErrorCode_BadSequenceOfCalls);
    }
  }
};


class StructureSetGeometry : public boost::noncopyable
{
private:
  bool      strict_;
  Vector3D  slicesNormal_;
  double    slicesSpacing_;
  double    minProjectionAlongNormal_;
  double    maxProjectionAlongNormal_;
  size_t    slicesCount_;

  bool LookupProjectionIndex(size_t& index,
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

    if (IsNear(d, round(d)))
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

public:
  StructureSetGeometry(const StructureSet& structures,
                       bool strict) :
    strict_(strict)
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
    RemoveDuplicateValues(projections);
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
      RemoveDuplicateValues(spacings);

      if (spacings.empty())
      {
        throw Orthanc::OrthancException(Orthanc::ErrorCode_InternalError);
      }

      slicesSpacing_ = spacings[spacings.size() / 10];  // Take the 90% percentile of smallest spacings
      assert(slicesSpacing_ > 0);
    }


    // Find the projection along the normal with the largest support

    bool first = true;
    size_t bestSupport;
    double bestProjection;

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
      it++;

      while (it != candidates.end())
      {
        double d = (projections[*it] - projections[reference]) / slicesSpacing_;
        if (IsNear(d, round(d)))
        {
          countSupport ++;
        }
        else
        {
          next.push_back(*it);
        }

        it++;
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


    // Compute the range of the projections

    minProjectionAlongNormal_ = bestProjection;
    maxProjectionAlongNormal_ = bestProjection;

    for (size_t i = 0; i < projections.size(); i++)
    {
      double d = (projections[i] - bestProjection) / slicesSpacing_;
      if (IsNear(d, round(d)))
      {
        minProjectionAlongNormal_ = std::min(minProjectionAlongNormal_, projections[i]);
        maxProjectionAlongNormal_ = std::max(maxProjectionAlongNormal_, projections[i]);
      }
    }

    double d = (maxProjectionAlongNormal_ - minProjectionAlongNormal_) / slicesSpacing_;
    if (IsNear(d, round(d)))
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

  const Vector3D& GetSlicesNormal() const
  {
    return slicesNormal_;
  }

  double GetSlicesSpacing() const
  {
    return slicesSpacing_;
  }

  double GetMinProjectionAlongNormal() const
  {
    return minProjectionAlongNormal_;
  }

  double GetMaxProjectionAlongNormal() const
  {
    return maxProjectionAlongNormal_;
  }

  bool ProjectAlongNormal(double& z,
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

  size_t GetSlicesCount() const
  {
    return slicesCount_;
  }

  bool LookupSliceIndex(size_t& slice,
                        const StructurePolygon& polygon) const
  {
    double z;
    return (ProjectAlongNormal(z, polygon) &&
            LookupProjectionIndex(slice, z));
  }
};




class XorFiller : public Orthanc::ImageProcessing::IPolygonFiller
{
private:
  Orthanc::ImageAccessor& target_;

public:
  XorFiller(Orthanc::ImageAccessor& target) :
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


static void WriteFloat(Orthanc::ChunkedBuffer& buffer,
                       float value,
                       Orthanc::Endianness endianness)
{
  switch (endianness)
  {
    case Orthanc::Endianness_Little:
      buffer.AddChunk(&value, sizeof(float));
      break;

    case Orthanc::Endianness_Big:
    {
      uint8_t tmp[4];
      tmp[0] = reinterpret_cast<uint8_t*>(&value) [3];
      tmp[1] = reinterpret_cast<uint8_t*>(&value) [2];
      tmp[2] = reinterpret_cast<uint8_t*>(&value) [1];
      tmp[3] = reinterpret_cast<uint8_t*>(&value) [0];
      buffer.AddChunk(&tmp, sizeof(float));
      break;
    }

    default:
      throw Orthanc::OrthancException(Orthanc::ErrorCode_NotImplemented);
  }
}


static void WriteInteger(Orthanc::ChunkedBuffer& buffer,
                         uint32_t value,
                         Orthanc::Endianness endianness)
{
  switch (endianness)
  {
    case Orthanc::Endianness_Little:
      buffer.AddChunk(&value, sizeof(uint32_t));
      break;

    case Orthanc::Endianness_Big:
    {
      uint8_t tmp[4];
      tmp[0] = reinterpret_cast<uint8_t*>(&value) [3];
      tmp[1] = reinterpret_cast<uint8_t*>(&value) [2];
      tmp[2] = reinterpret_cast<uint8_t*>(&value) [1];
      tmp[3] = reinterpret_cast<uint8_t*>(&value) [0];
      buffer.AddChunk(&tmp, sizeof(uint32_t));
      break;
    }

    default:
      throw Orthanc::OrthancException(Orthanc::ErrorCode_NotImplemented);
  }
}


static void EncodeSTL(std::string& target /* out */,
                      vtkPolyData& mesh /* in */)
{
  const Orthanc::Endianness endianness = Orthanc::Toolbox::DetectEndianness();

  Orthanc::ChunkedBuffer buffer;

  uint8_t header[80];
  memset(header, 0, sizeof(header));
  buffer.AddChunk(header, sizeof(header));  // This doesn't depend on endianness

  WriteInteger(buffer, mesh.GetNumberOfCells(), endianness);

  for (vtkIdType i = 0; i < mesh.GetNumberOfCells(); i++)
  {
    vtkCell* cell = mesh.GetCell(i);
    vtkTriangle* triangle = dynamic_cast<vtkTriangle*>(cell);

    double p0[3];
    double p1[3];
    double p2[3];
    triangle->GetPoints()->GetPoint(0, p0);
    triangle->GetPoints()->GetPoint(1, p1);
    triangle->GetPoints()->GetPoint(2, p2);

    double normal[3];
    vtkTriangle::ComputeNormal(p0, p1, p2, normal);

    WriteFloat(buffer, normal[0], endianness);
    WriteFloat(buffer, normal[1], endianness);
    WriteFloat(buffer, normal[2], endianness);
    WriteFloat(buffer, p0[0], endianness);
    WriteFloat(buffer, p0[1], endianness);
    WriteFloat(buffer, p0[2], endianness);
    WriteFloat(buffer, p1[0], endianness);
    WriteFloat(buffer, p1[1], endianness);
    WriteFloat(buffer, p1[2], endianness);
    WriteFloat(buffer, p2[0], endianness);
    WriteFloat(buffer, p2[1], endianness);
    WriteFloat(buffer, p2[2], endianness);

    uint16_t a = 0;
    buffer.AddChunk(&a, sizeof(a));  // This doesn't depend on endianness
  }

  buffer.Flatten(target);
}


bool EncodeVolume(std::string& stl,
                  vtkImageData* volume,
                  unsigned int resolution,
                  bool smooth)
{
  if (volume == NULL)
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_NullPointer);
  }

  vtkNew<vtkImageResize> resize;
  resize->SetOutputDimensions(resolution, resolution, resolution);
  resize->SetInputData(volume);
  resize->Update();

  vtkNew<vtkImageConstantPad> padding;
  padding->SetConstant(0);
  padding->SetOutputNumberOfScalarComponents(1);
  padding->SetOutputWholeExtent(-1, resolution, -1, resolution, -1, resolution);
  padding->SetInputData(resize->GetOutput());
  padding->Update();

  double range[2];
  padding->GetOutput()->GetScalarRange(range);

  const double isoValue = (range[0] + range[1]) / 2.0;

  vtkNew<vtkMarchingCubes> surface;
  surface->SetInputData(padding->GetOutput());
  surface->ComputeNormalsOn();
  surface->SetValue(0, isoValue);
  surface->Update();

  if (smooth)
  {
    vtkNew<vtkSmoothPolyDataFilter> smoothFilter;
    // Apply volume smoothing
    // https://examples.vtk.org/site/Cxx/PolyData/SmoothPolyDataFilter/
    smoothFilter->SetInputConnection(surface->GetOutputPort());
    smoothFilter->SetNumberOfIterations(15);
    smoothFilter->SetRelaxationFactor(0.1);
    smoothFilter->FeatureEdgeSmoothingOff();
    smoothFilter->BoundarySmoothingOn();
    smoothFilter->Update();

    vtkNew<vtkPolyDataNormals> normalGenerator;
    normalGenerator->SetInputConnection(smoothFilter->GetOutputPort());
    normalGenerator->ComputePointNormalsOn();
    normalGenerator->ComputeCellNormalsOn();
    normalGenerator->Update();

    EncodeSTL(stl, *normalGenerator->GetOutput());
  }
  else
  {
    EncodeSTL(stl, *surface->GetOutput());
  }

  return true;
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
                MyParseDouble(y1, items[0]) &&
                MyParseDouble(y2, items[1]) &&
                MyParseDouble(y3, items[2]) &&
                MyParseDouble(x1, items[3]) &&
                MyParseDouble(x2, items[4]) &&
                MyParseDouble(x3, items[5]))
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

  if (!IsNear(1, geometry.GetSlicesNormal().ComputeNorm()))
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_InternalError);
  }

  Vector3D axisX, axisY;
  GetReferencedVolumeAxes(axisX, axisY, structureSet);

  Vector3D axisZ = Vector3D::CrossProduct(axisX, axisY);

  if (!IsNear(1, axisX.ComputeNorm()) ||
      !IsNear(1, axisY.ComputeNorm()) ||
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
        }

        Orthanc::ImageAccessor slice;
        slice.AssignWritable(Orthanc::PixelFormat_Grayscale8, resolution, resolution, resolution /* pitch */,
                             reinterpret_cast<uint8_t*>(volume->GetScalarPointer()) + z * resolution * resolution);

        XorFiller filler(slice);
        Orthanc::ImageProcessing::FillPolygon(filler, points);
      }
    }
  }

  volume->SetSpacing(extent.GetWidth() / static_cast<double>(resolution),
                     extent.GetHeight() / static_cast<double>(resolution),
                     geometry.GetSlicesSpacing());

  // TODO
  // volume->SetOrigin()

  return EncodeVolume(stl, volume.Get(), resolution, smooth);
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
  ListStructuresNames(names, *dicom);

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
  if (GetStringValue(dataset, DCM_MIMETypeOfEncapsulatedDocument) != Orthanc::MIME_STL ||
      GetStringValue(dataset, DCM_SOPClassUID) != UID_EncapsulatedSTLStorage ||
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



class NiftiHeader : public boost::noncopyable
{
private:
  nifti_image* image_;

public:
  NiftiHeader(const std::string& nifti)
  {
    nifti_1_header header;
    if (nifti.size() < sizeof(header))
    {
      throw Orthanc::OrthancException(Orthanc::ErrorCode_BadFileFormat);
    }

    memcpy(&header, nifti.c_str(), sizeof(header));
    if (!nifti_hdr_looks_good(&header))
    {
      throw Orthanc::OrthancException(Orthanc::ErrorCode_BadFileFormat);
    }

    image_ = nifti_convert_nhdr2nim(header, "dummy_filename");
    if (image_ == NULL)
    {
      throw Orthanc::OrthancException(Orthanc::ErrorCode_BadFileFormat);
    }
  }

  ~NiftiHeader()
  {
    nifti_image_free(image_);
  }

  const nifti_image& GetInfo() const
  {
    assert(image_ != NULL);
    return *image_;
  }
};


static void LoadNifti(vtkImageData* volume,
                      std::string& nifti)
{
  if (volume == NULL)
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_NullPointer);
  }

  const uint8_t* p = reinterpret_cast<const uint8_t*>(nifti.c_str());

  if (nifti.size() >= 2 &&
      p[0] == 0x1f &&
      p[1] == 0x8b)
  {
    Orthanc::GzipCompressor compressor;
    std::string uncompressed;
    Orthanc::IBufferCompressor::Uncompress(uncompressed, compressor, nifti);
    nifti.swap(uncompressed);
  }

  NiftiHeader header(nifti);

  if (header.GetInfo().ndim != 3)
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_BadFileFormat,
                                    "Only 3D NIfTI volumes are allowed");
  }

  size_t itemSize;
  int vtkType;

  switch (header.GetInfo().datatype)
  {
    case DT_UNSIGNED_CHAR:
      itemSize = 1;
      vtkType = VTK_UNSIGNED_CHAR;
      break;

    case DT_FLOAT:
      itemSize = sizeof(float);
      vtkType = VTK_FLOAT;
      break;

    case DT_DOUBLE:
      itemSize = sizeof(double);
      vtkType = VTK_DOUBLE;
      break;

    default:
      throw Orthanc::OrthancException(Orthanc::ErrorCode_NotImplemented);
  }

  assert(static_cast<int>(header.GetInfo().nvox) == header.GetInfo().nx * header.GetInfo().ny * header.GetInfo().nz);

  const size_t pixelDataOffset = sizeof(nifti_1_header) + 4 /* extension */;

  if (nifti.size() != pixelDataOffset + header.GetInfo().nvox * itemSize)
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_CorruptedFile);
  }

  volume->SetDimensions(header.GetInfo().nx, header.GetInfo().ny, header.GetInfo().nz);
  volume->AllocateScalars(vtkType, 1);
  volume->SetSpacing(header.GetInfo().dx, header.GetInfo().dy, header.GetInfo().dz);
  memcpy(volume->GetScalarPointer(), &nifti[pixelDataOffset], header.GetInfo().nvox * itemSize);
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
  LoadNifti(volume.Get(), nifti);

  std::string stl;
  if (!EncodeVolume(stl, volume.Get(), resolution, smooth))
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
    OrthancPlugins::SetGlobalContext(context);

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

#if ORTHANC_FRAMEWORK_VERSION_IS_ABOVE(1, 7, 2)
    Orthanc::Logging::InitializePluginContext(context);
#else
    Orthanc::Logging::Initialize(context);
#endif

    Orthanc::InitializeFramework("", false);

    hasCreateDicomStl_ = OrthancPlugins::CheckMinimalOrthancVersion(1, 12, 1);

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

    // Extend the default Orthanc Explorer with custom JavaScript for STL
    std::string explorer;

    {
      Orthanc::EmbeddedResources::GetFileResource(explorer, Orthanc::EmbeddedResources::ORTHANC_EXPLORER);

      std::map<std::string, std::string> dictionary;
      dictionary["HAS_CREATE_DICOM_STL"] = (hasCreateDicomStl_ ? "true" : "false");
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
