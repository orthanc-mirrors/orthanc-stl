/**
 * SPDX-FileCopyrightText: 2023 Sebastien Jodogne, UCLouvain, Belgium
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/**
 * STL plugin for Orthanc
 * Copyright (C) 2023 Sebastien Jodogne, UCLouvain, Belgium
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

  double ComputeNorm() const
  {
    return sqrt(x_ * x_ + y_ * y_ + z_ * z_);
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
};



static bool MyParseDouble(double& value,
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
  bool                            hasGeometry_;
  Vector3D                        slicesNormal_;
  double                          slicesSpacing_;
  double                          minProjectionAlongNormal_;
  double                          maxProjectionAlongNormal_;
  std::string                     patientId_;
  std::string                     studyInstanceUid_;
  std::string                     seriesInstanceUid_;
  std::string                     sopInstanceUid_;
  bool                            hasFrameOfReferenceUid_;
  std::string                     frameOfReferenceUid_;

  void ComputeGeometry()
  {
    std::list<double> positionsList;
    hasGeometry_ = false;

    for (size_t i = 0; i < polygons_.size(); i++)
    {
      assert(polygons_[i] != NULL);

      Vector3D n;
      if (polygons_[i]->IsCoplanar(n))
      {
        const Vector3D& point = polygons_[i]->GetPoint(0);
        double z = Vector3D::DotProduct(point, n);

        if (!hasGeometry_)
        {
          hasGeometry_ = true;
          slicesNormal_ = n;
          minProjectionAlongNormal_ = z;
          maxProjectionAlongNormal_ = z;
        }
        else if (!IsNear(std::abs(Vector3D::DotProduct(n, slicesNormal_)), 1))
        {
          hasGeometry_ = false;

          // RT-STRUCT with non-parallel slices
          throw Orthanc::OrthancException(Orthanc::ErrorCode_NotImplemented);
        }
        else
        {
          minProjectionAlongNormal_ = std::min(minProjectionAlongNormal_, z);
          maxProjectionAlongNormal_ = std::max(maxProjectionAlongNormal_, z);
        }

        positionsList.push_back(Vector3D::DotProduct(n, point));
      }
    }

    if (hasGeometry_)
    {
      std::vector<double> positions(positionsList.begin(), positionsList.end());
      assert(!positions.empty());

      std::sort(positions.begin(), positions.end());
      RemoveDuplicateValues(positions);
      assert(!positions.empty());

      if (positions.size() == 1)
      {
        hasGeometry_ = false;
        return;
      }

      std::vector<double> offsets;
      offsets.resize(positions.size() - 1);

      for (size_t i = 0; i < offsets.size(); i++)
      {
        offsets[i] = positions[i + 1] - positions[i];
        assert(offsets[i] > 0);
      }

      std::sort(offsets.begin(), offsets.end());
      RemoveDuplicateValues(offsets);

      slicesSpacing_ = offsets[0];

      for (size_t i = 1; i < offsets.size(); i++)
      {
        double d = offsets[i] / slicesSpacing_;
        if (!IsNear(d, round(d)))
        {
          // Irregular spacing between the slices
          hasGeometry_ = false;
          break;
        }
      }
    }
  }

  void CheckHasGeometry() const
  {
    if (!hasGeometry_)
    {
      throw Orthanc::OrthancException(Orthanc::ErrorCode_BadSequenceOfCalls);
    }
  }

public:
  explicit StructureSet(Orthanc::ParsedDicomFile& dicom) :
    hasGeometry_(false),
    slicesSpacing_(0),
    minProjectionAlongNormal_(0),
    maxProjectionAlongNormal_(0),
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

    ComputeGeometry();
  }

  ~StructureSet()
  {
    for (size_t i = 0; i < polygons_.size(); i++)
    {
      assert(polygons_[i] != NULL);
      delete polygons_[i];
    }
  }

  const std::string& GetPatient() const
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
    std::string s;
    Orthanc::Toolbox::ComputeSHA1(s, patientId_ + "|" + studyInstanceUid_);
    return s;
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

  bool HasGeometry() const
  {
    return hasGeometry_;
  }

  const Vector3D& GetSlicesNormal() const
  {
    CheckHasGeometry();
    return slicesNormal_;
  }

  double GetSlicesSpacing() const
  {
    CheckHasGeometry();
    return slicesSpacing_;
  }

  double GetMinProjectionAlongNormal() const
  {
    CheckHasGeometry();
    return minProjectionAlongNormal_;
  }

  double GetMaxProjectionAlongNormal() const
  {
    CheckHasGeometry();
    return maxProjectionAlongNormal_;
  }

  double ProjectAlongNormal(const StructurePolygon& polygon) const
  {
    CheckHasGeometry();
    return Vector3D::DotProduct(slicesNormal_, polygon.GetPoint(0));
  }

  size_t GetSlicesCount() const
  {
    CheckHasGeometry();
    double c = (maxProjectionAlongNormal_ - minProjectionAlongNormal_) / slicesSpacing_;
    assert(c >= 0);

    if (!IsNear(c, round(c)))
    {
      throw Orthanc::OrthancException(Orthanc::ErrorCode_InternalError);
    }
    else
    {
      return static_cast<size_t>(round(c)) + 1;
    }
  }

  bool LookupSliceIndex(size_t& slice,
                        const StructurePolygon& polygon) const
  {
    CheckHasGeometry();

    double z = ProjectAlongNormal(polygon);

    if (z < minProjectionAlongNormal_ ||
        z > maxProjectionAlongNormal_)
    {
      return false;
    }
    else
    {
      double c = (z - minProjectionAlongNormal_) / slicesSpacing_;

      if (IsNear(c, round(c)))
      {
        slice = static_cast<size_t>(round(c));
        return true;
      }
      else
      {
        return false;
      }
    }
  }

  bool LookupReferencedSopInstanceUid(std::string& sopInstanceUid) const
  {
    if (HasGeometry())
    {
      for (size_t i = 0; i < polygons_.size(); i++)
      {
        assert(polygons_[i] != NULL);

        Vector3D n;
        if (polygons_[i]->IsCoplanar(n) &&
            Vector3D::DotProduct(n, slicesNormal_))
        {
          sopInstanceUid = polygons_[i]->GetReferencedSopInstanceUid();
          return true;
        }
      }
    }

    return false;
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


static void EncodeSTL(std::string& target /* out */,
                      vtkPolyData& mesh /* in */)
{
  // TODO - Conversion to little endian on big endian

  Orthanc::ChunkedBuffer buffer;

  uint8_t header[80];
  memset(header, 0, sizeof(header));
  buffer.AddChunk(header, sizeof(header));

  uint32_t n = mesh.GetNumberOfCells();
  buffer.AddChunk(&n, sizeof(n));

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

    float d[4 * 3] = {
      static_cast<float>(normal[0]), static_cast<float>(normal[1]), static_cast<float>(normal[2]),
      static_cast<float>(p0[0]), static_cast<float>(p0[1]), static_cast<float>(p0[2]),
      static_cast<float>(p1[0]), static_cast<float>(p1[1]), static_cast<float>(p1[2]),
      static_cast<float>(p2[0]), static_cast<float>(p2[1]), static_cast<float>(p2[2]) };
    buffer.AddChunk(d, sizeof(d));

    uint16_t a = 0;
    buffer.AddChunk(&a, sizeof(a));
  }

  buffer.Flatten(target);
}


bool EncodeStructureSetMesh(std::string& stl,
                            const StructureSet& structureSet,
                            const std::set<std::string>& roiNames,
                            unsigned int resolution,
                            bool smooth)
{
  if (!structureSet.HasGeometry())
  {
    return false;
  }

  if (resolution < 1)
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_ParameterOutOfRange);
  }

  if (!IsNear(1, structureSet.GetSlicesNormal().ComputeNorm()))
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_InternalError);
  }

  // TODO - Axes could be retrieved from the referenced CT volume
  Vector3D axisX(1, 0, 0);
  Vector3D axisY = Vector3D::CrossProduct(structureSet.GetSlicesNormal(), axisX);

  if (!IsNear(1, axisX.ComputeNorm()) ||
      !IsNear(1, axisY.ComputeNorm()))
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_InternalError);
  }

  Extent2D extent;
  for (size_t i = 0; i < structureSet.GetPolygonsCount(); i++)
  {
    structureSet.GetPolygon(i).Add(extent, axisX, axisY);
  }

  const int depth = structureSet.GetSlicesCount();

  vtkNew<vtkImageData> volume;
  volume->SetDimensions(resolution, resolution, depth);
  volume->AllocateScalars(VTK_UNSIGNED_CHAR, 1);

  assert(sizeof(unsigned char) == 1);
  memset(volume->GetScalarPointer(), 0, resolution * resolution * depth);

  for (size_t i = 0; i < structureSet.GetPolygonsCount(); i++)
  {
    const StructurePolygon& polygon = structureSet.GetPolygon(i);
    if (roiNames.find(polygon.GetRoiName()) == roiNames.end())
    {
      // This polygon doesn't correspond to a ROI of interest
      continue;
    }

    size_t j;
    if (!structureSet.LookupSliceIndex(j, polygon))
    {
      throw Orthanc::OrthancException(Orthanc::ErrorCode_InternalError);
    }

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
                         reinterpret_cast<uint8_t*>(volume->GetScalarPointer()) + j * resolution * resolution);

    XorFiller filler(slice);
    Orthanc::ImageProcessing::FillPolygon(filler, points);
  }

  vtkNew<vtkImageResize> resize;
  resize->SetOutputDimensions(resolution, resolution, resolution);
  resize->SetInputData(volume.Get());
  resize->Update();

  resize->GetOutput()->SetSpacing(
    extent.GetWidth() / static_cast<double>(resolution),
    extent.GetHeight() / static_cast<double>(resolution),
    (structureSet.GetMaxProjectionAlongNormal() - structureSet.GetMinProjectionAlongNormal()) / static_cast<double>(resolution));

  // TODO
  // resize->GetOutput()->SetOrigin()

  vtkNew<vtkImageConstantPad> padding;
  padding->SetConstant(0);
  padding->SetOutputNumberOfScalarComponents(1);
  padding->SetOutputWholeExtent(-1, resolution, -1, resolution, -1, resolution);
  padding->SetInputData(resize->GetOutput());
  padding->Update();

  vtkNew<vtkMarchingCubes> surface;
  surface->SetInputData(padding->GetOutput());
  surface->ComputeNormalsOn();
  surface->SetValue(0, 128 /*isoValue*/);
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


void Encode(OrthancPluginRestOutput* output,
            const char* url,
            const OrthancPluginHttpRequest* request)
{
  static const char* const KEY_INSTANCE = "Instance";
  static const char* const KEY_RESOLUTION = "Resolution";
  static const char* const KEY_ROI_NAMES = "RoiNames";
  static const char* const KEY_SMOOTH = "Smooth";
  static const char* const KEY_TAGS = "Tags";

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

  std::set<std::string> roiNames;
  Orthanc::SerializationToolbox::ReadSetOfStrings(roiNames, body, KEY_ROI_NAMES);

  std::unique_ptr<Orthanc::ParsedDicomFile> dicom(LoadInstance(instanceId));

  StructureSet structureSet(*dicom);

  std::string stl;
  if (!EncodeStructureSetMesh(stl, structureSet, roiNames, resolution, smooth))
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_BadFileFormat, "Cannot encode STL");
  }
  else
  {
    std::string content;
    Orthanc::Toolbox::EncodeDataUriScheme(content, "model/stl", stl);

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
      std::string description;

      if (dicom->GetTagValue(description, Orthanc::DICOM_TAG_SERIES_DESCRIPTION))
      {
        description += ": ";
      }
      else
      {
        description.clear();
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
          description += ", ";
        }

        description += *it;
      }

      normalized[Orthanc::DICOM_TAG_SERIES_DESCRIPTION.Format()] = description;
    }

    AddDefaultTagValue(normalized, Orthanc::DICOM_TAG_SERIES_NUMBER, "1");

    std::string s;
    if (structureSet.HasFrameOfReferenceUid())
    {
      s = structureSet.GetFrameOfReferenceUid();
    }
    else
    {
      s = Orthanc::FromDcmtkBridge::GenerateUniqueIdentifier(Orthanc::ResourceType_Instance);
    }

    AddDefaultTagValue(normalized, Orthanc::DICOM_TAG_FRAME_OF_REFERENCE_UID, s);
    AddDefaultTagValue(normalized, Orthanc::DICOM_TAG_INSTANCE_NUMBER, "1");

    const std::string title = "STL model generated from DICOM RT-STRUCT";

    AddDefaultTagValue(normalized, DCM_BurnedInAnnotation, "NO");
    AddDefaultTagValue(normalized, DCM_DeviceSerialNumber, ORTHANC_STL_VERSION);
    AddDefaultTagValue(normalized, DCM_DocumentTitle, title);
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
      item["CodeMeaning"] = title;

      normalized[MEASUREMENT_UNITS_CODE_SEQUENCE.Format()].append(item);
    }

    Json::Value create;
    create["Content"] = content;
    create["Parent"] = structureSet.HashStudy();
    create["Tags"] = normalized;

    Json::Value answer;
    if (OrthancPlugins::RestApiPost(answer, "/tools/create-dicom", create.toStyledString(), false))
    {
      std::string s = answer.toStyledString();
      OrthancPluginAnswerBuffer(OrthancPlugins::GetGlobalContext(), output, s.c_str(), s.size(), Orthanc::MIME_JSON);
    }
    else
    {
      throw Orthanc::OrthancException(Orthanc::ErrorCode_BadRequest, "Cannot create DICOM from STL");
    }
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
      GetStringValue(dataset, DCM_SOPClassUID) != "1.2.840.10008.5.1.4.1.1.104.3" ||
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

    OrthancPluginSetDescription(context, "STL plugin for Orthanc.");

    OrthancPlugins::RegisterRestCallback<ServeFile>("/stl/app/(.*)", true);
    OrthancPlugins::RegisterRestCallback<ExtractStl>("/instances/([0-9a-f-]+)/stl", true);
    OrthancPlugins::RegisterRestCallback<ListStructures>("/stl/rt-struct/([0-9a-f-]+)", true);

    if (hasCreateDicomStl_)
    {
      OrthancPlugins::RegisterRestCallback<Encode>("/stl/encode", true);
    }

    // Extend the default Orthanc Explorer with custom JavaScript for STL
    std::string explorer;

    {
      Orthanc::EmbeddedResources::GetFileResource(explorer, Orthanc::EmbeddedResources::ORTHANC_EXPLORER);

      std::map<std::string, std::string> dictionary;
      dictionary["HAS_CREATE_DICOM_STL"] = (hasCreateDicomStl_ ? "true" : "false");
      explorer = Orthanc::Toolbox::SubstituteVariables(explorer, dictionary);

      OrthancPluginExtendOrthancExplorer(OrthancPlugins::GetGlobalContext(), explorer.c_str());
    }

    return 0;
  }


  ORTHANC_PLUGINS_API void OrthancPluginFinalize()
  {
    Orthanc::FinalizeFramework();
  }


  ORTHANC_PLUGINS_API const char* OrthancPluginGetName()
  {
    return "stl";
  }


  ORTHANC_PLUGINS_API const char* OrthancPluginGetVersion()
  {
    return ORTHANC_STL_VERSION;
  }
}
