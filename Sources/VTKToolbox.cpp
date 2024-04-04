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


#include "VTKToolbox.h"

#include <ChunkedBuffer.h>
#include <Compression/GzipCompressor.h>
#include <Endianness.h>
#include <OrthancException.h>
#include <Toolbox.h>

#include <vtkImageConstantPad.h>
#include <vtkImageResize.h>
#include <vtkMarchingCubes.h>
#include <vtkNew.h>
#include <vtkPolyDataNormals.h>
#include <vtkSmoothPolyDataFilter.h>
#include <vtkTriangle.h>

#include <nifti1_io.h>


namespace VTKToolbox
{
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


  void EncodeSTL(std::string& target /* out */,
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


  namespace
  {
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
  }


  void LoadNifti(vtkImageData* volume /* out */,
                 std::string& nifti /* in */)
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
}
