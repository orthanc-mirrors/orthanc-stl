#!/usr/bin/python3

# SPDX-FileCopyrightText: 2023-2025 Sebastien Jodogne, ICTEAM UCLouvain, Belgium
# SPDX-License-Identifier: GPL-3.0-or-later

# STL plugin for Orthanc
# Copyright (C) 2023-2025 Sebastien Jodogne, ICTEAM UCLouvain, Belgium
#
# This program is free software: you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.


import gzip
import hashlib
import io
import os
import sys

if len(sys.argv) <= 2:
    raise Exception('Usage: %s [target C++] [folder prefixes] [source folders]' % sys.argv[0])

SOURCES = sys.argv[2:]
TARGET = sys.argv[1]

if len(SOURCES) % 2 != 0:
    raise Exception('There must be an even number of sources')

FILES = []

for i in range(len(SOURCES) // 2):
    prefix = SOURCES[i]
    if '/' in prefix:
        raise Exception('Prefix cannot contain a slash, but found: %s' % prefix)

    folder = SOURCES[i + len(SOURCES) // 2]

    if not os.path.isdir(folder):
        raise Exception('Nonexistent source folder: %s' % folder)

    for root, dirs, files in os.walk(folder):
        files.sort()
        dirs.sort()

        for f in files:
            FILES.append({
                'path' : os.path.join(root, f),
                'key' : prefix + '/' + os.path.relpath(os.path.join(root, f), folder),
            })

FILES = sorted(FILES, key = lambda x: x['key'])


def EncodeFileAsCString(f, variable, content):
    f.write('static const uint8_t %s[%d] = \n  "' % (variable, len(content) + 1))
    
    column = 0
        
    for c in content:
        
        if sys.version_info < (3, 0):
            # Python 2.7
            i = ord(c)
        else:
            # Python 3.x
            i = c
            
        if i < 32 or i >= 127 or i == ord('?'):
            f.write('\\{0:03o}'.format(i))
        elif i in [ ord('"'), ord('\\') ]:
            f.write('\\' + chr(i))
        else:
            f.write(chr(i))

        column += 1
        if column >= 120:
            f.write('"\n  "')
            column = 0
            
    f.write('";\n\n')


def WriteChecksum(f, variable, content):
    md5 = hashlib.md5(content).hexdigest()
    g.write('static const char* %s = "%s";\n\n' % (variable, md5))


with open(TARGET, 'w') as g:
    g.write('''
#include <stdint.h>
#include <Compression/GzipCompressor.h>
#include <OrthancException.h>
#include <Toolbox.h>

static void Uncompress(std::string& target, const void* data, size_t size, const std::string& md5Expected)
{
  Orthanc::GzipCompressor compressor;
  compressor.Uncompress(target, data, size);
  std::string md5Actual;
  Orthanc::Toolbox::ComputeMD5(md5Actual, target);
  if (md5Actual != md5Expected)
  {
    throw Orthanc::OrthancException(Orthanc::ErrorCode_CorruptedFile);
  }
}

''')    

    index = {}
    count = 0

    for file in FILES:
        variable = 'data_%06d' % count
        count += 1

        with open(file['path'], 'rb') as f:
            content = f.read()

        if sys.version_info < (3, 0):
            # Python 2.7
            fileobj = io.BytesIO()
            gzip.GzipFile(fileobj=fileobj, mode='w', mtime=0).write(content)
            compressed = fileobj.getvalue()
        else:
            # Python 3.x
            compressed = gzip.compress(content, mtime=0)

        EncodeFileAsCString(g, variable, compressed)
        WriteChecksum(g, variable + '_md5', content)

        file['variable'] = variable
    
    g.write('void ReadStaticAsset(std::string& target, const std::string& path)\n')
    g.write('{\n')
    for file in FILES:
        g.write('  if (path == "%s")\n' % file['key'])
        g.write('  {\n')
        g.write('    Uncompress(target, %s, sizeof(%s) - 1, %s_md5);\n' % (file['variable'], file['variable'], file['variable']))
        g.write('    return;\n')
        g.write('  }\n\n')

    g.write('  throw Orthanc::OrthancException(Orthanc::ErrorCode_InexistentItem, "Unknown static asset: " + path);\n')
    g.write('}\n')
