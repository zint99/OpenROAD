///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2019, Nefelus Inc
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include "dbCore.h"
#include "dbVector.h"
#include "odb/dbId.h"
#include "odb/dbTypes.h"
#include "odb/odb.h"

namespace odb {

class _dbWire;
class _dbNet;

struct _dbWireFlags
{
  uint _is_global : 1;
  uint _spare_bits : 31;
};

class _dbWire : public _dbObject
{
 public:
  _dbWireFlags _flags;
  dbVector<int> _data;
  dbVector<unsigned char> _opcodes;
  dbId<_dbNet> _net;

  _dbWire(_dbDatabase*)
  {
    _flags._is_global = 0;
    _flags._spare_bits = 0;
  }

  _dbWire(_dbDatabase*, const _dbWire& w)
      : _flags(w._flags), _data(w._data), _opcodes(w._opcodes), _net(w._net)
  {
  }

  ~_dbWire() {}

  uint length() { return _opcodes.size(); }

  bool operator==(const _dbWire& rhs) const;
  bool operator!=(const _dbWire& rhs) const { return !operator==(rhs); }
  void collectMemInfo(MemInfo& info);
};

dbOStream& operator<<(dbOStream& stream, const _dbWire& wire);
dbIStream& operator>>(dbIStream& stream, _dbWire& wire);

}  // namespace odb
