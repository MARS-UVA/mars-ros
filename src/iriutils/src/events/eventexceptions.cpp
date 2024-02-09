// Copyright (C) 2009-2010 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
// Author Sergi Hernandez  (shernand@iri.upc.edu)
// All rights reserved.
//
// This file is part of iriutils
// iriutils is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "eventexceptions.h"
#include <string.h>
#include <stdio.h>

const std::string event_exception_msg="[CEvent class] - ";
const std::string event_server_exception_msg="[CEventServer class] - ";
const std::string event_timeout_exception_msg="[Timeout] - ";

CEventException::CEventException(const std::string& where, const std::string& error_msg,const std::string& event_id):CException(where,event_exception_msg)
{
  this->error_msg+=error_msg;
  this->error_msg+=" - ";
  this->error_msg+=event_id;
}

CEventServerException::CEventServerException(const std::string& where, const std::string& error_msg,const std::string& event_id):CException(where,event_server_exception_msg)
{
  this->error_msg+=error_msg;
  this->error_msg+=" - ";
  this->error_msg+=event_id;
}

CEventTimeoutException::CEventTimeoutException(const std::string& event_id):CException("",event_timeout_exception_msg)
{
  this->error_msg+=event_id;
}
