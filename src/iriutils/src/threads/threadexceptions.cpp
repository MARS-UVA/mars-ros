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

#include "threadexceptions.h"
#include <string.h>
#include <stdio.h>

const std::string thread_exception_msg="[CThread class] - ";
const std::string thread_server_exception_msg="[CThreadServer class] - ";

CThreadException::CThreadException(const std::string& where,const std::string& error_msg,const std::string& thread_id):CException(where,thread_exception_msg)
{
  this->error_msg+=error_msg;
  this->error_msg+=" - ";
  this->error_msg+=thread_id;
}

CThreadServerException::CThreadServerException(const std::string& where,const std::string& error_msg,const std::string& thread_id):CException(where,thread_server_exception_msg)
{
  this->error_msg+=error_msg;
  this->error_msg+=" - ";
  this->error_msg+=thread_id;
}

