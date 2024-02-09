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

#include "exceptions.h"
#include <string.h>
#include <stdio.h>

const std::string exception_msg="[Exception caught] - ";

CException::CException(const std::string& where, const std::string& error_msg)
{
  this->error_msg=exception_msg;
  this->error_msg+=where;
  this->error_msg+="\nError: ";
  this->error_msg+=error_msg;
}

const std::string& CException::what(void)
{
  return this->error_msg;
}

CException::~CException() throw()
{
  /* the error message is destroyed by its own destructor */
}
