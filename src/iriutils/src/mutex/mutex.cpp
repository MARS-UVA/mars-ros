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

#include "mutex.h"
#include <cerrno>
#include <stdio.h>
#include "mutexexceptions.h"

CMutex::CMutex()
{
  int error=0;

  if((error=pthread_mutex_init(&this->access,NULL))!=0)
  {
    /* handle exception */
    throw CMutexException(_HERE_,"Impossible to create a new mutex object.\n");
  }
}

void CMutex::enter(void)
{
  int error=0;

  if ((error=pthread_mutex_lock(&this->access))!=0 )
  {
    /* handle exception */
    throw CMutexException(_HERE_,"Impossible to lock the mutex.\n");
  }
}

bool CMutex::try_enter(void)
{
  int error=0;

  if ((error=pthread_mutex_trylock(&this->access))!=0 )
  {
    if(error == EBUSY)
      return false;
    else
    {
      /* handle exception */
      throw CMutexException(_HERE_,"Impossible to try to enter the mutex.\n");
    }
  }

  return true;
}

void CMutex::exit(void)
{
  int error=0;

  if ((error=pthread_mutex_unlock(&this->access))!=0 )
  {
    /* handle exception */
    throw CMutexException(_HERE_,"Impossible to unlock the mutex.\n");
  }
}

CMutex::~CMutex()
{
  this->try_enter();
  this->exit();
  pthread_mutex_destroy(&this->access);
}
