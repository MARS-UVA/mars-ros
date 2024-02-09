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

#include "event.h"
#include <string.h>
#include <unistd.h>
#include <exception>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/select.h> 
#include "eventexceptions.h"

CEvent::CEvent(const std::string& id)
{
  this->pipe_fd[0]=-1;
  this->pipe_fd[1]=-1;
  this->event_id=="";
  this->num_activations=0;
  if(pipe(this->pipe_fd)==-1)
  {
    /* handle exceptions */
    throw CEventException(_HERE_,"Impossible to create the internal pipe to generate the events",id);
  }
  this->set_id(id);
}

CEvent::CEvent(const std::string& id, bool create_signaled)
{
  this->pipe_fd[0]=-1;
  this->pipe_fd[1]=-1;
  this->event_id="";
  this->num_activations=0;
  if(pipe(this->pipe_fd)==-1)
  {
    /* handle exceptions */
    throw CEventException(_HERE_,"Impossible to create the internal pipe to generate the events",id);
  }
  this->set_id(id);
  if(create_signaled)
    this->set();
}

void CEvent::reset(void)
{
  char event_instance;

  this->access.enter();
  if(this->num_activations>0)
  {
    if(read(this->pipe_fd[0],&event_instance,1)==-1)
    {
      this->access.exit();
      /* handle exceptions */
      throw CEventException(_HERE_,"Impossible to read from the internal pipe",this->event_id);
    }
    this->num_activations--;
  }
  this->access.exit();
}

void CEvent::set(void)
{
  char event_instance=0x55;

  this->access.enter();
  if(write(this->pipe_fd[1],&event_instance,1)==-1)
  {
    this->access.exit();
    /* handle exceptions */
    throw CEventException(_HERE_,"Impossible to write to the internal pipe",this->event_id);
  }
  this->num_activations++;
  this->access.exit();
}

void CEvent::wait(int timeout_us)
{
  struct timeval timeout_t;
  fd_set wait_set;
  int error;

  this->access.enter();
  FD_ZERO(&wait_set);
  FD_SET(this->pipe_fd[0],&wait_set);
  if(timeout_us<0)
  { 
    this->access.exit();
    error=select(this->pipe_fd[0]+1,&wait_set,NULL,NULL,NULL);
  }
  else
  {
    timeout_t.tv_sec=0;
    timeout_t.tv_usec=timeout_us;
    this->access.exit();
    error=select(this->pipe_fd[0]+1,&wait_set,NULL,NULL,&timeout_t);
  }
  if(error==-1)
  {
    /* handle exceptions */
    throw CEventException(_HERE_,"Unexpected error while waiting the activation of the event",this->event_id);
  }
  else if(error==0)
  {
    /* handle exceptions */
    throw CEventTimeoutException(this->event_id);
  }
  else
  {
    this->reset();
  }

}

bool CEvent::is_set(void)
{
  this->access.enter();
  if(this->num_activations>0)
  {
    this->access.exit();
    return true;
  }
  else
  {
    this->access.exit();
    return false;
  }
  return false;
}

void CEvent::set_id(const std::string& id)
{
  if(id.size()==0)
  {
    /* handle exceptions */
    throw CEventException(_HERE_,"Invalid event id","empty string");
  }
  else
    this->event_id=id;
}

int CEvent::get_num_activations(void)
{
  return this->num_activations;
}

std::string CEvent::get_id(void)
{
  return this->event_id;
}

int CEvent::get_fd(void)
{
  return this->pipe_fd[0];
}

CEvent::~CEvent()
{
  this->access.enter();
  if(this->pipe_fd[0]!=-1)
    close(this->pipe_fd[0]);
  if(this->pipe_fd[1]!=-1)
    close(this->pipe_fd[1]);
  this->num_activations=0;
  this->access.exit();
}
