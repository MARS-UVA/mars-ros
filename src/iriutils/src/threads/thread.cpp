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
#include "thread.h"
#include <exception>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <netdb.h>

CThread::CThread(const std::string& id)
{
  this->user_thread_function=NULL;
  this->state=detached;
  this->set_id(id);
}

void CThread::attach(void *(*user_thread_function)(void *param),void *param)
{
  this->access.enter();
  if(user_thread_function==NULL)
  {
    this->access.exit();
    /* handle exceptions */
    throw CThreadException(_HERE_,"Invalid thread function",this->thread_id);
  }
  else
  {
    this->user_thread_function=user_thread_function;
    this->param=param;
    this->state=attached;
    this->access.exit();
  }
}

void CThread::start(void)
{
  int error;

  this->access.enter();
  if(this->user_thread_function==NULL)
  {
    this->access.exit();
    /* handle exceptions */
    throw CThreadException(_HERE_,"The thread has not been attached to a function",this->thread_id);
  }
  else
  {
    if((error=pthread_create(&this->thread,NULL,this->thread_function,this))!=0)
    {
      this->access.exit();
      /* handle exceptions */
      throw CThreadException(_HERE_,"Impossible to start the thread",this->thread_id);
    }
    else
    {
      this->state=starting;
      this->access.exit();
    }
  }
}

void CThread::end(void)
{
  int error;

  this->access.enter();
  if(this->state==active || this->state==starting)
  {
    this->access.exit();
    if((error=pthread_join(this->thread,NULL))!=0)
    {
      /* handle exceptions */
      throw CThreadException(_HERE_,"Error while waiting the thread to end",this->thread_id);
    }
    this->access.enter();
    this->state=attached;
  }
  this->access.exit();
  /* otherwise do nothing */
}

void CThread::kill(void)
{
  int error;

  this->access.enter();
  if(this->state==active || this->state==starting)
  {
    this->access.exit();
    if((error=pthread_cancel(this->thread))!=0)
    {
      /* handle the exceptions */
      if(error!=ESRCH)
        throw CThreadException(_HERE_,"Error while cancelling the thread",this->thread_id);
      //otherwise, the thread has already finished
    }
    if((error=pthread_join(this->thread,NULL))!=0)
    {
      /* handle exceptions */
      throw CThreadException(_HERE_,"Error while waiting the thread to end",this->thread_id);
    }
    this->access.enter();
    this->state=attached;
  }
  this->access.exit();
}

void CThread::detach(void)
{
  this->access.enter();
  if(this->state==attached)
  {
    this->user_thread_function=NULL;
    this->param=NULL;
    this->state=detached;
  }
  this->access.exit();
  /* otherwise do nothing */
}

int CThread::get_state(void)
{
  int tmp_state;

  this->access.enter();
  tmp_state=this->state;
  this->access.exit();

  return tmp_state;
}

void CThread::set_id(const std::string& id)
{
  if(id.size()==0)
  {
    /* handle exception */
    throw CThreadException(_HERE_,"Invalid thread id","empty string");
  }
  else
    this->thread_id=id;
}

std::string CThread::get_id(void)
{
  return this->thread_id;
}

void *CThread::thread_function(void *param)
{
  CThread *thread=(CThread *)param;

  thread->access.enter();
  thread->state=active;
  thread->access.exit();
  thread->user_thread_function(thread->param);
  pthread_exit(NULL);
}

CThread::~CThread()
{
  this->kill();
  this->detach();
}


