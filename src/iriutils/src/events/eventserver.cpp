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

#include <math.h>
#include <sys/select.h>
#include "ctime.h"
#include "eventserver.h"
#include "eventexceptions.h"
#include <errno.h>

CEventServer *CEventServer::pinstance=NULL;

CEventServer::CEventServer()
{
}

CEventServer::CEventServer(const CEventServer& object)
{
}

CEventServer& CEventServer::operator = (const CEventServer& object)
{
  return *this->pinstance;
}

std::list<CEvent>::iterator CEventServer::search_event(const std::string& event_id)
{
  std::string id;
  std::list<CEvent>::iterator it;

  if(event_id.size()==0)
  {
    /* handle exceptions */
    throw CEventServerException(_HERE_,"Invalid event id","empty string");
  }
  else
  {
    for(it=this->event_list.begin();it!=this->event_list.end();it++)
    {
      id=it->get_id();
      if(event_id==id)
        return it;
    }
    return (std::list<CEvent>::iterator)NULL;
  }
  return (std::list<CEvent>::iterator)NULL;
}

CEventServer *CEventServer::instance(void)
{
  if (CEventServer::pinstance == NULL)
  {
    CEventServer::pinstance = new CEventServer(); // Creamos la instancia
  }
  return CEventServer::pinstance; // Retornamos la dirección de la instancia
}

void CEventServer::create_event(const std::string& event_id)
{
  CEvent *new_event;
  std::list<CEvent>::iterator old_event;

  this->access_events.enter();
  if((old_event=this->search_event(event_id))==(std::list<CEvent>::iterator)NULL)
  {
    new_event=new CEvent(event_id);
    this->event_list.push_back(*new_event);
    this->access_events.exit();
  }
  else
  {
    this->access_events.exit();
    /* handle exceptions */
    throw CEventServerException(_HERE_,"Event already exists!",event_id);
  }
}

void CEventServer::delete_event(const std::string& event_id)
{
  std::list<CEvent>::iterator old_event;

  this->access_events.enter();
  if((old_event=this->search_event(event_id))==(std::list<CEvent>::iterator)NULL)
  {
    this->access_events.exit();
    /* handle exceptions */
    throw CEventServerException(_HERE_,"Unknown event",event_id);
  }
  else
  {
    this->event_list.erase(old_event);
    this->access_events.exit();
  }
}

void CEventServer::set_event(const std::string& event_id)
{
  std::list<CEvent>::iterator event;

  this->access_events.enter();
  if((event=this->search_event(event_id))==(std::list<CEvent>::iterator)NULL)
  {
    this->access_events.exit();
    /* handle exceptions */
    throw CEventServerException(_HERE_,"Unknown event",event_id);
  }
  else
  {
    this->access_events.exit();
    event->set();
  }
}

void CEventServer::reset_event(const std::string& event_id)
{
  std::list<CEvent>::iterator event;

  this->access_events.enter();
  if((event=this->search_event(event_id))==(std::list<CEvent>::iterator)NULL)
  {
    this->access_events.exit();
    /* handle exceptions */
    throw CEventServerException(_HERE_,"Unknown event",event_id);
  }
  else
  {
    this->access_events.exit();
    event->reset();
  }
}

bool CEventServer::event_is_set(const std::string& event_id)
{
  std::list<CEvent>::iterator event;

  this->access_events.enter();
  if((event=this->search_event(event_id))==(std::list<CEvent>::iterator)NULL)
  {
    this->access_events.exit();
    /* handle exceptions */
    throw CEventServerException(_HERE_,"Unknown event",event_id);
  }
  else
  {
    this->access_events.exit();
    return event->is_set();
  }
}

int CEventServer::get_num_events(void)
{
  size_t num;

  this->access_events.enter();
  num=this->event_list.size();
  this->access_events.exit();

  return (int)num;
}

int CEventServer::get_num_activations(const std::string& event_id)
{
  std::list<CEvent>::iterator event;

  this->access_events.enter();
  if((event=this->search_event(event_id))==(std::list<CEvent>::iterator)NULL)
  {
    this->access_events.exit();
    /* handle exceptions */
    throw CEventServerException(_HERE_,"Unknown event",event_id);
  }
  else
  {
    this->access_events.exit();
    return event->get_num_activations();
  }
}

int CEventServer::wait_first(std::list<std::string> events,int timeout)
{
  std::list<std::string>::iterator it;
  std::list<CEvent>::iterator event;
  std::string event_list="";
  int error,max_fd=0,pos=-1;
  fd_set wait_set;
  CTime time_out;
  bool end=false;
  timeval time;

  if(timeout>=0)
  {
    time_out.set(timeout);
    time=time_out.getTimeInTimeval();
  }
  this->access_events.enter();
  while(!end)
  {
    FD_ZERO(&wait_set);
    for(it=events.begin();it!=events.end();it++)
    {
      if ((event=this->search_event(*it)) == (std::list<CEvent>::iterator)NULL)
      {
        this->access_events.exit();
        /* handle exceptions */
        throw CEventServerException(_HERE_,"Unknown event",*it);
      }
      else
      {
        FD_SET(event->get_fd(),&wait_set);
        if(event->get_fd()>max_fd)
          max_fd=event->get_fd();
        if(it!=events.begin())
          event_list+=",";
        event_list+=*it;
      }
    }
  
    this->access_events.exit();
    if(timeout>=0)
      error=select(max_fd+1,&wait_set,NULL,NULL,&time);
    else
      error=select(max_fd+1,&wait_set,NULL,NULL,NULL);
    if(error==-1)
    {
      if(errno!=EINTR)
      {
        /* handle exceptions */
        throw CEventServerException(_HERE_,"Unexpected error while waiting the activation of one the events",event_list);
      }
    }
    else if(error==0)
    {
      /* handle exceptions */
      throw CEventTimeoutException(event_list);
    }
    else
    {
      this->access_events.enter();
      for(it=events.begin();it!=events.end();it++)
      {
        pos++;
        if((event=this->search_event(*it))==(std::list<CEvent>::iterator)NULL)
        {
          this->access_events.exit();
          /* handle exceptions */
          throw CEventServerException(_HERE_,"Unknown event",*it);
        }
        else
        {
          if(FD_ISSET(event->get_fd(),&wait_set))
          {
            if(event->is_set())
            {
              event->reset();
              this->access_events.exit();
              return pos;
            }
          }
        }
      }
    }
  }
  this->access_events.exit();

  return -1;
}

void CEventServer::wait_all(std::list<std::string> events,int timeout)
{
  std::list<std::string>::iterator it;
  std::list<CEvent>::iterator event;
  std::string event_list;
  int error,max_fd=0;
  fd_set wait_set;
  bool end=false;
  CTime time_out;
  timeval time;

  if(timeout>=0)
  {
    time_out.set(timeout);
    time=time_out.getTimeInTimeval();
  }
  this->access_events.enter();
  while(!end)
  {
    event_list="";
    FD_ZERO(&wait_set);
    for(it=events.begin();it!=events.end();it++)
    {
      if((event=this->search_event(*it))==(std::list<CEvent>::iterator)NULL)
      {
        this->access_events.exit();
        /* handle exceptions */
        throw CEventServerException(_HERE_,"Unknown event",*it);
      }
      else
      {
        FD_SET(event->get_fd(),&wait_set);
        if(event->get_fd()>max_fd)
          max_fd=event->get_fd();
        if(it!=events.begin())
          event_list+=",";
        event_list+=*it;
      }  
    }
    this->access_events.exit();
    if(timeout>=0)
      error=select(max_fd+1,&wait_set,NULL,NULL,&time);
    else
      error=select(max_fd+1,&wait_set,NULL,NULL,NULL);
    if(error==-1)
    {
      if(errno!=EINTR)
      {
        /* handle exceptions */
        throw CEventServerException(_HERE_,"Unexpected error while waiting the activation of one the events",event_list);
      }
    }
    else if(error==0)
    {
      /* handle exceptions */
      throw CEventTimeoutException(event_list);
    }
    else
    {
      this->access_events.enter();
      for(it=events.begin();it!=events.end();it++)
      {
        if((event=this->search_event(*it))==(std::list<CEvent>::iterator)NULL)
        {
	  this->access_events.exit();
          /* handle exceptions */
          throw CEventServerException(_HERE_,"Unknown event",*it);
        }
        else
        {
          if(FD_ISSET(event->get_fd(),&wait_set))
	  {
            if(event->is_set())
            {
              event->reset();
              it=events.erase(it);
              if(events.size()==0)
                end=true;
              else
              {
                if(timeout>=0)
                  time=time_out.getTimeInTimeval();
              }
            }
      	  }
        }
      }
    }
  }
  this->access_events.exit();
}
