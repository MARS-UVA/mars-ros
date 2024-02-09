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

#include "threadserver.h"
#include "threadexceptions.h"

CThreadServer *CThreadServer::pinstance=NULL;

CThreadServer::CThreadServer()
{
}

CThreadServer::CThreadServer(const CThreadServer& object)
{
}

CThreadServer& CThreadServer::operator = (const CThreadServer& object)
{
  return *this->pinstance;
}

std::list<CThread>::iterator CThreadServer::search_thread(const std::string& thread_id)
{
  std::string id;
  std::list<CThread>::iterator it;

  if(thread_id.size()==0)
  {
    /* handle exception */
    throw CThreadServerException(_HERE_,"Invalid thread id","NULL pointer");
  }
  else
  {
    for(it=this->thread_list.begin();it!=this->thread_list.end();it++)
    {
      id=it->get_id();
      if(thread_id==id)
      {
        return it;
      }
    }
    return (std::list<CThread>::iterator)NULL;
  }
  return (std::list<CThread>::iterator)NULL;
}

CThreadServer *CThreadServer::instance(void)
{
  if (CThreadServer::pinstance == NULL)  // ¿Es la primera llamada?
  {
    CThreadServer::pinstance = new CThreadServer(); // Creamos la instancia
  }
  return CThreadServer::pinstance; // Retornamos la dirección de la instancia
}

void CThreadServer::create_thread(const std::string& thread_id)
{
  CThread *new_thread;
  std::list<CThread>::iterator old_thread;

  this->access_threads.enter();
  if((old_thread=this->search_thread(thread_id))==(std::list<CThread>::iterator)NULL)
  {
    new_thread=new CThread(thread_id);
    this->thread_list.push_back(*new_thread);
    this->access_threads.exit();
  }
  else
  {
    this->access_threads.exit();
    /* handle exceptions */
    throw CThreadServerException(_HERE_,"Thread already exists",thread_id);
  }
}

void CThreadServer::delete_thread(const std::string& thread_id)
{
  std::list<CThread>::iterator old_thread;

  this->access_threads.enter();
  if((old_thread=this->search_thread(thread_id))==(std::list<CThread>::iterator)NULL)
  {
    this->access_threads.exit();
    /* handle exceptions */
    throw CThreadServerException(_HERE_,"Unknown thread",thread_id);
  }
  else
  {
    this->thread_list.erase(old_thread);
    this->access_threads.exit();
  }
}

void CThreadServer::attach_thread(const std::string& thread_id,void *(*user_thread_function)(void *param),void *param)
{
  std::list<CThread>::iterator thread;

  this->access_threads.enter();
  if((thread=this->search_thread(thread_id))==(std::list<CThread>::iterator)NULL)
  {
    this->access_threads.exit();
    /* handle exception */
    throw CThreadServerException(_HERE_,"Unknown thread",thread_id);
  }
  else
  {
    this->access_threads.exit();
    thread->attach(user_thread_function,param);
  }
}

void CThreadServer::start_thread(const std::string& thread_id)
{
  std::list<CThread>::iterator thread;

  this->access_threads.enter();
  if((thread=this->search_thread(thread_id))==(std::list<CThread>::iterator)NULL)
  {
    this->access_threads.exit();
    /* handle exceptions */
    throw CThreadServerException(_HERE_,"Unknown thread",thread_id);
  }
  else
  {
    this->access_threads.exit();
    thread->start();
  }
}

void CThreadServer::end_thread(const std::string& thread_id)
{
  std::list<CThread>::iterator thread;

  this->access_threads.enter();
  if((thread=this->search_thread(thread_id))==(std::list<CThread>::iterator)NULL)
  {
    this->access_threads.exit();
    /* handle exceptions */
    throw CThreadServerException(_HERE_,"Unknown thread",thread_id);
  }
  else
  {
    this->access_threads.exit();
    thread->end();
  }
}

void CThreadServer::kill_thread(const std::string& thread_id)
{
  std::list<CThread>::iterator thread;

  this->access_threads.enter();
  if((thread=this->search_thread(thread_id))==(std::list<CThread>::iterator)NULL)
  {
    this->access_threads.exit();
    /* handle exceptions */
    throw CThreadServerException(_HERE_,"Unknown thread",thread_id);
  }
  else
  { 
    this->access_threads.exit();
    thread->kill();
  }
}

void CThreadServer::detach_thread(const std::string& thread_id)
{
  std::list<CThread>::iterator thread;

  this->access_threads.enter();
  if((thread=this->search_thread(thread_id))==(std::list<CThread>::iterator)NULL)
  {
    this->access_threads.exit();
    /* handle exceptions */
    throw CThreadServerException(_HERE_,"Unknown thread",thread_id);
  }
  else
  {
    this->access_threads.exit();
    thread->detach();
  }
}

int CThreadServer::get_thread_state(const std::string &thread_id)
{
  std::list<CThread>::iterator thread;

  this->access_threads.enter();
  if((thread=this->search_thread(thread_id))==(std::list<CThread>::iterator)NULL)
  {
    this->access_threads.exit();
    /* handle exceptions */
    throw CThreadServerException(_HERE_,"Unknown thread",thread_id);
  }
  else
  {
    this->access_threads.exit();
    return thread->get_state();
  }
}
