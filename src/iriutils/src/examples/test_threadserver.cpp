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
#include <stdio.h>
#include <unistd.h>
#include <iostream>

/**
 * \brief Identifier of the first thread
 */
const std::string thread1="thread_1";
/**
 * \brief Identifier of the second thread
 */
const std::string thread2="thread_2";

/**
 * \brief Identifier of the invalid thread
 */
const std::string thread3="invalid_thread";

/**
 * \brief First thread function
 *
 * This function is executed by the first thread and loops continuously until
 * it is terminated by a call to the kill() function.
 */
void *my_thread_function1(void *param)
{
  int i=0;

  while(1)
  {
    std::cout << "thread 1 loop " << i << std::endl;
    i++;
    sleep(1);
  }  
  pthread_exit(NULL);
}

/**
 * \brief Second thread function
 *
 * This function is executed by the second thread and repeats the loop 15 times
 * before exiting. The end() function is used to wait for this thread to terminate.
 */
void *my_thread_function2(void *param)
{
  int i=0;

  for(i=0;i<15;i++)
  {
    std::cout << "thread 2 loop " << i << std::endl;
    sleep(1);
  }

  pthread_exit(NULL);
}

/**
 * \example test_threadserver.cpp
 *
 * This is an example on how to use the CThreadServer class
 *
 * In this example threads are handled through the CThreadServer class instead
 * of being directly handled by the user. The sequence of function calls is the
 * same as when dealing with CThread objects, but the main difference is that
 * all function calls are done through the CThreadServer class using the
 * unique identifier of each thread.
 *
 * The second thread starts over 2 seconds after the first one, and then the 
 * program waits for the second one to terminate before killing the first one.
 * Notice that the threads are created by calling the create_thread() function
 * with the desired unique identifier, and they must be destroyed by calling
 * the destroy_thread() function. Otherwise they are kept in memory until the
 * application ends.
 *
 * The putput of this example should be something like this:
 *
 * \verbatim
 * thread 1 loop 0
 * thread 1 loop 1
 * thread 2 loop 0
 * thread 1 loop 2
 * thread 2 loop 1
 * thread 1 loop 3
 * thread 2 loop 2
 * thread 1 loop 4
 * thread 1 loop 5
 * thread 2 loop 3
 * thread 1 loop 6
 * thread 2 loop 4
 * thread 1 loop 7
 * thread 2 loop 5
 * thread 1 loop 8
 * thread 2 loop 6
 * thread 1 loop 9
 * thread 2 loop 7
 * thread 1 loop 10
 * thread 2 loop 8
 * thread 1 loop 11
 * thread 2 loop 9
 * thread 1 loop 12
 * thread 2 loop 10
 * thread 1 loop 13
 * thread 2 loop 11
 * thread 1 loop 14
 * thread 2 loop 12
 * thread 1 loop 15
 * thread 2 loop 13
 * thread 1 loop 16
 * thread 2 loop 14
 * thread 1 loop 17
 * [Exception caught] - [CThread class] - The thread has not been attached to a function - thread_1
 * [Exception caught] - [CThreadServer class] - Unknown thread - invalid_thread
 * \endverbatim
 *
 * Several error may be thrown by the CThreadServer class:
 *
 * * The provided identifier does not exists
 *
 * * All errors of the CThread class
 */
int main(int argc, char *argv[])
{
  CThreadServer *thread_server;

  thread_server=CThreadServer::instance();

  thread_server->create_thread(thread1);
  thread_server->create_thread(thread2);

  thread_server->attach_thread(thread1,my_thread_function1,NULL);
  thread_server->attach_thread(thread2,my_thread_function2,NULL);

  thread_server->start_thread(thread1);
  sleep(2);
  thread_server->start_thread(thread2);
  thread_server->end_thread(thread2);
  thread_server->kill_thread(thread1);

  thread_server->detach_thread(thread1);
  try{
    thread_server->start_thread(thread1);
  }catch(CThreadException &e){
    std::cout << e.what() << std::endl;
  }
  
  thread_server->delete_thread(thread1);
  thread_server->delete_thread(thread2);

  try{
    thread_server->attach_thread(thread3,my_thread_function2,NULL);
  }catch(CThreadServerException &e){
    std::cout << e.what() << std::endl;
  }
}

