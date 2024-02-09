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

#include "eventserver.h"
#include "threadserver.h"
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include "log.h"

/**
 * \brief Identifier of the first thread
 */
const std::string thread1_id="thread1";
/**
 * \brief Identifier of the second thread
 */
const std::string thread2_id="thread2";

/**
 * \brief Identifier of the first event
 */
const std::string event1_id="event1";
/**
 * \brief Identifier of the second event
 */
const std::string event2_id="event2";
/**
 * \brief Identifier of the thirth event
 */
const std::string event3_id="event3";

/**
 * \brief Function executed by the first thread
 *
 * This function waits for the first event to get active (the second thread
 * should activate it), and then activates the second and thirth events with
 * some delay between the activations (this activations should awaken the
 * second thread). After that the thread ends its execution
 */
void *thread1_function(void *param)
{
  std::list<std::string> event_list1;
  CEventServer *event_server=CEventServer::instance();
  CLog thread_log("thread1");

  event_list1.push_back(event1_id);
  std::cout << "Thread1 - Waiting for event " << event1_id << std::endl;
  thread_log.log((const std::string)"Thread1 - Waiting for event" + event1_id);
  event_server->wait_first(event_list1);
  std::cout << "Thread1 - Event " << event1_id << " received" << std::endl;
//  thread_log.log((const std::string)"Thread1 - Event " + event1_id + " received");
//  std::cout << "Thread1 - Sending event " << event2_id << std::endl;
//  thread_log.log((const std::string)"Thread1 - Sending event" + event2_id);
//  event_server->set_event(event2_id);
//  sleep(2);
//  std::cout << "Thread1 - Sending event " << event3_id << std::endl;
//  thread_log.log((const std::string)"Thread1 - Sending event" + event3_id);
//  event_server->set_event(event3_id);
//  std::cout << "Thread1 - Ending" << std::endl;
//  thread_log.log((const std::string)"Thread1 - Ending");

  pthread_exit(NULL);
}

/**
 * \brief Function executed by the second thread
 *
 * This function first activate the first event to awaken the first thread, 
 * and then waits for the second and thirth events to get active (these 
 * events should be activated by the first thread). After that, the thread
 * ends its execution.
 */
void *thread2_function(void *param)
{
  std::list<std::string> event_list2;
  CEventServer *event_server=CEventServer::instance();
  CLog thread_log("thread2");

  event_list2.push_back(event1_id);
  std::cout << "Thread2 - Waiting for event " << event1_id << std::endl;
  thread_log.log((const std::string)"Thread2 - Waiting for event" + event1_id);
  event_server->wait_first(event_list2);
  std::cout << "Thread2 - Event " << event1_id << " received" << std::endl;
//  event_list2.push_back(event2_id);
//  event_list2.push_back(event3_id);
//  std::cout << "Thread2 - Sending event " << event1_id << std::endl;
//  thread_log.log((const std::string)"Thread2 - Sending event " + event1_id);
//  event_server->set_event(event1_id);
//  std::cout << "Thread2 - Waiting for event " << event2_id << std::endl;
//  thread_log.log((const std::string)"Thread2 - Waiting for event " + event2_id);
//  event_server->wait_all(event_list2);
//  std::cout << "Thread2 - Event " << event2_id << " and " << event3_id << " received" << std::endl;
//  thread_log.log((const std::string)"Thread2 - Event " + event2_id + " and " + event3_id + " received");
//  std::cout << "Thread2 - Ending" << std::endl;
//  thread_log.log((const std::string)"Thread2 - Ending");

  pthread_exit(NULL);
}

/**
 * \example test_bothservers.cpp
 *
 * This is an example of the CEventServer and the CThreadServer classes
 * working together.
 *
 * This example shows how the both servers can work together to synchronize
 * several threads of execution. This example create two threads that 
 * communicate to each other using events in order to synchronize their 
 * execution.
 *
 * The main program creates all the events and threads, and then start the 
 * execution of the threads, and waits for their termination. The output of
 * this example should be somthing like this:
 *
 * \verbatim
 * Thread1 - Waiting for event event1
 * Thread2 - Sending event event1
 * Thread2 - Waiting for event event2
 * Thread1 - Event event1 received
 * Thread1 - Sending event event2
 * Thread1 - Sending event event3
 * Thread1 - Ending
 * Thread2 - Event event2 and event3 received
 * Thread2 - Ending
 * End of the program
 * \endverbatim
 *
 * After the first message, and before the reception of the activation of the
 * second and thirth events, the program halts for a few seconds in the wait
 * functions.
 *
 */
int main(int argc,char *argv[])
{
  CEventServer *event_server=CEventServer::instance();
  CThreadServer *thread_server=CThreadServer::instance();
  CLog main_log("main");

  //thread test
  main_log.log("Creating events ...");
  event_server->create_event(event1_id);
  event_server->create_event(event2_id);
  event_server->create_event(event3_id);

  main_log.log((const std::string)"Creating threads ...");
  thread_server->create_thread(thread1_id);
  thread_server->create_thread(thread2_id);
  thread_server->attach_thread(thread1_id,thread1_function,NULL);
  thread_server->attach_thread(thread2_id,thread2_function,NULL);
  main_log.log((const std::string)"Starting thread 1 ...");
  thread_server->start_thread(thread1_id);
  sleep(3);
  main_log.log((const std::string)"Starting thread 2 ...");
  thread_server->start_thread(thread2_id);
  sleep(3);
  event_server->set_event(event1_id);
  sleep(2);
  event_server->set_event(event1_id);
  sleep(2);
  thread_server->end_thread(thread2_id);
  thread_server->end_thread(thread1_id);
  std::cout << "End of the program" << std::endl;
  main_log.log((const std::string)"End of the program");
}
