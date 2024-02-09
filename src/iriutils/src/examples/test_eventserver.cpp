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
#include "eventexceptions.h"
#include <iostream>

/**
 * \brief Identifier of the first event
 */
const std::string event1="event_1";

/**
 * \brief Identifier of the second event
 */
const std::string event2="event_2";

/**
 * \brief Identifier of the second event
 */
const std::string event3="invalid_event";

/**
 * \example test_eventserver.cpp
 *
 * This is an example on how to use the CEventServer class
 *
 * This example shows how to handle multiple events using the CEventServer
 * class. The possible actions on events are the same as the one available
 * with the CEvent class, but in this case all the operations are done 
 * through the event server, using the unique identifier provided at 
 * creation time.
 *
 * The events are first created with a unique name using the create_event()
 * function and must be destroyed with the delete_event() function. Otherwise
 * the event are kept in memory until the application ends.
 *
 * The server provides a couple of functions (wait_first() and wait_all()) to
 * wait in any combination of events, build as a stl list object. In these 
 * functions a timeout exception is also present.
 *
 * The output of this example should be something like this:
 *
 * \verbatim
 * Activate event 1
 * Activate event 2
 * Reset event 1
 * Wait for the first event to get active ... 1
 * event 1 is not set
 * event 2 is not set
 * Activate event 1
 * Activate event 2
 * All events were active
 * event 1 is not set
 * event 2 is not set
 * [Exception caught] - [CEventServer class] - The maximum time to wait for the event activation has ellapsed -
 * [Exception caught] - [CEventServer class] - Unknown event - invalid_event
 * \endverbatim
 *
 * Before the first exception message, the program will halt for about 1 second.
 * Several errors may be thrown by the CeventServer class:
 *
 * * The provided identifier does not exist
 *
 * * The maximum time to wait has ellapsed
 *
 * * All errors of the CEvent class
 *
 */
int main(int argc, char *argv[])
{
  CEventServer *event_server;
  std::list<std::string> events;
  int event_id;

  event_server=CEventServer::instance();
  event_server->create_event(event1);
  event_server->create_event(event2);

  events.push_back(event1);
  events.push_back(event2);
 
  std::cout << "Activate event 1" << std::endl;
  event_server->set_event(event1);
  std::cout << "Activate event 2" << std::endl;
  event_server->set_event(event2);
  if (event_server->event_is_set(event1))
  {
    std::cout << "Reset event 1" << std::endl;
    event_server->reset_event(event1);
  }
  std::cout << "Wait for the first event to get active ... " << std::endl;
  event_id=event_server->wait_first(events);
  std::cout << event_id << std::endl;
  if (event_server->event_is_set(event1))
    std::cout << "event 1 is set" << std::endl;
  else
    std::cout << "event 1 is not set" << std::endl;
  if (event_server->event_is_set(event2))
    std::cout << "event 2 is set" << std::endl;
  else
    std::cout << "event 2 is not set" << std::endl;
  std::cout << "Activate event 1" << std::endl;
  event_server->set_event(event1);
  std::cout << "Activate event 2" << std::endl;
  event_server->set_event(event2);
  event_server->wait_all(events);
  std::cout << "All events were active" << std::endl;
  if (event_server->event_is_set(event1))
    std::cout << "event 1 is set" << std::endl;
  else
    std::cout << "event 1 is not set" << std::endl;
  if (event_server->event_is_set(event2))
    std::cout << "event 2 is set" << std::endl;
  else
    std::cout << "event 2 is not set" << std::endl;
  try{
    event_server->wait_all(events,1000);
  }catch(CEventTimeoutException &e){
    std::cout << e.what() << std::endl;
  }catch(CException& e){
    std::cout << e.what() << std::endl;
    return 0;
  }
  event_server->delete_event(event1);
  event_server->delete_event(event2);
  try{
    event_server->delete_event(event3);
  }catch(CEventServerException &e){
    std::cout << e.what() << std::endl;
  } 
  delete event_server;
}
