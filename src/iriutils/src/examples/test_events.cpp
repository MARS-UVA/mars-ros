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

#include "eventexceptions.h"
#include "event.h"
#include <stdio.h>
#include <unistd.h>
#include <iostream>

/**
 * \example test_events.cpp
 *
 * This is an example on how to use events
 *
 * This example shows the basic operation with events and the most important
 * feature, which is the wait() function. This example mainly sets the event
 * several times and waits for it. Most of the times the wait function return
 * immediatelly after being called, and some other times, a timeout error is
 * generated.
 *
 * In this case the name of the event is just informative, but when the 
 * CEventServer class is used, this name is used as a unique identifier of
 * each event.
 *
 * The output of this example program should be something like this:
 *
 * \verbatim
 * Event identifier: test_event1
 * Number of instances of the event: 1
 * Number of instances of the event: 1
 * [Exception caught] - [CEvent class] - The maximum time to wait for the event activation has ellapsed - test_event1
 * Number of instances of the event: 3
 * Number of instances of the event: 2
 * Number of instances of the event: 1
 * Number of instances if the event: 0
 * \endverbatim
 *
 * Before the timeout exception, the program halts for about 1 second, which
 * is the desired timeout. Several error may be thrown by the CEvent class:
 *
 * * The maximum time to wait for an event ellapsed.
 */
int main(int argc, char *argv[])
{
  std::string event_id="";
  int num=0;
  CEvent Event("test_event1");

  event_id=Event.get_id();
  std::cout << "Event identifier: " << event_id << std::endl;
  // activate the event once
  Event.set();
  num=Event.get_num_activations();
  // the expected value is 1
  std::cout << "Number of instances of the event: " << num << std::endl;
  // wait for it to be signaled (this function returns immediatelly)
  Event.wait(1000000);
  // activate the event again
  Event.set();
  num=Event.get_num_activations();
  std::cout << "Number of instances of the event: " << num << std::endl;
  // reset the event
  Event.reset();
  // wait for the event to be signales. This function will get blocked until
  // the timeout expires.
  try {
    Event.wait(1000000);
  }catch(CEventTimeoutException& e){
    std::cout << e.what() << std::endl;
  }catch(CException& e){
    std::cout << e.what() << std::endl;
    return 0;
  }
  // activate the event several times
  Event.set();
  Event.set();
  Event.set();
  num=Event.get_num_activations();
  // the expected value is 3
  std::cout << "Number of instances of the event: " << num << std::endl;
  Event.reset();
  num=Event.get_num_activations();
  // the expected value is 2
  std::cout << "Number of instances of the event: " << num << std::endl;
  Event.wait(-1);
  num=Event.get_num_activations();
  // the expected value is 1
  std::cout << "Number of instances of the event: " << num << std::endl;
  if(Event.is_set())
  {
    Event.wait(-1);
  }
  num=Event.get_num_activations();
  // the expected value is 0
  std::cout << "Number of instances if the event: " << num << std::endl;
  if(Event.is_set())
  {
    Event.wait(-1);
  }

  return 0;
}
