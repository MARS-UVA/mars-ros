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

#include "thread.h"
#include "threadexceptions.h"
#include <stdio.h>
#include <unistd.h>
#include <iostream>

/// simple thread function
/** The execution of this function is finite, since when it is finished, it 
 * returns to the calling function and therefore finishes the thread. This
 * function is used to test the end() function which waits until the thread
 * end. 
 *
 * In this case no parameter is passed to the function, so param is not used.
 */
void *my_thread_function(void *param)
{
  int i=0;

  for(i=0;i<=2;i++)
  {
    std::cout << "loop number: " << i << std::endl;
    sleep(1);
  }
  pthread_exit(NULL);
}

/// Infinite loop thread function
/** This function continuously print a message on screen. Its execution never
 * ends, and a call to the end() function from the main thread will get blocked
 * forever.
 *
 * In this case, it is necessary to call the kill() function in order to cancel 
 * the execution of the thread before waiting for it to finish execution. In this
 * case, no parameter is used.
 */
void *my_thread_function2(void *param)
{
  int i=0;

  while(1)
  {
    std::cout << "another loop number: " << i << std::endl;
    i++;
    sleep(1);
  }
  pthread_exit(NULL);
}

/// Function with parameters
/** This funciton executes a finite loop like the first function, but in this case
 * the number of iterations is passed to the thead as a parameter. The end() 
 * function can be used in this case to wait for the thread to end execution.
 *
 * The parameter passed to the thread is a pointer, so the referenced variable must
 * be always valid when the thread is started.
 */
void *my_thread_function3(void *param)
{
  int i=0;
  int num_iter=*((int *)param);

  for(i=0;i<=num_iter;i++)
  {
    std::cout << "user defined loop number: " << i << std::endl;
    sleep(1);
  }

  pthread_exit(NULL);
}

/**
 * \example test_threads.cpp
 *
 * This is an example on how to use threads
 *
 * This example shows how to use the CThread class. In this example three 
 * different thread functions are attached to a single thread:
 *
 * * The first function executes a loop a finite number of times and then
 *   exits. So the end() function can be used to wait for the thread to end.
 *
 * * The second function executes an infinite loop that is only interrupted
 *   by the call to the kill function which immediately terminates the thead.
 *
 * * The last function repeats the loop as many times as indicated by the
 *   thread function parameter. The end function can be used to wait for the
 *   thread to end.
 *
 * Notice that the sequence of functions called is always the same: first a 
 * function is attached (attach()) and then the thread is started by calling
 * the start() function. If the thread ends by its own, the end() function can
 * be used, otherwise, it is necessary to use the kill() function to terminate
 * the thread.
 *
 * After terminating the execution of the thread, a new function can be attached
 * or the previous one restarted. Notice that it is not required to use the 
 * detach() function to remove the previous function before attaching a new one.
 *
 * In this case the name provided to the thread is just informative, but when 
 * the CThreadServer class is used to handle multiple threads, this name is 
 * used to identify thre desired thread, so each thread must have a different
 * name.
 *
 * The output of this example should be something like this:
 *
 * \verbatim
 * Thread identifier: test_thread
 * Thread state: 3
 * Thread state: 0
 * Thread state: 1
 * loop number: 0
 * loop number: 1
 * loop number: 2
 * Thread state: 0
 * Thread state: 3
 * Thread state: 0
 * Thread state: 1
 * another loop number: 0
 * another loop number: 1
 * another loop number: 2
 * Thread state: 0
 * Thread state: 0
 * Thread state: 1
 * user defined loop number: 0
 * user defined loop number: 1
 * user defined loop number: 2
 * user defined loop number: 3
 * Thread state: 0
 * Thread state: 3
 * [Exception caught] - [CThread class] - Invalid thread function - test_thread
 * [Exception caught] - [CThread class] - The thread has not been attached to a function - test_thread
 * [Exception caught] - [CThread class] - Invalid thread id - NULL pointer
 * \endverbatim
 *
 * Several error may be thrown by the CThread class:
 *
 * * An invalid thread function is given to the attach() function.
 *
 * * A thread is started without any valid thread function.
 *
 * * A thread is given an invalid identifier at construction time.
 */
int main(int argc, char *argv[])
{
  int num_iter=3;
  std::string thread_id;
  CThread Thread("test_thread"),*Thread2;

  // get the thread identifier
  thread_id=Thread.get_id();
  std::cout << "Thread identifier: " << thread_id << std::endl;
  // test with the first function
  // the expected state is detachhed (3)
  std::cout << "Thread state: " << Thread.get_state() << std::endl;
  Thread.attach(my_thread_function,NULL);
  // the expected state is attached (0)
  std::cout << "Thread state: " << Thread.get_state() << std::endl;
  Thread.start();
  // the expected state is active (2) or starting (1)
  std::cout << "Thread state: " << Thread.get_state() << std::endl;
  // wait until the thread executes all the iterations
  Thread.end();
  // the expected state is attached (0) 
  std::cout << "Thread state: " << Thread.get_state() << std::endl;
  Thread.detach();
  // the expected state is detached (3)
  std::cout << "Thread state: " << Thread.get_state() << std::endl;

  // test with the second function
  Thread.attach(my_thread_function2,NULL);
  // the expected state is attached (0)
  std::cout << "Thread state: " << Thread.get_state() << std::endl;
  Thread.start();
  // the expected state is active (2) or starting (1)
  std::cout << "Thread state: " << Thread.get_state() << std::endl;
  // wait for the thread to execute some loops (15 aprox)
  sleep(3);
  // cancel the thread immediatelly
  Thread.kill();
  // the expected state is attached (0)
  std::cout << "Thread state: " << Thread.get_state() << std::endl;

  // test with the thirth function
  Thread.attach(my_thread_function3,&num_iter);
  // the expected state is attached (0)
  std::cout << "Thread state: " << Thread.get_state() << std::endl;
  Thread.start();
  // the expected state is active (2) or starting (1)
  std::cout << "Thread state: " << Thread.get_state() << std::endl;
  // wait until the trhead executes num_iter iterations
  Thread.end();
  // the expected state is attached (0)
  std::cout << "Thread state: " << Thread.get_state() << std::endl;
  Thread.detach();
  // the expected state is detached (3)
  std::cout << "Thread state: " << Thread.get_state() << std::endl;

  // test thread exceptions
  try{
    Thread.attach(NULL,NULL);
  }catch(CThreadException &e){
    std::cout << e.what() << std::endl;
  }
  try{
    Thread.start();
  }catch(CThreadException &e){
    std::cout << e.what() << std::endl;
  }
  try{
    Thread2=new CThread("");
    delete Thread2;
  }catch(CThreadException &e){
    std::cout << e.what() << std::endl;
  }
}
