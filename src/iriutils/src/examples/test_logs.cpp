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

#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include "log.h"

/**
 * \example test_logs.cpp
 *
 * This is an example of the CLog class.
 *
 * This example shows the differents log options provided by the class
 * which include logging:
 *
 * * messages
 * * basic data types such as int, double, etc.
 * * vectors of basic data types.
 *
 */
int main(int argc,char *argv[])
{
  int int_test=5;
  double real_test=4.4;
  std::string string_test("string test");
  std::vector<int> int_vector_test;
  std::vector<float> real_vector_test;
  CLog main_log("log_test");

  // logging a string
  main_log.log(string_test);
  // logging an integer
  main_log.log(int_test);
  // logging a real value
  main_log.log(real_test);
  // logging an integer vector
  int_vector_test.push_back(1);
  int_vector_test.push_back(2);
  int_vector_test.push_back(3);
  int_vector_test.push_back(4);
  int_vector_test.push_back(5);
  main_log.log_vector(int_vector_test);
  // logging a real vector
  real_vector_test.push_back(1.1);
  real_vector_test.push_back(2.2);
  real_vector_test.push_back(3.3);
  real_vector_test.push_back(4.4);
  real_vector_test.push_back(5.5);
  real_vector_test.push_back(6.6);
  main_log.log_vector(real_vector_test);
}
