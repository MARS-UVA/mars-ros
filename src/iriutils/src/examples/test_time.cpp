// Copyright (C) 2009-2010 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
// Author Martí Morta  (mmorta@iri.upc.edu)
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

/*! \example test_time.cpp
    \brief This test checks all functions in CTime class.

    This example tests time creation and set, time operations (-,+,==,/), get
    time in different formats and throws an exception.

    This example output will be something similar to this:
    \verbatim

    TIME MANAGER EXAMPLE
    Example: create times (t0,t1,t2)

    t0: 1314886453.799155

    Example: sleep 300 ms

    Example: set t1 (0.300 rel)

    t1: 1314886454.099320
    t1o: 0.300164

    Example: set t3 delayed 200 ms (0.200 abs)

    t3: 0.200000

    Example: sleep 1705 ms

    Example: set t2 (2.005 rel)

    t2: 1314886455.804489
    t2o: 2.005333

    Example: t2 - t1 (1.705)

    td: 1.705169
    tdo: 1.705169

    Example: + addition (t1+t2)

    ts: 2629772909.903809
    tso: 2.305498
    ulongmax: 4294967295

    Example: + Exception (overvalue)

    [Exception caught] - CTime CTime::operator+(CTime&) at /home/mmorta/codi/drivers/iriutils/trunk/src/time/ctime.cpp:195
    Error: [CTime class] - Result higher than ulong max

    Example: average (t1,t2)

    ta: 1314886454.951905
    tao: 1.152749

    Example: comparative (t1==t2, t3==t2)

    ?: false true

    Example: comparative (t1<t2, t1<=t2, t3>t2, t3>=t2)

    t1: 1314886454.099320
    t2: 1314886455.804489
    t3: 1314886455.804489
    ?: <: true <=: true
    ?: >: false >=: true

    Example: getting time (t1/t0)

    seconds and nanoseconds:
    1314886454  99320101
    0  300164204
    double Seconds:
    1.31489e+09
    0
    double milliseconds:
    6.26462e+08
    300
    Format ctf_secnano:
    1314886454 099320101
    0 300164204
    Format ctf_datetime:
    2011-09-01,16:14:14
    1970-01-01,01:00:00
    Format ctf_ms:
    1314886454.099
    0.300
    Format ctf_us:
    1314886454.099320
    0.300164
    Timespec
    1314886454 99320101
    Time_t
    1314886454
    some formats using time_t
    2011-09-01,16:14:14
    04:14PM
    Thu Sep  1 16:14:14 2011

    Example: Using Reference Time

    Set Reference and wait 1 second

    Time ref: 1314886455.805064
    t1:       1.000107
    t2:       1314886455.804489

    Set useRef(true) for t2 and set its time

    t2:       1.000163  Is ref used?: true


    Example: END

    \endverbatim

    In this example namespace std is used because cout and endl are used
    a lot.

    The example code is the following:

*/


#include "ctime.h"
#include <iostream>
#include <unistd.h>

using namespace std;

int main(void)
{
  cout << "\nTIME MANAGER EXAMPLE" << endl;

  cout << "  Example: create times (t0,t1,t2)\n\n";
    CTime t0,t1,t2,t3,t20,td,ta,td0,ta0,ts,ts0,ta2,ta20;
    cout << "    t0: " << t0 << endl;

  cout << "\n  Example: sleep 300 ms " << endl;
    usleep(300000);

  cout << "\n  Example: set t1 (0.300 rel)\n\n";
    t1.set();
    CTime t10 = t1 - t0;
    cout << "    t1: " << t1 << "\n    t1o: " << t10 << endl;

  cout << "\n  Example: set t3 delayed 200 ms (0.200 abs)\n\n";
    t3.set(200);
    cout << "    t3: " << t3 << endl;

  cout << "\n  Example: sleep 1705 ms " << endl;
    usleep(1705000);

  cout << "\n  Example: set t2 (2.005 rel)\n\n";
    t2.set();
    t20 = t2 - t0;
    cout << "    t2: " << t2 << "\n    t2o: " << t20 << endl;

  cout << "\n  Example: t2 - t1 (1.705)\n\n";
    td = t2 - t1;
    td0 = t20 - t10;
    cout << "    td: " << td << "\n    tdo: " << td0 << endl;

  cout << "\n  Example: + addition (t1+t2)\n\n";
    ts = t1 + t2;
    ts0 = t10 + t20;
    cout << "    ts: " << ts << "\n    tso: " << ts0 << "\n    ulongmax: " << ULONG_MAX <<  endl;

  cout << "\n  Example: + Exception (overvalue)\n\n";
    try{
      ts = ts + ts;
    }catch(CException &e)
    {
      cout << e.what() << endl;
    }
  cout << "\n  Example: - Exception (negative)\n\n";
    try{
      ts = t1 - t2;
    }catch(CException &e)
    {
      cout << e.what() << endl;
    }

  cout << "\n  Example: average (t1,t2)\n\n";
    ta = ( t1 + t2 ) / 2;
    ta0 = (t10 + t20)/2 ;
    cout << "    ta: " << ta << "\n    tao: " << ta0 << endl;

  cout << "\n  Example: comparative (t1==t2, t3==t2)\n\n";
    t3 = t2;
    cout << "    ?: " <<  boolalpha << (bool)(t1==t2) << " "
                                    << (bool)(t3==t2)  << endl;

  cout << "\n  Example: comparative (t1<t2, t1<=t2, t3>t2, t3>=t2)\n\n";
  t3 = t2;
  cout
  << "    t1: " << t1 << "\n    t2: " << t2 << endl << "    t3: " << t3 << endl
  << "    ?: <: " <<  boolalpha << (bool)(t1<t2) << " <=: " << (bool)(t1<=t2) << endl
  << "    ?: >: " <<  boolalpha << (bool)(t3>t2) << " >=: " << (bool)(t3>=t2) << endl;


  cout << "\n  Example: getting time (t1/t0)\n\n";

    cout << "    seconds and nanoseconds:"                   << endl
      << "     " << t1.seconds() << "  " << t1.nanoseconds() << endl
      << "     " << t10.seconds() << "  " << t10.nanoseconds() << endl;
    cout << "    double Seconds:"                 << endl
        << "     " << t1.getTimeInSeconds()       << endl
        << "     " << t10.getTimeInSeconds()      << endl
        << "    double milliseconds:"             << endl
        << "     " << t1.getTimeInMilliseconds()  << endl
        << "     " << t10.getTimeInMilliseconds() << endl;

    t1.setFormat(ctf_secnano);
    t10.setFormat(ctf_secnano);
    cout << "    Format ctf_secnano: "  << endl
        << "     "          << t1       << endl
        << "     "          << t10      << endl;

    t1.setFormat(ctf_datetime);
    t10.setFormat(ctf_datetime);
    cout << "    Format ctf_datetime: " << endl
        << "     "          << t1       << endl
        << "     "          << t10      << endl;

    t1.setFormat(ctf_dtfile);
    t10.setFormat(ctf_dtfile);
    cout << "    Format ctf_dtfile: "   << endl
        << "     "          << t1       << endl
        << "     "          << t10      << endl;

    t1.setFormat(ctf_ms);
    t10.setFormat(ctf_ms);
    cout << "    Format ctf_ms: "       << endl
        << "     "          << t1       << endl
        << "     "          << t10      << endl;

    t1.setFormat(ctf_us);
    t10.setFormat(ctf_us);
    cout << "    Format ctf_us: "       << endl
        << "     "          << t1       << endl
        << "     "          << t10      << endl;

    timespec t1t;
    t1t = t1.getTimeInTimespec();
    cout << "    Timespec" << endl;
    cout << "     " << t1t.tv_sec << " " << t1t.tv_nsec << endl;

    time_t t1tt;
    struct tm * timeinfo;
    char buffer [80];
    t1tt = t1.getTimeInTime_t();
    cout << "    Time_t" << endl;
    cout << "     " << t1tt << endl;
    timeinfo = localtime ( &t1tt );
    cout << "     some formats using time_t" << endl;
    strftime (buffer,80,"%F,%T\n       %I:%M%p\n       %c",timeinfo);
    cout << "       " << buffer << endl;

    cout << "\n  Example: Using Reference Time\n\n";
    cout << "    Set Reference and wait 1 second" << endl << endl;
    t1.setRef();
    sleep(1);
    t1.set();
    CTime tzero = t1.getRef();
    cout << "    Time ref:  " << tzero << endl;
    cout << "    t1 (ref.): " << t1 << endl;
    cout << "    t2 (raw):  " << t2 << endl << endl;
    cout << "    Set useRef(true) for t2 and set its time" << endl << endl;
    t2.set();
    t2.useRef(true);
    cout << "    t2:       " << t2 << "  Is ref used?: " << t2.isRefUsed() << endl << endl;

  cout << "\n  Example: END\n\n";

	return(1);
}

