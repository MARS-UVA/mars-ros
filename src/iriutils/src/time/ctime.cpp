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

#include "ctime.h"
using namespace std;

// static members
CTime CTime::zero;

//X
// CONSTRUCTOR / DESTRUCTOR
//
// CONSTRUCTOR
// set own_time to current time
CTime::CTime()
{
  this->use_zero = false;
  this->set();
  this->setFormat(ctf_us);
}

CTime::CTime(double relative)
{
  this->set(relative);
}

// DESTRUCTOR
//
CTime::~CTime()
{
}

//
// GET TIME
//

unsigned long CTime::seconds(void)
{
  return sec;
}

unsigned long CTime::nanoseconds(void)
{
  return nsec;
}

// IN SECONDS (1.234)

double CTime::getTimeInSeconds(void)
{
  return (double)this->sec + (double)this->nsec/1000000000;
}

// IN MILLISECONDS (1234)
double CTime::getTimeInMilliseconds(void)
{
  return ( (double) ( this->sec * 1000 + this->nsec / 1000000 ));
}

long CTime::getTimeInMicroseconds(void)
{
  return ( (double) ( this->sec * 1000000 + this->nsec / 1000 ));
}

// IN TIMESPEC
// timespec { 1, 234000000 }
timespec CTime::getTimeInTimespec(void)
{
  timespec t;

  t.tv_sec = (long) this->sec;
  t.tv_nsec = (long) this->nsec;

  return t;
}

timeval CTime::getTimeInTimeval(void)
{
  timeval t;

  t.tv_sec = (long) this->sec;
  t.tv_usec = (long) (this->nsec/1000);

  return t;
}

time_t CTime::getTimeInTime_t(void)
{
  return (time_t)this->sec;
}


// SET TIME
// SET (Current time)
void CTime::set(double milliseconds)
{
  timespec time_temp;
  if(milliseconds<0.0)
  {
    #if defined __APPLE__
    struct timeval tv;
    gettimeofday(&tv, NULL);
    time_temp.tv_sec = tv.tv_sec;
    time_temp.tv_nsec = tv.tv_usec*1000;
    #else
    clock_gettime(CLOCK_REALTIME, &time_temp );
    #endif
  }
  else
  {
    time_temp = this->msToTimespec(milliseconds);
  }
  this->sec = time_temp.tv_sec;
  this->nsec = time_temp.tv_nsec;
  if(this->use_zero && milliseconds<0.0)
  {
    CTime tmp;
    tmp.sec    = this->sec;
    tmp.nsec   = this->nsec;
    tmp        = diff(tmp,this->zero);
    this->sec  = tmp.sec;
    this->nsec = tmp.nsec;
  }
}

void CTime::setRef()
{
  this->zero.set();
  this->use_zero = true;
}

CTime CTime::getRef()
 {
   return this->zero;
 }

void CTime::resetRef()
{
  this->zero.set(0);
  this->use_zero = false;
}

void CTime::useRef(bool use)
{
  if(*this >= this->zero)
  {
    *this = diff(*this,this->zero);
  }
  else{
    *this = this->zero;
  }
  this->use_zero = use;
}

bool CTime::isRefUsed()
{
  return this->use_zero;
}

CTime CTime::diff(CTime &t1, CTime &t0)
{
  long long s,n;

  s = (long long)t1.sec - (long long)t0.sec;
  n = (long long)t1.nsec - (long long)t0.nsec;

  if ( n < 0 )
  {
    s -= 1;
    n += 1000000000;
  }

  if (s < 0)
    throw CTimeException(_HERE_,"negative time");

  CTime tmp;
  tmp.sec=s;
  tmp.nsec=n;

  return tmp;
}

// OPERATIONS (t1-t0)
CTime CTime::operator - (CTime &t0)
{
  CTime t1;
  t1.sec=this->sec;
  t1.nsec=this->nsec;
  
  return diff(t1,t0);
}

CTime CTime::operator + (CTime &t)
{
  CTime tmp;
  unsigned long tn;

  if ( this->sec + t.sec < ULONG_MAX )
  {
    tn    = this->nsec + t.nsec;
    tmp.sec = this->sec  + t.sec;

    if( tn / 1000000000 > 0)
      tmp.sec++;

    tmp.nsec = (this->nsec + t.nsec) % 1000000000;
  }else{
    throw CTimeException(_HERE_,"Result higher than ulong max");
  }

  return tmp;
}

CTime CTime::operator / (int div)
{
  CTime tmp;
  double s,ns,t;

  t = (double) this->sec + (((double) this->nsec) / 1000000000) ;
  t /= div;
  ns = modf( t , &s);

  tmp.sec  = (unsigned long) s;
  tmp.nsec = (unsigned long)(  ns * 1000000000 );

  return tmp;
}

bool CTime::operator == (CTime &t)
{
  return( (this->sec == t.sec) && (this->nsec == t.nsec) );
}

bool CTime::operator != (CTime &t)
{
  return( (this->sec != t.sec) || (this->nsec != t.nsec) );
}

bool CTime::operator >= (CTime &t)
{
  if(this->sec == t.sec)
    return((this->nsec >= t.nsec));
  else
    return((this->sec  > t.sec));
}

bool CTime::operator <= (CTime &t)
{
  if(this->sec == t.sec)
    return((this->nsec <= t.nsec));
  else
    return((this->sec  < t.sec));
}

bool CTime::operator > (CTime &t)
{
  if(this->sec == t.sec)
    return((this->nsec > t.nsec));
  else
    return((this->sec  > t.sec));
}

bool CTime::operator < (CTime &t)
{
  if(this->sec == t.sec)
    return((this->nsec < t.nsec));
  else
    return((this->sec  < t.sec));
}

std::ostream& operator << (std::ostream &o,CTime &t)
{
  o << t.getString();

  return o;
}

std::string CTime::getString()
{
  int ms=0,us=0;
  char outstr [80];
  struct tm * timeinfo;
  std::string extra_zero;
  std::stringstream output;

  switch(this->print_format)
  {
    default:
    case ctf_secnano:
      //sec nsec
      //o << t.own_time.tv_sec << " " << t.own_time.tv_nsec;
      if(this->nsec<10)             extra_zero = "00000000";
      else if(this->nsec<100)       extra_zero = "0000000";
      else if(this->nsec<1000)      extra_zero = "000000";
      else if(this->nsec<10000)     extra_zero = "00000";
      else if(this->nsec<100000)    extra_zero = "0000";
      else if(this->nsec<1000000)   extra_zero = "000";
      else if(this->nsec<10000000)  extra_zero = "00";
      else if(this->nsec<100000000) extra_zero = "0";
      output << this->sec << " " << extra_zero << this->nsec;
    break;
    case ctf_datetime:
      //YYYY-MM-DD,HH:MM:SS
      timeinfo = localtime ( (time_t *)&this->sec );
      strftime (outstr,23,"%F,%T",timeinfo);
      output << outstr;
    break;
    case ctf_dtfile:
      //YYYY-MM-DD-HHMMSS-MS
      timeinfo = localtime ( (time_t *)&this->sec );
      strftime (outstr,23,"%F-%H%M%S-",timeinfo);
      output << outstr << (this->nsec/1000000);
    break;
    case ctf_ms:
      // sec.0ms
      ms = round( this->nsec/1000000 );
      if(ms<100 && ms>0) extra_zero = "0";
      if(ms<10  && ms>0)  extra_zero = "00";
      output << this->sec << "." << extra_zero << ms ;
    break;
    case ctf_us:
      // sec.0000us
      us = round( this->nsec/1000 );
      if(us<10)             extra_zero = "00000";
      else if(us<100)       extra_zero = "0000";
      else if(us<1000)      extra_zero = "000";
      else if(us<10000)     extra_zero = "00";
      else if(us<100000)    extra_zero = "0";
      output << this->sec << "." << extra_zero << us ;
    break;
  }

  return output.str();
}

void CTime::setFormat(ctimeformat format)
{
  this->print_format = format;
}

ctimeformat CTime::getFormat()
{
  return this->print_format;
}


// CONVERSION
// timespec to milliseconds
double CTime::timespecToMs( timespec time )
{
  return( time.tv_sec*1000.0 + (time.tv_nsec/1000000.0) );
}

// milliseconds to timespec
timespec CTime::msToTimespec( double time_ms )
{
  timespec temp;

  double seconds = (double)time_ms/1000.0;
  temp.tv_sec  = (long long)floor(seconds);
  temp.tv_nsec = (long long)floor((seconds-temp.tv_sec)*1000000000);

  return temp;
}


