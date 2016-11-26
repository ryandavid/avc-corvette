// -*- C++ -*-
//
// $Id: m_date.cpp,v 1.1.1.1 2006/01/11 09:34:31 mervart Exp $
//
// Comment: c.f. m_date.h


#include "ntrip_parser/m_date.h"

// for memset
#include <string.h>

//
// Initialize
//
StructDate::StructDate() {
  memset(&tms,0,sizeof(tms));
}

//
// construct from Date-Instance
// 
void StructDate::set(const Date &date) {
  // is not 100% thread save ...
  tms = *gmtime(&date.data);
}

//
// calculate a date-Instance
//
Date StructDate::date() const {
  Date d;
  // breaks constness: tms can be normalized by mktime
#if !defined(__CYGWIN32__)
  d.data = mktime((BaseType *)&tms) - timezone; 
#else
  d.data = mktime((BaseType *)&tms); 
#endif
  return d;
}

//
// convert via well-known strftime timeformat string into a string
// 
string StructDate::form(const char *fmt) const {
  char buffer[1024];
  strftime(buffer,sizeof(buffer),fmt,&tms);
  return string(buffer);
}

//
// Construct from day,month year ...
//
void StructDate::set(int day,int month,int year,int h,int m,int s) {
  StructDate sd(Date::timeNow());
  sd.tms.tm_sec= s;
  sd.tms.tm_min= m;
  sd.tms.tm_hour= h;
  sd.tms.tm_mday = day;
  sd.tms.tm_mon = month-1;
  sd.tms.tm_year = ((year >= 1900) ? -1900 : 0) +  year + ((year < 80) ? 100 : 0);
  *this = sd;
}


const char *StructDate::_monthNames[] 
= { "jan","feb","mar","apr","may","jun"
    ,"jul","aug","sep","oct","nov","dez" } ;

const char *StructDate::monthStr() {
  return _monthNames[tms.tm_mon];
}

// ------------------------------------------------------------------------------------

Date::Date(int day,int month,int year) {
  StructDate sd(day,month,year);
  set(sd.date().get());
}

Date::Date(int day,int month,int year,int h,int m,int s) {
  StructDate sd(day,month,year,h,m,s);
  set(sd.date().get());
}

void Date::doy_set(short doy) {
  const StructDate current(timeNow());
  const StructDate sd2(doy,1,current.year4());
  set(sd2.date().get());
}

short Date::doy_get() const {
  const StructDate sd(*this);
  return sd.dayOfYear1();
}



// Converting to string and streaming out is more a matter of DateStruct 
// but Date was the first.

string Date::form(int s,bool trail0,const char *fmt) const {
  const StructDate sd(*this);
  string buffer(sd.form(fmt));
  if(s>0) { 
    buffer.resize(s,' ');
    if(trail0) buffer[s-1]= '\0';
  }
  return buffer;
}


//
// prints out the date on the given ostream
//
ostream & Date::print(ostream &os,const char *fmt) const {
  const StructDate sd(*this);
  os << sd.form(fmt);
  return os;
}

istream & Date::read_some(istream &is) {
  string formS("");
  if(is.peek() == '('
	 && is.get()) {
	while(is.peek() != ')') {
		formS = formS + char(is.get());
	}
	is.get();
	if(formS == "rinex") return read_rinex(is);
	else if(formS == "gps_tag") {
		long tag;
		if(is >> tag) {
		  tag_set(tag);
		  return is;
		}
	} else if (formS == "gps") {
		short week;
		long weekSec;
		if (   is >> week 
			&& is >> weekSec) {
			gps_set(week,weekSec);
			return is;
		}
	} 
  } else {
	get_i(is);
  } 
  return is;
}


//
double Date::rinex_default_secondsFraction;

//
istream & Date::read_rinex(istream &is,double &fraction) {
  StructDate sd;
  float fsec;

  if (   is   >> sd.tms.tm_year
      && is   >> sd.tms.tm_mon
      && is   >> sd.tms.tm_mday
      && is   >> sd.tms.tm_hour
      && is   >> sd.tms.tm_min
      && is   >> fsec           ) {
    sd.tms.tm_sec = int(fsec);
	sd.tms.tm_mon--;
	fraction = fsec - sd.tms.tm_sec;
    // y2k !!!
    if(sd.tms.tm_year <= 80) sd.tms.tm_year += 100;
	Date d2(sd.date());
    set(d2.get());
  }
  return is;
}

//
bool Date::get_i(istream &is) {
  int day;
  int month;
  int year;
  
  if(   get_i2(is,day)
     && get_i2(is,month)
     && get_i2(is,year)) {
    const StructDate sd(day,month,year);
    *this =  sd.date();
    return true;
  }
  return false;
}

//
// get2 - misplaces decimal to number converter and reader reads exactly 2 digits
// and writes them into result.

bool Date::get_i2(istream &is,int &result) {
  char c1;
  result= 0;
  
  do {
    is >> c1;
  } while(is && (c1 == ' '));

  if(isdigit(c1)) {
    result = 10 * ( c1 - '0' );
    char c2;
    is >> c2;
    if(isdigit(c2)) {
      result += (c2 - '0');
      return true;
    } else is.putback(c2);
  } else is.putback(c1);
  return false;
}

// arquire the current time ...

// DateType Date::_timeNow()
// gets the current time
DateType Date::_timeNow() { 
  return time(NULL);
} 

//
// (static) Date Date::time()
// Constructs a Date from the system time at time of invocation.
Date Date::timeNow() {
  Date d;
  d.data = _timeNow();
  return d;
}

