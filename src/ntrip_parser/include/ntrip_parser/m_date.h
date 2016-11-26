// -*- C++ -*-
//
// $Id: m_date.h,v 1.1.1.1 2006/05/30 11:05:27 mervart Exp $
//

#if !defined(__m_date_h__)
// protects the header from been included more than once
#define __m_date_h__
#if defined(__GNUC__)
using namespace std;
#endif
//
// Date & StructDate
// Wrapper around c-lib and system service for date and time.
// class Date is a wrapper around time_t.
// class StructDate is a wrapper around struct tm.
//
#include "m_data.h"
 
class Date;
 
typedef time_t DateType;

// Date - Date -> DateDiff ...
class DateDiff : public  C_Data<DateType> { 
public:
  friend class Date;
  explicit DateDiff(DateType t= 0):C_Data<DateType>(t) {} ;
  DateDiff operator+(const DateDiff &arg2) { return DateDiff(get()+arg2.get()); }
  DateDiff operator*(int factor) { return DateDiff(get()*factor); }
};

class Date : public C_Data<DateType> {
public:
 
  typedef C_Data<DateType> BaseType;
  typedef DateDiff DiffType;

  friend class StructDate;

  Date():C_Data<DateType>(0) {}

  Date(int day,int month,int year);
  Date(int day,int month,int year,int h,int m,int s);


  enum {
    GPS_START = 315964800,             // 6.1.1980 00:00 UTC 
    SECONDS_PER_WEEK = 60 * 60 * 24 * 7
  } ;

  // convient conversion to some stream format
  friend ostream &operator<<(ostream &os,const Date &d) { return d.print(os);}

  // convient conversion from yymmdd
  friend istream &operator>>(istream &is,Date &d) {
#if 1
	d.read_some(is);
#else
    d.get_i(is);
#endif
    return is;
  }

  void tag_set(long tag) {
    set(tag + GPS_START);
  }

  long tag_get() const {
    return get() - GPS_START;
  }


  void gps_set(short gpsWeek,long gpsSeconds) {
    // GPS-Zeit ist linear wie UNIX-Zeit, faengt am an.
    tag_set(gpsWeek * SECONDS_PER_WEEK + gpsSeconds);
  }

  void gps_get(short &gpsWeek,long &gpsSeconds) const {
#if 1
	gpsWeek = tag_get() / SECONDS_PER_WEEK;
	gpsSeconds = tag_get() % SECONDS_PER_WEEK;
#else
    divmod(tag_get(),(long)SECONDS_PER_WEEK,gpsWeek,gpsSeconds);
#endif
  }

  void doy_set(short doy);
  short doy_get() const;

  istream &read_rinex(istream &is,double &fraction = rinex_default_secondsFraction);
  istream &read_some(istream &is);
  ostream &write_rinex(ostream &os);

  // Translate the Date into a string ...
  // with some extentions for special needs ...
  string form(int s= 64,bool trail0=true,const char *fmt="%Y %m %d %H %M %S") const;

  ostream &print(ostream &os,const char *fmt="%d%m%y") const ;

  // getDate reads a day in the form "ttmmyy"
  // for the current system time, but with the day as in the stream.
  // expects that the time is present in the stream.
  //
  // For a complete missing date the function returns false leaving the
  // stream unchanged via putback().
  // For a partial malformed date the character get lost.

  bool get_i(istream &is);


  // returns the time of invocation: unix system time
  static Date timeNow();

  DateDiff operator-(const Date &arg2) const {
    const DateType diff(get() - arg2.get());
    return DateDiff(diff);
  }

  Date operator+(const DateDiff &arg2) const {
    return Date(get() + arg2.get());
  }

private:
  Date(DateType dt):C_Data<DateType>(dt) {}

  bool get_i2(istream &is,int &result);

  static DateType _timeNow();
  static double rinex_default_secondsFraction;
} ;

//
// StructDate
// wrapper around gmtime,mktime and strftime
//
class StructDate {
public:
  typedef struct tm BaseType;

  StructDate();
  // default copy-constructor
  // default assignment
  // default destructor

  StructDate(const Date &date) { set(date); }
  StructDate(int day,int month,int year) { set(day,month,year); }
  StructDate(int day,int month,int year,int h,int m,int s) { set(day,month,year,h,m,s); }

  Date date() const;


  string form(const char *fmt) const;

  void set(int day,int month,int year,int h= 12,int m= 0,int s= 0);
  void set(const Date &date);

  int sec() const { 
    return tms.tm_sec; 
  } 

  int min() const { 
    return tms.tm_min; 
  }

  int hour() const { 
    return tms.tm_hour; 
  }

  int day() const { 
    return tms.tm_mday; 
  }

  int month() const {
    return tms.tm_mon+1; 
  }
  static const char *_monthNames[];
  const char *monthStr();

  int year4() const {
    return tms.tm_year + 1900;
  }

  int year2() const {
    return tms.tm_year%100;
  }

  // returns the day of year 0-based.
  int dayOfYear0() const {
	return tms.tm_yday;
  }

  // return the day of year 1-based.
  int dayOfYear1() const {
	return tms.tm_yday+1;
  }

  BaseType tms;
} ;

#endif
