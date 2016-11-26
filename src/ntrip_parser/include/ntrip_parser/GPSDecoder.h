// -*- C++ -*-
//
// $Id: GPSDecoder.h,v 1.1.1.1 2006/01/11 09:34:31 mervart Exp $
// 2005/04/11: include 'int iPCode' into class 'GPSDecoder' (BKG)
#if !defined(__GPSDecoder_h__)
#define __GPSDecoder_h__

#include <list>
#include <iostream>

using namespace std;


#include "format.h"

//
// One Code/Phase - Measurement
//
struct Observation {
    Observation()
    :SVPRN(0)
    ,GPSWeek(0)
    ,GPSWeeks(0)
    ,sec(0.) 
    ,C1(0.)
    ,P2(0.)
    ,L1(0.)
    ,L2(0.)
    ,SNR1(0)
    ,SNR2(0) 
    ,pCodeIndicator(0)
    ,cumuLossOfCont(0)
    {StatID[0] = '\0';}

    Observation( char _statID
	        ,char _svprn
	        ,short _GPSWeek
	        ,int _GPSWeeks
		,double _sec
	        ,double _C1
		,double _P2
		,double _L1
		,double _L2
		,short _SNR1
		,short _SNR2
		,int _pCodeIndicator
		,u_int _cumuLossOfCont)
    :SVPRN(_svprn)
    ,GPSWeek(_GPSWeek)
    ,GPSWeeks(_GPSWeeks)
    ,sec(_sec)
    ,C1(_C1)
    ,P2(_P2)
    ,L1(_L1)
    ,L2(_L2)
    ,SNR1(_SNR1)
    ,SNR2(_SNR2) 
    ,pCodeIndicator(_pCodeIndicator)
    ,cumuLossOfCont(_cumuLossOfCont)
    {StatID[0] = _statID; StatID[1] = '\0';}

    char StatID[5+1];  //< Station ID
    char SVPRN;    //<  Satellite PRN
    short GPSWeek; //< Week of GPS-Time
    int GPSWeeks; //< Second of Week (GPS-Time>
    double sec;
    double C1;     //<  CA-code validated raw pseudorange (meters)
    double P2;     //<  P2-code validated raw pseudorange (meters)
    double L1;     //<  validated raw carrier phase (meters)
    double L2;     //<  validated raw carrier phase (meters)
    short SNR1;    //<  signal-to noise ration (0.1 dB)
    short SNR2;    //<  signal-to noise ration (0.1 dB)
    int   pCodeIndicator;  // 0 ... CA Code, 1 ... P Code
    u_int cumuLossOfCont;  // 0 to 31

    // Operator to write ascii formatted members of Observation to an outstream
    friend ostream& operator<<(ostream& os, const Observation & o) {
	os <<format("%3d%3d", atoi(o.StatID) ,(int) o.SVPRN)
	   <<format(" %4d %10d",(int)o.GPSWeek, o.GPSWeeks)
	   <<format("\t%15.3f\t%15.3f\t%15.3f\t%15.3f\t%15.3f\t%15.3f"
		    ,o.C1, o.P2, o.L1, o.L2, 0.1*o.SNR1, 0.1*o.SNR2)
	   <<endl;
	return os;
    }
} ;

//
// GPS Orbitinformation
//
struct Ephemeris {
    short   svprn;    //< Satellite PRN
    short   wn;       //< GPS - week number
    short   aodc;     //< Age of data issue Clock
    short   aode;     //< Age of data issue Orbit
    double  tow;      //< Seconds of GPS week
    double  toc;      //< Reference time, Clock (sec)
    double  toe;      //< Ref.time for Orbit:(sec)
    double  tgd;      //< Group delay (sec)
    double  af2;      //< Clock parameter: (sec/sec^2)
    double  af1;      //< Clock parameter: (sec/sec)
    double  af0;      //< Clock parameter: (sec)
    double  crs;      //< Sin-harmonic correction term, orbit radius:(meters)
    double  deltan;   //< Mean anomaly correction:(semi-cirl/sec)
    double  m0;       //< Mean anomaly @ ref.time:(semi-circle)
    double  cuc;      //< Cos-harmonic correction term, argument of Latitude:(radians)
    double  e;        //< Eccentricity
    double  cus;      //< Sin-harmonic correction term, argument of Latitude:(radians)
    double  roota;    //< Square root of semi-major axis:(m ^1/2)
    double  cic;      //< Cos-harmonic correction term, angle of inclination:(radians)
    double  omega0;   //< Lon. of Asc. node at weekly epoch: (semi-circle)
    double  cis;      //< Sin-harmonic correction term, angle of Inclination:(radians)
    double  i0;       //< Inclination angle at Ref.time: (semi-circle)
    double  crc;      //< Cos-harmonic correction term, orbit radius:(meters)
    double  omega;    //< Argument of Perigee:(semi-circle)
    double  omegadot; //< Rate of right ascension: (semi-circle/sec)
    double  idot;     //< Rate of inclination angle: (semi-circle/sec)
    short   svaccu;   //< SV accuracy (0-15)
    short   fit;      //< Curve fit interval (0-1)
    short   cReserved1; //< Not used - yet
    short   health;     //< 0 if healthy, else unhealthy
    short   cReserved2; //< Not used - yet
    short   SVEnable;   //< Not used - yet
} ;

const double   lambda1 = 0.1902936727984; // [m]
const double   lambda2 = 0.2442102134241; // [m]
const unsigned glonass_svid = 45;

class GPSDecoder {
 public:
  int iPCode;  // pointer for CA or P code on L1

  virtual void Decode(char* _ptrBuffer=NULL, int _nBufLen=0) = 0;
  virtual ~GPSDecoder() {}
  typedef list<Observation*> ObsList_t;
  typedef list<Ephemeris*>   EphList_t;

  ObsList_t m_lObsList;
  EphList_t m_lEphList;

} ;

#endif
