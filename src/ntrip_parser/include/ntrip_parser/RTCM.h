// -*- C++ -*-
//
// $Id: RTCM.h,v 1.1.1.1 2006/05/30 11:05:27 mervart Exp $

#include "GPSDecoder.h"
#include "m_date.h"
#include <cmath>
#include <map>
#include <iostream>
#include <sys/types.h>
#include <stdint.h>

class RTCM : public GPSDecoder {
  public:
    RTCM(char _StationID,bool fullRange)
    :GPSDecoder()
    ,StationID(_StationID)
    ,lowerBound(fullRange ? -715827883 :  715827882  )
    ,upperBound(fullRange ?  715827882 : 1431655764  )
    ,rolloverIntervall(fullRange ?       4294967296. /* pow(2,32) */ 
		                 :       2147483648. /* pow(2,31) */)
    ,debug(0)
    ,this_word(0)
    ,data_word(0)
    ,next_bits(0)
    ,rtcm_state(/* NO_SYNC */ 0)
    ,fail_count(0)
    ,p_fail(0)
    ,preamble_flag(0)
    ,sync_bit(0)
    ,fill_shift(0)
    ,word_sync(0)
    ,frame_sync(0)
    ,frame_fill(0)
    ,word_count(0)
    ,frame_count(0)
    ,pf_count(0)
    ,msg_type(0)
    ,station_id(0)
    ,z_count(0)
    ,seqno(0)
    ,msg_len(0)
    ,health(0)
    ,fillptr(message)
    ,pfptr(NULL)
    ,ref_x(0.0)
    ,ref_y(0.0)
    ,ref_z(0.0)
    ,last_rx_observation(0.0) {}

    // defined as required by baseclass
    void Decode(char* _ptrBuffer=NULL, int _nBufLen=0) {
      while(_nBufLen--) {
	new_byte((u_char)*_ptrBuffer);
	_ptrBuffer++;
      }
    }

    void new_byte(u_char b);

    std::map<uint32_t, uint32_t> rx_msg_counts;
    int msg_type, station_id, z_count, seqno, msg_len, health;
    float ref_x, ref_y, ref_z;
    double last_rx_observation;

  protected:
    // Station ID used in m_lObsList
    char StationID;

    typedef u_int SVPRN;

    struct Rollover {
      Rollover():lastValue(0),rollOver(0) {}
      int lastValue;
      int rollOver;
    } ;

    struct RolloverChannel {
      Rollover l1;
      Rollover l2;
    } ;

    typedef map<SVPRN,RolloverChannel> RolloverMap;
    RolloverMap rolloverMap;

    // Collector for Measurement
    struct ObsSort {
      ObsSort():C1(0.),P2(0.),l1(0),l2(0),assigned(0) {}
      void set_C1(double _C1) { assigned|=1; C1= _C1; }
      void set_P2(double _P2) { assigned|=2; P2= _P2; }
      void set_l1(int _l1) { assigned|=4; l1= _l1; }
      void set_l2(int _l2) { assigned|=8; l2= _l2; }
      void set_pCodeIndicator(int _pCodeIndicator){ assigned|=16; pCodeIndicator = _pCodeIndicator;}
      void set_cumuLossOfCont(u_char _cumuLossOfCont){ assigned|=32; cumuLossOfCont = _cumuLossOfCont;}
      bool allAssigned() const { return assigned == 63; }
      double C1; // m
      double P2; // m
      int l1; // [1/256 Cycles mod int/half int ]
      int l2; // [1/256 Cycles mod int/half int ]
      int   pCodeIndicator;
      u_int cumuLossOfCont;
      u_int assigned;
    } ;

    typedef map<SVPRN,ObsSort> ObsMap;
    ObsMap obsMap;

    // GPStime mod 3600 of current epoch
    // GLONASS measurements/epochs are ignored
    double obsMapTimeTagMod3600;   

    // Complete mod 3600s time with the help of system time
    // to timetag. Assume that the diffence is below 20min.
    static long completeMod3600BySysTime(int mod3600time) {
      const long sysTimeTag= Date::timeNow().tag_get();
      const long offset = ((sysTimeTag%3600) < 1200) && (mod3600time > 2400) 
                           ? -1
	                   : ((sysTimeTag%3600) > 2400) && (mod3600time < 1200)
                             ? 1
	                     : 0;
      return ((sysTimeTag/3600)+offset)*3600 + mod3600time;
    }


    //
    //
    //
    virtual void endOfEpoch(Date d) {}

    //
    //
    // called if reference position arises in datastream
    virtual void onPosition(double x,double y,double z) {}
 
    // Rollover managment
    int lowerBound;
    int upperBound;
    double rolloverIntervall;

    int rollOverCondition(int oldv,int newv) {
      return (oldv < lowerBound && newv > upperBound )
	     ? -1
	     : (newv < lowerBound && oldv > upperBound )
	       ? 1
	       : 0;
    }

    //
    //
    //
    void updateRollovers() {
      ObsMap::iterator i;
      for(i= obsMap.begin();i != obsMap.end();i++) {
	RolloverMap::iterator rm(rolloverMap.find(i->first));
	if(rm != rolloverMap.end()) {
	  rm->second.l1.rollOver +=
	    rollOverCondition( rm->second.l1.lastValue
		  	      ,i->second.l1 );
	  rm->second.l2.rollOver +=
	    rollOverCondition( rm->second.l2.lastValue
			      ,i->second.l2 );
	} else {
	  rolloverMap[i->first].l1.rollOver= 0;
	  rolloverMap[i->first].l2.rollOver= 0;
	}
	rolloverMap[i->first].l1.lastValue= i->second.l1;
	rolloverMap[i->first].l2.lastValue= i->second.l2;
      }
    }

    //
    //
    //
  virtual void debugOut( SVPRN svprn,long timetag,double mod3600,long mod3600int
			  ,double C1,int l1,int roll_l1,double unrolled_l1
                          ,int l2,int roll_l2,double unrolled_l2) {}

    //
    //
    //
    void dumpObsMap() {
      const int obsMapTimeTagMod3600int=int(rint(obsMapTimeTagMod3600));
      const long timeTag = completeMod3600BySysTime(obsMapTimeTagMod3600int);
      Date d;
      d.tag_set(timeTag);

      updateRollovers();

      endOfEpoch(d);

      ObsMap::iterator i;
      for(i= obsMap.begin();i != obsMap.end();i++) {
	if(!i->second.allAssigned() ) {
          // cerr << "incomplete measurements" << endl;
	} else {
	  Observation  *op = new Observation;

	  op->StatID[0] = StationID;
	  op->StatID[1] = '\0';
	  op->SVPRN= i->first;
	  op->GPSWeek=  timeTag / Date::SECONDS_PER_WEEK;
	  op->GPSWeeks= timeTag % Date::SECONDS_PER_WEEK;
          op->sec = obsMapTimeTagMod3600!=3600.0 ? obsMapTimeTagMod3600 : 0.0;
	  op->C1= i->second.C1;
	  op->P2= i->second.P2;
          op->pCodeIndicator = i->second.pCodeIndicator;
          op->cumuLossOfCont = i->second.cumuLossOfCont;
	  const double l1_offset(  rolloverMap[i->first].l1.rollOver 
				 * rolloverIntervall                 );
	  const double unrolled_l1 = i->second.l1 + l1_offset;
	  op->L1=(unrolled_l1 / -256.) * lambda1;

	  const double l2_offset(  rolloverMap[i->first].l2.rollOver 
				 * rolloverIntervall                 );
	  const double unrolled_l2 = i->second.l2 + l2_offset;
	  op->L2=(unrolled_l2 / -256.) * lambda2;
	  
	  op->SNR1= 0;
	  op->SNR2= 0;
          m_lObsList.push_back(op);
	  debugOut( i->first,timeTag,obsMapTimeTagMod3600,obsMapTimeTagMod3600int
                   ,op->C1
		   ,i->second.l1,rolloverMap[i->first].l1.rollOver,unrolled_l1
		    ,i->second.l2,rolloverMap[i->first].l2.rollOver,unrolled_l2);
	}
      }
      obsMap.clear();
    }

    // SC-104 V2.3 4-42 Note 1 4.
    // Recover TimeTag ( modulo 3600s ) GPS/GLONASS Time and ClockError
    // from expandedTimeOfMeasurement
    // Assume receiver measurements at hard edges at a grid of 10ms 
    // and clock error as less 5 ms (1.1ms max recommended).
    // 
    struct RecoveredTimeTagAndClockError {
      static double fastestUpdatePeriod() { return 0.01; }
      static double shift() { return fastestUpdatePeriod() / 2.; }
      RecoveredTimeTagAndClockError(double expandedTimeOfMeasurement /* [s] */)
      :clockError(fmod(expandedTimeOfMeasurement+shift(),fastestUpdatePeriod()) - shift())
      ,timeTagMod3600(expandedTimeOfMeasurement - clockError) {}
      double clockError; /* [s] */
      double timeTagMod3600;    /* [s] */
    } ;

  virtual void debugTime(double timeTagMod3600,double clockError) {}

    void dumpOnNewTimeTag(double expandedTimeOfMeasurement) {
      const RecoveredTimeTagAndClockError r(expandedTimeOfMeasurement);
      if(r.timeTagMod3600 != obsMapTimeTagMod3600) {
        debugTime(r.timeTagMod3600,r.clockError);
      	if(obsMap.size()) {
          dumpObsMap();
        }
      	obsMapTimeTagMod3600 = r.timeTagMod3600;
      }
    }

    int preamble();
    void state_change(u_int s);
    u_int parity_ok();
    int find_sync(u_char b);
    void next_word();
    int filled_word(u_char b);

    void msgRTKUncorrectedCarrierPhases();
    void msgRTKUncorrectedPseudoranges();

    void printcor();
    void printref();
    void printba();
    void printspec();
    void printnull();
    void printdatum();
    void printconh();

    void new_frame();
    void buffer(u_int w);
    void frame_start();
    void find_start();
    void fill_frame();
    void status_byte(u_char b);
    void data_byte(u_char b);

    int	debug;

    u_int this_word, data_word;
    u_char next_bits;

    int rtcm_state, fail_count, p_fail, preamble_flag;
    int sync_bit, fill_shift, word_sync, frame_sync, frame_fill;
    int word_count, frame_count, pf_count;

    u_int message[40];	        /* message buffer */
    u_int *fillptr;		/* pointer to fill message */
    u_int *pfptr;		/* pointer to first parity error */

    static char *state_name[];
    static u_int tx_speed[];
    static u_char parity_of[];
    static u_char reverse_bits[];
} ;
