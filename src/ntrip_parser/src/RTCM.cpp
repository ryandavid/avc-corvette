// Original attribution and licensing below.
// Modified to be a ROS node and add additional message types
// to the decoder.


#include "ntrip_parser/RTCM.h"

#include <cmath>

#if !defined(__GNUC__)
double rint(double val) {
  return ((val < 0.0) ? -floor(-val+0.5) : floor(val+0.5));
}
#endif

/* Org-ID: rtcm.c,v 1.11 1999/08/28 00:06:20 wolfgang Exp   */

/* rtcm.c - decode RTCM SC-104 data from a DGPS beacon receiver */

/*
    written by John Sager john.sager@btinternet.com
    Copyright (C) 1999  John C Sager

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

*/

/*  History:

    v0.3	22nd February 1999
	Incorrect scale factor on range error field fixed.
	dx,dy,dz fields in Datum message are signed.

    v0.2	9th February 1999
	Sundry improvements to the code by Wolfgang Rupprecht so
	he could incorporate it in his dgpsip client.

	Addition of decode capability for message types 4 and 5.

    v0.1	4th February 1999
	First version - reads RTCM data from standard input & sends
	decoded output to standard output. Implements decoding of
	types 1,3,6,7,9,16. This is original software and not
	a modification of any other program.
*/

/* TODO
    Add facility to decode from tty port input & write to a file
*/

/*
 * The data comes across as bytes where the top 2 bits indicate the type
 * of data and the bottom 6 bits are the data itself. Bytes 01xxxxxx
 * are received RTCM data. Other bytes are other kinds of data/commands
 * etc. The data framing can be at any bit position. Also the data is
 * lsb first, so it needs to be bit-reversed.
 *
 * This program calculates parity over a 32-bit field starting at each
 * bit position until a parity match is found. Then parity is checked
 * at successive 30-bit boundaries until sufficient error-free words
 * have been found. This gives word sync. Then frame sync is acquired
 * by searching for the preamble and checking at subsequent frame boundaries
 * until sufficient frames have been found. Then data is decoded.
 *
 * Parity errors are flagged in frames so that data before the parity
 * error can be decoded, if possible. A parity error in the preamble
 * word or the next word causes frame sync to be lost. Sufficient
 * successive parity errors causes sync to be lost totally.
 */
#define DEBUG

#ifdef DEBUG
#include <stdio.h>
#include <stdlib.h>
#endif
#include <sys/types.h>


/* state machine state */

#define	NO_SYNC		0
#define WORD_SYNCING	1
#define WORD_SYNC	2
#define FRAME_SYNCING	3
#define FULL_SYNC	4

#define W_SYNC_TEST	6
#define F_SYNC_TEST	3
#define P_FAIL_TEST	10

/* message types */

#define MSG_FULLCOR	1
#define MSG_REFPARM	3
#define MSG_DATUM	4
#define MSG_CONHLTH	5
#define MSG_NULL	6
#define MSG_BEACALM	7
#define MSG_SUBSCOR	9
#define MSG_SPECIAL	16

/* field scale factors */

#define	ZCOUNT_SCALE	0.6		/* sec */
#define	RANGE_SMALL	0.02		/* metres */
#define	RANGE_LARGE	0.32		/* metres */
#define	RANGERATE_SMALL	0.002		/* metres/sec */
#define	RANGERATE_LARGE	0.032		/* metres/sec */
#define XYZ_SCALE	0.01		/* metres */
#define DXYZ_SCALE	0.1		/* metres */
#define	LA_SCALE	90.0/32767.0	/* degrees */
#define	LO_SCALE	180.0/32767.0	/* degrees */
#define	FREQ_SCALE	0.1		/* kHz */
#define	FREQ_OFFSET	190.0		/* kHz */
#define CNR_OFFSET	24		/* dB */
#define TU_SCALE	5		/* minutes */

char * RTCM::state_name[] = {
 "NO_SYNC",
 "WORD_SYNCING",
 "WORD_SYNC",
 "FRAME_SYNCING",
 "FULL_SYNC"};

u_int RTCM::tx_speed[] = { 25, 50, 100, 110, 150, 200, 250, 300 };

const static char *freqIndicatorS[] = { "L1" , "?1" , "L2" , "?3" };

/* parity stuff */

#define P_30_MASK	0x40000000
#define P_31_MASK	0x80000000

#define	PARITY_25	0xbb1f3480
#define	PARITY_26	0x5d8f9a40
#define	PARITY_27	0xaec7cd00
#define	PARITY_28	0x5763e680
#define	PARITY_29	0x6bb1f340
#define	PARITY_30	0x8b7a89c0

#define W_DATA_MASK	0x3fffffc0

u_char RTCM::parity_of[] = {
0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,
0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0,1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,
1,0,0,1,0,1,1,0,0,1,1,0,1,0,0,1,0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0};

u_char RTCM::reverse_bits[] = {
0,32,16,48,8,40,24,56,4,36,20,52,12,44,28,60,
2,34,18,50,10,42,26,58,6,38,22,54,14,46,30,62,
1,33,17,49,9,41,25,57,5,37,21,53,13,45,29,61,
3,35,19,51,11,43,27,59,7,39,23,55,15,47,31,63};


#define DATA_SHIFT	6
#define B_DATA_MASK	0x3f
#define FILL_BASE	24

#define PREAMBLE	0x19800000
#define PREAMBLE_MASK	0x3fc00000

int RTCM::preamble() {
  return ((data_word & PREAMBLE_MASK) == PREAMBLE);
}

/* state_change - change state & print a useful message */

void RTCM::state_change(u_int s) {
#ifdef DEBUG
  printf("M\tstate change: %s -> %s\n",
	state_name[rtcm_state], state_name[s]);
  fflush(stdout);
#endif
  rtcm_state = s;
}

/* check the parity on this_word. bits 31,30 are parity on previous word.
 * bits 29-6 are data bits. Bits 5-0 are parity bits.
 */

u_int RTCM::parity_ok() {
  u_int t, th, p;

  th = this_word;
  if (th & P_30_MASK)
    th ^= W_DATA_MASK;

  t = th & PARITY_25;
  p = parity_of[t & 0xff] ^ parity_of[(t>>8)&0xff] ^
      parity_of[(t>>16)&0xff] ^ parity_of[(t>>24)];
  t = th & PARITY_26;
  p = (p<<1) | (parity_of[t & 0xff] ^ parity_of[(t>>8)&0xff] ^
      parity_of[(t>>16)&0xff] ^ parity_of[(t>>24)]);
  t = th & PARITY_27;
  p = (p<<1) | (parity_of[t & 0xff] ^ parity_of[(t>>8)&0xff] ^
      parity_of[(t>>16)&0xff] ^ parity_of[(t>>24)]);
  t = th & PARITY_28;
  p = (p<<1) | (parity_of[t & 0xff] ^ parity_of[(t>>8)&0xff] ^
      parity_of[(t>>16)&0xff] ^ parity_of[(t>>24)]);
  t = th & PARITY_29;
  p = (p<<1) | (parity_of[t & 0xff] ^ parity_of[(t>>8)&0xff] ^
      parity_of[(t>>16)&0xff] ^ parity_of[(t>>24)]);
  t = th & PARITY_30;
  p = (p<<1) | (parity_of[t & 0xff] ^ parity_of[(t>>8)&0xff] ^
      parity_of[(t>>16)&0xff] ^ parity_of[(t>>24)]);


  if (!this_word || ((this_word &0x3f) != p)) {
    if (rtcm_state > WORD_SYNCING)
      pf_count++;
    return 0;
  }
  
  return th;
}

/* find word sync by a successful parity check */

int RTCM::find_sync(u_char b) {
  int i;

  b <<= 2;
  i = 1;

  while (i <= DATA_SHIFT) {
    this_word <<= 1;
    this_word |= (b & 0x80) ? 1 : 0; /* next bit into 32 bits */
    b <<= 1;
    if ((data_word = parity_ok())) {
      sync_bit = (i==DATA_SHIFT) ? 0 : i;
      fill_shift = FILL_BASE - DATA_SHIFT + i;
      next_bits = b >> 2;
      return 1;
    }
    i++;
  }
  return 0;
}

void RTCM::next_word() {
  this_word = (this_word << 30) | (next_bits << 24);
}

/* fill bits into this_word and indicate when filled. Put the residue bits in
   next_bits.
 */

int RTCM::filled_word(u_char b) {

  if (fill_shift <= 0) { /* can complete fill */
    if (fill_shift) {
      next_bits = b << (DATA_SHIFT + fill_shift);
      this_word |= b >> (-fill_shift);
    }
    else {
      next_bits = 0;
      this_word |= b;
    }
    fill_shift += FILL_BASE;
    word_count++;
    return 1;
  }
  this_word |= b << fill_shift;
  fill_shift -= DATA_SHIFT;
  return 0;
}


//
//
//
void RTCM::msgRTKUncorrectedCarrierPhases() {

  
  const u_int freqIndicator   = (message[2]>>28)&0x3;

  const u_int gnssTimeofMeasurement = (message[2]>>6)&0xfffff;
  const double expandedTimeOfMeasurement = z_count*ZCOUNT_SCALE + gnssTimeofMeasurement * .000001;

  int m;
  for(m= 3 ; m < msg_len+2 ; m+=2 ) {
    const int multipleMessageIndicator = (message[m] >> 29) & 0x1;
    const int pCodeIndicator  = (message[m] >> 28) & 0x1;
    const int glonassIndicator = (message[m] >> 27) & 0x1;

#ifdef DEBUG
    const u_int svprn    = (message[m] >> 22) & 0x1f 
      + (glonassIndicator ? glonass_svid : 0);
#else
    const u_int svprn    = (message[m] >> 22) & 0x1f;
#endif
    if(!glonassIndicator) {
      dumpOnNewTimeTag(expandedTimeOfMeasurement);

      const u_int dataQuality    = (message[m] >> 18) & 0x7;
      const u_int cumuLossOfCont = (message[m] >> 14) & 0x1f;

      obsMap[svprn].set_cumuLossOfCont(cumuLossOfCont);
      obsMap[svprn].set_pCodeIndicator(pCodeIndicator);

      const int   carrierPhase   = (int)(  (((message[m]  >>  6) & 0xff) << 24)
	  			         | ((message[m+1]>>  6) & 0xffffff));
      switch(freqIndicator) {
      case 0: /* L1 */
        // TBD unroll L1
        obsMap[svprn].set_l1(carrierPhase);
        if(pCodeIndicator > 0) {
          iPCode++;
        }
        break;
      case 2: /* L2 */
        // TBD unroll L2
        obsMap[svprn].set_l2(carrierPhase);
        break;
      default:
        /* unknown frequency */;
      }
    }

#ifdef DEBUG
    printf("   * Carrier Phases\n");
    printf("     - PRN:\t%d\n", svprn);
    printf("     - System:\t%s\n", glonassIndicator ? "GLONASS" : "GPS");
    printf("     - Freq:\t%s\n", freqIndicatorS[freqIndicator]);
#endif
  }
}


void RTCM::msgRTKUncorrectedPseudoranges() {
  const u_int freqIndicator   = (message[2]>>28)&0x3;
  const u_int smoothingIntervall = (message[2]>>26)&0x3;
  const u_int gnssTimeofMeasurement = (message[2]>>6)&0xfffff;
  const double expandedTimeOfMeasurement = z_count*ZCOUNT_SCALE + gnssTimeofMeasurement * .000001;

  int m;
  for(m= 3 ; m < msg_len+2 ; m+=2 ) {
    const int multipleMessageIndicator = (message[m] >> 29) & 0x1;
    const int pCodeIndicator  = (message[m] >> 28) & 0x1;
    const int glonassIndicator = (message[m] >> 27) & 0x1;
    if(!glonassIndicator) {
      dumpOnNewTimeTag(expandedTimeOfMeasurement);
      const u_int svprn    = (message[m] >> 22) & 0x1f 
                             + (glonassIndicator ? glonass_svid : 0) ;
      const u_int dataQual   = (message[m] >> 18) & 0xf;
      const u_int multiPathError = (message[m] >> 14) &0xf;
      const u_int pseudoRange    = (  (((message[m]  >>  6) & 0xff) << 24)
			           | ((message[m+1]>>  6) & 0xffffff));
      switch(freqIndicator) {
      case 0: /* L1 */
        obsMap[svprn].set_C1(pseudoRange*0.02);
        if(pCodeIndicator > 0) {
          iPCode++;
        }
        break;
      case 2: /* L2 */
        obsMap[svprn].set_P2(pseudoRange*0.02);
        break;
      default:
        /* unknown frequency */;
      }

      #ifdef DEBUG
        printf("   * Pseudoranges:\n");
        printf("     - PRN:\t%d\n", svprn);
        printf("     - System:\t%s\n", glonassIndicator ? "GLONASS" : "GPS");
        printf("     - Freq:\t%s\n", freqIndicatorS[freqIndicator]);
      #endif
    }
  }
}


/* printcor - print differential corrections - full or subset */

void RTCM::printcor() {
  int i, w;
  u_int m, n;
  int scale, udre, sat, range, rangerate, iod;

  i = 0;
  w = 2;
  m = 0;
  if (pfptr) {
    msg_len = (pfptr-message) - 2;
    n = msg_len % 5;
    if (n == 1 || n == 3) msg_len--;
    if (msg_len < 2)
      return;
  }
  while (w < msg_len+2) {
    if ((i & 0x3) == 0){
      m = message[w++] & W_DATA_MASK;
      scale = m >> 29 & 0x1;
      udre = (m >>27) & 0x3;
      sat = (m >>22) & 0x1f;
      range = (m >>6) & 0xffff;
      if (range > 32767) range -= 65536;
      m = message[w++] & W_DATA_MASK;
      rangerate = (m >>22) & 0xff;
      if (rangerate > 127) rangerate -= 256;
      iod = (m >>14) & 0xff;
      i++;
    }
    else if ((i & 0x3) == 1){
      scale = m >> 13 & 0x1;
      udre = (m >>11) & 0x3;
      sat = (m >>6) & 0x1f;
      m = message[w++] & W_DATA_MASK;
      range = (m >>14) & 0xffff;
      if (range > 32767) range -= 65536;
      rangerate = (m >>6) & 0xff;
      if (rangerate > 127) rangerate -= 256;
      m = message[w++] & W_DATA_MASK;
      iod = (m >>22) & 0xff;
      i++;
    }
    else {
      scale = m >> 21 & 0x1;
      udre = (m >>19) & 0x3;
      sat = (m >>14) & 0x1f;
      range = (m <<2) & 0xff00;
      m = message[w++] & W_DATA_MASK;
      range |= (m >>22) & 0xff;
      if (range > 32767) range -= 65536;
      rangerate = (m >>14) & 0xff;
      if (rangerate > 127) rangerate -= 256;
      iod = (m >>6) & 0xff;
      i+= 2;
    }
#ifdef DEBUG
    printf("S\t%d\t%d\t%d\t%.1f\t%.3f\t%.3f\n", sat, udre, iod,
	z_count*ZCOUNT_SCALE, range*(scale?RANGE_LARGE:RANGE_SMALL),
	rangerate*(scale?RANGERATE_LARGE:RANGERATE_SMALL));
#endif
  }
}

/* printref - print reference position. The transmitted xyz quantities are
   integers scaled in units of 0.01m
*/

void RTCM::printref(void) {
  int x, y, z;

  if (pfptr)
    return;
  x = ((message[2] & W_DATA_MASK) << 2) | ((message[3] & W_DATA_MASK) >> 22);
  y = ((message[3] & W_DATA_MASK) << 10) | ((message[4] & W_DATA_MASK) >> 14);
  z = ((message[4] & W_DATA_MASK) << 18) | ((message[5] & W_DATA_MASK) >> 6);

  onPosition(x*XYZ_SCALE, y*XYZ_SCALE, z*XYZ_SCALE);
 
#ifdef DEBUG
  printf("   * Reference Station\n");
  printf("     - X:\t%fm\n", x * XYZ_SCALE);
  printf("     - Y:\t%fm\n", y * XYZ_SCALE);
  printf("     - Z:\t%fm\n", z * XYZ_SCALE);
#endif
}

/* printba - print beacon almanac
*/

void RTCM::printba() {
  int la, lo, range, freq, hlth, id, bitrate;

  if (pfptr)
    return;
  la = ((message[2] >> 14) & 0xffff);
  if (la > 32767) la -= 65536;
  lo = ((message[2] << 2) & 0xff00) | ((message[3] >> 22) & 0xff);
  if (lo > 32767) lo -= 65536;
  range = ((message[3] >> 12) & 0x3ff);
  freq = ((message[3]) & 0xfc0) | ((message[4] >> 24) & 0x3f);
  hlth = ((message[4] >> 22) & 0x3);
  id = ((message[4] >> 12) & 0x3ff);
  bitrate = ((message[4] >> 9) & 0x7);

#ifdef DEBUG
  printf("A\t%.4f\t%.4f\t%d\t%.1f\t%d\t%d\t%d\n", (la*LA_SCALE), (lo*LO_SCALE),
	range, (freq*FREQ_SCALE+FREQ_OFFSET), hlth, id, tx_speed[bitrate]);
#endif
}

/* printspec - print text of special message
*/

void RTCM::printspec() {
  u_int i, d, c;
#ifdef DEBUG
  if (pfptr)
    msg_len = (pfptr-message)>>2;
  printf("T\t");
  for (i=0; i< msg_len; i++) {
    d = message[i+2] & W_DATA_MASK;
    if ((c=d>>22)) putchar(c); else break;
    if ((c=((d>>14) & 0xff))) putchar(c); else break;
    if ((c=((d>>6) & 0xff))) putchar(c); else break;
  }
  printf("\n");
#endif
}

/* printnull - print a marker for a null message
*/

void RTCM::printnull() {
#ifdef DEBUG
    printf("N\n");
#endif
}

/* printdatum - print datum message
*/

void RTCM::printdatum() {
  char dname[6];
  char *dn;
  u_int d, dgnss, dat;
  int dx, dy, dz;

  if (pfptr)
    return;
  d = message[2] & W_DATA_MASK;
  dgnss = d>>27;
  dat = (d>>26) & 1;
  dname[0] = (d>>14) & 0xff;
  if (dname[0]) { /* not null */
    dname[1] = (d>>6) & 0xff;
    d = message[3] & W_DATA_MASK;
    dname[2] = (d>>22) & 0xff;
    dname[3] = (d>>14) & 0xff;
    dname[4] = (d>>6) & 0xff;
    dname[5] = '\0';
    dn = dname;
  }
  else
    dn = "NUL";
#ifdef DEBUG
  printf("D\t%s\t%1d\t%s", (dgnss==0)?"GPS":((dgnss==1)?"GLONASS":"???"),
	dat, dn);
#endif
  if (msg_len > 2) {
    d = message[4] & W_DATA_MASK;
    dx = (d>>14) & 0xffff;
    if (dx > 32767) dx -= 65536;
    dy = (d<<2) & 0xff00;
    d = message[5] & W_DATA_MASK;
    dy |= (d>>22) & 0xff;
    if (dy > 32767) dy -= 65536;
    dz = (d>>6) & 0xffff;
    if (dz > 32767) dz -= 65536;
#ifdef DEBUG
    printf("\t%.1f\t%.1f\t%.1f", dx*DXYZ_SCALE, dy*DXYZ_SCALE, dz*DXYZ_SCALE);
#endif
  }
#ifdef DEBUG
  printf("\n");
#endif
}

/* printconh - print constellation health message
*/

void RTCM::printconh() {
  u_int i, d, sat, iodl, hlth, cnr, he, nd, lw, tu;

  if (pfptr) {
    msg_len = (pfptr-message) - 2;
    if (!msg_len)
      return;
  }
  for (i=0; i< msg_len; i++) {
    d = message[i+2] & W_DATA_MASK;
    sat = (d>>24) & 0x1f;
    if (!sat) sat = 32;
    iodl = (d>>23) & 1;
    hlth = (d>>20) & 0x7;
    cnr = (d>>15) & 0x1f;
    he = (d>>14) & 1;
    nd = (d>>13) & 1;
    lw = (d>>12) & 1;
    tu = (d>>8) & 0x0f;
#ifdef DEBUG
    printf("C\t%2d\t%1d  %1d\t%2d\t%1d  %1d  %1d\t%2d\n",
	sat, iodl, hlth, (cnr?(cnr+CNR_OFFSET):-1), he, nd, lw, tu*TU_SCALE);
#endif
  } 
}

/* new_frame - called when a new frame is complete */

void RTCM::new_frame() {
  char s[8];

  frame_count++;
  if (pfptr == message) /* dud frame */
    return;

  msg_type = (message[0]>>16)&0x3f;
  station_id = (message[0]>>6)&0x3ff;
  z_count = (message[1]>>17)&0x1fff;
  health = (message[1]>>6)&0x7;
#ifdef DEBUG
  if (pfptr)
    sprintf(s, "\tT\t%d", (pfptr-message)-2);

  printf("New Frame:\n");
  printf(" - Msg Type:\t%d\n", msg_type);
  printf(" - Station ID:\t%d\n", station_id);
  printf(" - Z Count:\t%d\n", z_count);
  printf(" - Sequence:\t%d\n", seqno);
  printf(" - Msg Length:\t%d\n", msg_len);
  printf(" - Health:\t%d\n", health);
#endif
  switch (msg_type) {

    case MSG_FULLCOR:
    case MSG_SUBSCOR:
      printcor();
      break;

    case MSG_REFPARM:
      printref();
      break;

    case MSG_DATUM:
      printdatum();
      break;

    case MSG_CONHLTH:
      printconh();
      break;

    case MSG_NULL:
      printnull();
      break;

    case MSG_BEACALM:
      printba();
      break;

    case MSG_SPECIAL:
      printspec();
      break;

    case 18: /* RTK Uncorrected Carrier Phases */
      msgRTKUncorrectedCarrierPhases();
      break;

    case 19: /* RTK Uncorrected Pseudoranges */
      msgRTKUncorrectedPseudoranges();
      break;

    default:
#ifdef DEBUG
      printf("?\t%d\n", msg_type);
#endif
      break;
  }
}

void RTCM::buffer(u_int w) {
  *fillptr++ = w;
}

void RTCM::frame_start() {
  frame_fill = -1;
  fillptr = message;
  pfptr = NULL;
  buffer(data_word);
}

void RTCM::find_start() {
  if ((data_word = parity_ok()))
    p_fail = 0;
  else if (++p_fail >= P_FAIL_TEST) { /* too many consecutive parity fails */
    state_change(NO_SYNC);
    return;
  }
  next_word();
  if (preamble()) {
    seqno = -1; /* resync at next word */
    frame_start();
    state_change(FRAME_SYNCING);
  }
}

void RTCM::fill_frame() {
  int seq;

  if ((data_word = parity_ok()))
    p_fail=0;
  else if (++p_fail >= P_FAIL_TEST) {
    state_change(NO_SYNC);
    return;
  }
  next_word();
  frame_fill++; /* another word */
  if (frame_fill == 0) { /* this is second header word */
    if (!data_word) { /* parity fail - bad news! */
      state_change(WORD_SYNC); /* lost frame sync */
      frame_sync = F_SYNC_TEST-1; /* resync rapidly */
      return;
    }
    buffer(data_word);
    seq = (data_word>>14)&0x7;
    msg_len = (data_word>>9)&0x1f;
#ifdef DEBUG
    if (debug)
	fprintf(stderr, "ff=%d: %08x %08x %d %d %d %d %s\n", frame_fill,
		fillptr,
	data_word, msg_len, word_count, pf_count, seqno, state_name[rtcm_state]);
#endif
    if ((seqno < 0) || (((seqno +1) & 0x7) == seq))
      seqno = seq; /* resync */
    else { /* sequence error */
      state_change(WORD_SYNC); /* to be on the safe side */
#ifdef DEBUG
fprintf(stderr,"2\n");
#endif
      return;
    }
  }
  else if (frame_fill > msg_len) { /* should be next preamble */
#ifdef DEBUG
    if (debug)
	fprintf(stderr, "ff=%d: %08x %08x %d %d %d %d %s\n", frame_fill,
		fillptr,
	data_word, msg_len, word_count, pf_count, seqno, state_name[rtcm_state]);
#endif
    if (rtcm_state == FRAME_SYNCING) { /* be very tough */
      if (!(data_word && preamble())) {
	state_change(WORD_SYNC); /* start again */
      }
      else if (++frame_sync >= F_SYNC_TEST) {
/*	frame_count = 0; */
	state_change(FULL_SYNC);
	new_frame(); /* output the last frame acquired before we start a new one */
      }
      frame_start(); /* new frame here */
    }
    else {
      frame_start(); /* new frame here */
      if (!data_word) /* parity error on preamble - keep sync but lose message */
	pfptr = message; /* indicates dud message */
      else if (!preamble()) { /* good word but no preamble! */
	state_change(WORD_SYNC); /* can't carry on */
      }
    }
  }
  else { /* other message words */
#ifdef DEBUG
    if (debug)
	fprintf(stderr, "ff=%d: %08x %08x %d %d %d %d %s\n", frame_fill,
		fillptr,
	data_word, msg_len, word_count, pf_count, seqno, state_name[rtcm_state]);
#endif
    if (!data_word && !pfptr)
      pfptr = fillptr; /* mark the (first) error */
    buffer(data_word);
    if ((frame_fill == msg_len) && (rtcm_state == FULL_SYNC)) /* frame completed */
      new_frame();
  }
}

void RTCM::status_byte(u_char b) {
#ifdef DEBUG
#if defined(PRINTSTATUS)
    printf("-\tstatus\t-\t%d 0x%x\n", b, b);
#endif
#endif
}


/* take a data byte and process it. This will change state according to
 * the data, place parity-checked data in a buffer and call a function to
 * process completed frames.
 */

void RTCM::data_byte(u_char b) {

  b = reverse_bits[b&0x3f];

  if (rtcm_state == NO_SYNC) {
    if(find_sync(b)) {
      state_change(WORD_SYNCING);
#ifdef DEBUG
      printf("M\tsync_bit: %d\n", sync_bit);
#endif
      word_sync = 1;
      next_word();
    }
  }
  else if (filled_word(b)) {
    switch (rtcm_state) {

    case WORD_SYNCING:
      data_word = parity_ok();
      next_word();
      if (data_word) {
        if (++word_sync >= W_SYNC_TEST) {
	  state_change(WORD_SYNC);
	  p_fail = 0;
	  frame_sync = 1;
	  if (preamble()) { /* just in case we hit one immediately */
	    frame_start();
	    state_change(FRAME_SYNCING);
	  }
	}
      }
      else {
     	if (--word_sync <= 0) {
	  state_change(NO_SYNC);
	  return;
	}
      }
      break;

    case WORD_SYNC: /* look for frame start */
      find_start();
      break;

    case FRAME_SYNCING:
      fill_frame();
      break;

    case FULL_SYNC:
      fill_frame();
      break;
    }
  }
}


void RTCM::new_byte(u_char b) {

  switch (b >> DATA_SHIFT) {

    case 0:
    case 2:
#ifdef DEBUG
	fprintf(stderr, "RTCM 2.x: unknown byte type %d (%d 0x%0x)\n", b >> DATA_SHIFT,
		b, b);
#endif
      return;

    case 3: /* status */
      status_byte(b);
      return;

    case 1: /* data */
      data_byte(b);
      return;
  }
}

int main() {
  int b;

  RTCM decoder = RTCM(280, true);

  while ((b=getchar()) != EOF)
    decoder.new_byte(b);

  printf("M\tword count: %d\tparity failures: %d\tframe count: %d\n",
	  decoder.word_count, decoder.pf_count, decoder.frame_count);

  return 0;
} 
