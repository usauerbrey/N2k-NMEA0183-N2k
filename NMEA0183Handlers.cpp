/* 
NMEA0183Handlers.cpp

2015 Copyright (c) Kave Oy, www.kave.fi  All right reserved.

Author: Timo Lappalainen

  This library is free software; you can redistribute it and/or
  modify it as you like.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
 
#include <N2kMsg.h>
#include <NMEA2000.h>
#include <N2kMessages.h>
#include <NMEA0183Messages.h>
#include "NMEA0183Handlers.h"
#include "AIS.h"

#define pi 3.1415926535897932384626433832795
#define kmhToms 1000.0 / 3600.0
#define knToms 1852.0 / 3600.0
#define degToRad pi / 180.0
#define radToDeg 180.0 / pi
#define msTokmh 3600.0 / 1000.0
#define msTokn 3600.0 / 1852.0
#define nmTom 1.852 * 1000
#define mToFathoms 0.546806649
#define mToFeet 3.2808398950131


extern tNMEA0183 NMEA0183_Out;

struct tNMEA0183Handler {
  const char *Code;
  void (*Handler)(const tNMEA0183Msg &NMEA0183Msg); 
};

// Predefinition for functions to make it possible for constant definition for NMEA0183Handlers
void HandleRMB(const tNMEA0183Msg &NMEA0183Msg);
void HandleRMC(const tNMEA0183Msg &NMEA0183Msg);
void HandleGGA(const tNMEA0183Msg &NMEA0183Msg);
void HandleHDT(const tNMEA0183Msg &NMEA0183Msg);
void HandleVTG(const tNMEA0183Msg &NMEA0183Msg);
void HandleVDM(const tNMEA0183Msg &NMEA0183Msg);
void HandleZTG(const tNMEA0183Msg &NMEA0183Msg);

// Internal variables
tNMEA2000 *pNMEA2000=0;
tBoatData *pBD=0;

//Stream* NMEA0183HandlersDebugStream = 0;
Stream* NMEA0183HandlersDebugStream = &Serial;

tNMEA0183Handler NMEA0183Handlers[]={
  {"GGA",&HandleGGA},
  {"HDT",&HandleHDT},
  {"VTG",&HandleVTG},
  {"RMB",&HandleRMB},
  {"RMC",&HandleRMC},
  {"VDM",&HandleVDM},
  {"VDO",&HandleVDM},
  {"ZTG",&HandleZTG},
  {0,0}
};

/*
Conversions from NMEA 0183 to NMEA 2000
NMEA 0183	NMEA 2000 PGN								Comment
---------------------------------------------------------------------------------------------------------
APB			129283 Cross Track Error					Also used in PGN 129284
DIN			127488 Engine Parameters, Rapid Update		According SeaSmart.Net protocol
			127489 Engine Parameters, Dynamic			specification v1.6.0
			127493 Transmission Parameters, Dynamic
			127505 Fluid Level
			127508 Battery Status
DPT			128267 Water Depth
DTM			129044 Datum
*GGA		129029 GNSS Position Data					ZDA or RMC are required
GLL			129025 Position, Rapid Update				See note (7)
HDG			127250 Vessel Heading
HDM, *HDT	127250 Vessel Heading						Use variation and deviation from HDG
MDA			130311 Environmental Parameters				Relative air humidity, air and water
			130314 Actual Pressure						temperature, atmospheric pressure,
			130306 Wind Data							wind data
MOB			127233 Man Overboard Notification (MOB)
MTW			130311 Environmental Parameters
MWD			130306 Wind Data
MWV			130306 Wind Data							Theoretical wind sent as ground referenced to True North; calculated using COG/SOG
*RMB		129283 Cross Track Error					Use data from APB
			129284 Navigation Data						Sent with true bearings, use ETA from ZTG
			129285 Navigation — Route/WP information
*RMC		126992 System Time							See note (7)
			127258 Magnetic Variation
			129025 Position, Rapid Update
			129026 COG & SOG, Rapid Update
RSA			127245 Rudder
RTE			130066 Route and WP Service —				Use data from WPL
				   Route/WP-List Attributes
			130067 Route and WP Service —
				   Route - WP Name & Position
ROT			127251 Rate of Turn
VHW			128259 Speed, Water referenced
VDR			129291 Set & Drift, Rapid Update
VLW			128275 Distance Log
*VTG		129026 COG & SOG, Rapid Update
VWR			130306 Wind Data
WPL			130074 Route and WP Service —				Only waypoints not included to the route
				   WP List — WP Name & Position			(the RTE should be received during	3 seconds after WPL)
XTE			129283 Cross Track Error
ZDA			126992 System Time
			129033 Local Time Offset
*ZTG		129284 Navigation Data						use ETA from ZTG
VDO, VDM	129038 AIS Class A Position Report			AIS VHF messages 1, 2 and 3
			129039 AIS Class B Position Report			AIS VHF message 18
			129040 AIS Class B Extended Position Report	AIS VHF message 19
			129041 AIS Aids to Navigation (AtoN) Report	AIS VHF message 21
			129793 AIS UTC and Date Report				AIS VHF messages 4 and 11
			129794 AIS Class A Static+Voyage Rel Data	AIS VHF message 5
			129798 AIS SAR Aircraft Position Report		AIS VHF message 9
			129809 AIS Class B "CS" Static Data, Part A	AIS VHF message 24
			129810 AIS Class B "CS" Static Data, Part B	AIS VHF message 24

Note (7): All NMEA 2000 periodic messages are sending with interval specified in the Standard.
Except PGN 127488, 127489, 127493, 127505 and 127508, these messages are sending
immediately on receiving of DIN sentence.
Note (8): Sentences with no significant data (or data marked as invalid) may not be translated to
NMEA 2000 messages. NMEA 0183 sentences with invalid check sum are ignored.

* NMEA 0183 currently suported

*/
// *****************************************************************************
void InitNMEA0183Handlers(tNMEA2000 *_NMEA2000, tBoatData *_BoatData) {
  pNMEA2000=_NMEA2000;
  pBD=_BoatData;
}

// *****************************************************************************
tN2kGNSSmethod GNSMethofNMEA0183ToN2k(int Method) {
  switch (Method) {
    case 0: return N2kGNSSm_noGNSS;
    case 1: return N2kGNSSm_GNSSfix;
    case 2: return N2kGNSSm_DGNSS;
    default: return N2kGNSSm_noGNSS;  
  }
}

// *****************************************************************************
void HandleNMEA0183Msg(const tNMEA0183Msg &NMEA0183Msg) {
  int iHandler;
  // Find handler
  for (iHandler=0; NMEA0183Handlers[iHandler].Code!=0 && !NMEA0183Msg.IsMessageCode(NMEA0183Handlers[iHandler].Code); iHandler++);
  
  if (NMEA0183Handlers[iHandler].Code!=0) {
//	  if (NMEA0183HandlersDebugStream != 0) { NMEA0183HandlersDebugStream->print("NMEA0183 message parsed: "); NMEA0183HandlersDebugStream->println(NMEA0183Handlers[iHandler].Code); }
	  NMEA0183Handlers[iHandler].Handler(NMEA0183Msg);
  }
  
  // Forward all received NMEA0183 messages to the NMEA0183 out stream
  // NMEA0183_Out.SendMessage(NMEA0183Msg);
}

// NMEA0183 message Handler functions

/*****************************************************************************
RMB - Recommended Minimum Navigation Information
															14
	   1 2   3 4    5    6       7 8        9 10  11  12  13|
	   | |   | |    |    |       | |        | |   |   |   | |
$--RMB,A,x.x,a,c--c,c--c,llll.ll,a,yyyyy.yy,a,x.x,x.x,x.x,A*hh<CR><LF>
Field Number :
1) Status, V = Navigation receiver warning
2) Cross Track error - nautical miles
3) Direction to Steer, Left or Right
4) FROM Waypoint ID
5) TO Waypoint ID
6) Destination Waypoint Latitude
7) N or S
8) Destination Waypoint Longitude
9) E or W
10) Range to destination in nautical miles
11) Bearing to destination in degrees True
12) Destination closing velocity in knots
13) Arrival Status, A = Arrival Circle Entered
14) Checksum

*/
void HandleRMB(const tNMEA0183Msg &NMEA0183Msg) {
	double XTE;
	double Latitude;
	double Longitude;
	double DTW;
	double BTW;
	double VMG;
	char arrivalAlarm;
	char originID[NMEA0183_MAX_WP_NAME_LENGTH];
	char destID[NMEA0183_MAX_WP_NAME_LENGTH];
	
//	if (NMEA0183HandlersDebugStream != 0) { NMEA0183HandlersDebugStream->print("NMEA0183Msg="); NMEA0183HandlersDebugStream->println(NMEA0183Msg->Data[MAX_NMEA0183_MSG_LEN]); }

	if (pBD == 0) return;

	if (NMEA0183ParseRMB_nc(NMEA0183Msg, XTE, Latitude, Longitude, DTW, BTW, VMG, arrivalAlarm, *originID, *destID)) {
		if (NMEA0183HandlersDebugStream != 0) {
			NMEA0183HandlersDebugStream->print("XTE="); NMEA0183HandlersDebugStream->println(XTE);
			NMEA0183HandlersDebugStream->print("Latitude="); NMEA0183HandlersDebugStream->println(Latitude);
			NMEA0183HandlersDebugStream->print("Longitude="); NMEA0183HandlersDebugStream->println(Longitude);
			NMEA0183HandlersDebugStream->print("DTW="); NMEA0183HandlersDebugStream->println(DTW);
			NMEA0183HandlersDebugStream->print("BTW="); NMEA0183HandlersDebugStream->println(BTW);
			NMEA0183HandlersDebugStream->print("VMG="); NMEA0183HandlersDebugStream->println(VMG);
			NMEA0183HandlersDebugStream->print("arrivalAlarm="); NMEA0183HandlersDebugStream->println(arrivalAlarm);
			NMEA0183HandlersDebugStream->print("originID="); NMEA0183HandlersDebugStream->println(originID);
			NMEA0183HandlersDebugStream->print("destID="); NMEA0183HandlersDebugStream->println(destID);
		}
/*
RMB			129283 Cross Track Error
			129284 Navigation Data
			129285 Navigation — Route / WP information
*/
			if (pNMEA2000 != 0) {
			tN2kMsg N2kMsg;
			SetN2kXTE(N2kMsg, 1, N2kxtem_Autonomous /*XTEMode*/, false /*NavigationTerminated*/, XTE*nmTom);
			pNMEA2000->SendMsg(N2kMsg);
            SetN2kNavigationInfo(N2kMsg, 1, DTW*nmTom, N2khr_true, false, false, N2kdct_GreatCircle,
				N2kDoubleNA, N2kInt16NA, /* double ETATime, int16_t ETADate */ 0, /* double BearingToOriginal */ 
				BTW*degToRad, N2kUInt8NA, N2kUInt8NA, /* uint8_t OriginWaypointNumber, uint8_t DestinationWaypointNumber*/ Latitude, Longitude, VMG*knToms);
			pNMEA2000->SendMsg(N2kMsg);
/*
'{"timestamp":"2017-04-15T21:50:57.780Z","prio":6,"pgn":129285,"src":3,"dst":255,"fields":{"Start RPS#":12,"nItems":3,"Database ID":0,"Route ID":0,
"Navigation direction in route":3,"Supplementary Route/WP data available":1,"Route Name":"Route","list":[{"WP ID":12,"WP Name":"Waypoint 240","WP Latitude":39.1525868,
"WP Longitude":-76.1811988},{"WP ID":13,"WP Name":"Waypoint 241","WP Latitude":39.1568741,"WP Longitude":-76.1834715},{"WP ID":14,"WP Name":"Waypoint 242",
"WP Latitude":39.159168,"WP Longitude":-76.182178}]},"description":"Navigation - Route/WP Information"}'
			SetN2kPGN129285(N2kMsg, uint16_t Start, uint16_t Database, uint16_t Route,
				bool NavDirection, bool SupplementaryData, char* RouteName);

			SetN2kPGN129285(N2kMsg, 12, 0, 0, true, true, "13Waypoint 24139.1568741-76.1834715");
			pNMEA2000->SendMsg(N2kMsg);
*/
			}

		if (NMEA0183HandlersDebugStream != 0) {
		}
	}
	else if (NMEA0183HandlersDebugStream != 0) { NMEA0183HandlersDebugStream->println("Failed to parse RMB"); }
}

/*****************************************************************************
RMC - Recommended Minimum Navigation Information
		1         2 3       4 5        6 7   8   9    10 11 12
		|         | |       | |        | |   |   |    |   | |
 $--RMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,xxxx,x.x,a*hh<CR><LF>
 Field Number:
  1) UTC Time
  2) Status, V = Navigation receiver warning, P = Precise
  3) Latitude
  4) N or S
  5) Longitude
  6) E or W
  7) Speed over ground, knots
  8) Track made good, degrees true
  9) Date, ddmmyy
 10) Magnetic Variation, degrees
 11) E or W
 12) Checksum

*/

void HandleRMC(const tNMEA0183Msg &NMEA0183Msg) {
  if (pBD==0) return;
  
  if (NMEA0183ParseRMC_nc(NMEA0183Msg, pBD->GPSTime, pBD->Latitude, pBD->Longitude, pBD->COG, pBD->SOG, pBD->DaysSince1970, pBD->Variation)) {
/*
RMC		  126992 System Time
		  127258 Magnetic Variation
		  129025 Position, Rapid Update
		  129026 COG & SOG, Rapid Update   fehlt
*/
		  if (pNMEA2000 != 0) {
		  tN2kMsg N2kMsg;
		  SetN2kMagneticVariation(N2kMsg, 1, N2kmagvar_Calc, pBD->DaysSince1970, pBD->Variation);
		  pNMEA2000->SendMsg(N2kMsg);
		  SetN2kPGN129025(N2kMsg, pBD->Latitude, pBD->Longitude);
		  pNMEA2000->SendMsg(N2kMsg);
		  SetN2kCOGSOGRapid(N2kMsg, 1, N2khr_true, pBD->COG, pBD->SOG);
		  pNMEA2000->SendMsg(N2kMsg);
		  }

	  if (NMEA0183HandlersDebugStream != 0) {
	  }
  }
  else if (NMEA0183HandlersDebugStream != 0) { NMEA0183HandlersDebugStream->println("Failed to parse RMC"); }
}

/*****************************************************************************
GGA - Global Positioning System Fix Data, Time, Position and fix related data fora GPS receiver.
													  11
		1         2       3 4        5 6 7  8   9  10 |  12 13  14   15
		|         |       | |        | | |  |   |   | |   | |   |    |
 $--GGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh<CR><LF>
 Field Number:
  1) Universal Time Coordinated (UTC)
  2) Latitude
  3) N or S (North or South)
  4) Longitude
  5) E or W (East or West)
  6) GPS Quality Indicator,
	 0 - fix not available,
	 1 - GPS fix,
	 2 - Differential GPS fix
  7) Number of satellites in view, 00 - 12
  8) Horizontal Dilution of precision
  9) Antenna Altitude above/below mean-sea-level (geoid)
 10) Units of antenna altitude, meters
 11) Geoidal separation, the difference between the WGS-84 earth
	 ellipsoid and mean-sea-level (geoid), "-" means mean-sea-level
	 below ellipsoid
 12) Units of geoidal separation, meters
 13) Age of differential GPS data, time in seconds since last SC104
	 type 1 or 9 update, null field when DGPS is not used
 14) Differential reference station ID, 0000-1023
 15) Checksum

*/

void HandleGGA(const tNMEA0183Msg &NMEA0183Msg) {
  if (pBD==0) return;
  
  if (NMEA0183ParseGGA_nc(NMEA0183Msg,pBD->GPSTime,pBD->Latitude,pBD->Longitude,
                   pBD->GPSQualityIndicator,pBD->SatelliteCount,pBD->HDOP,pBD->Altitude,pBD->GeoidalSeparation,
                   pBD->DGPSAge,pBD->DGPSReferenceStationID)) {
/*
GGA		129029 GNSS Position Data
*/
	  if (pNMEA2000!=0) {
      tN2kMsg N2kMsg;
      SetN2kMagneticVariation(N2kMsg,1,N2kmagvar_Calc,pBD->DaysSince1970,pBD->Variation);
      pNMEA2000->SendMsg(N2kMsg); 
      SetN2kGNSS(N2kMsg,1,pBD->DaysSince1970,pBD->GPSTime,pBD->Latitude,pBD->Longitude,pBD->Altitude,
                N2kGNSSt_GPS,GNSMethofNMEA0183ToN2k(pBD->GPSQualityIndicator),pBD->SatelliteCount,pBD->HDOP,0,
                pBD->GeoidalSeparation,1,N2kGNSSt_GPS,pBD->DGPSReferenceStationID,pBD->DGPSAge
                );
      pNMEA2000->SendMsg(N2kMsg); 
    }

    if (NMEA0183HandlersDebugStream!=0) {
      NMEA0183HandlersDebugStream->print("Time="); NMEA0183HandlersDebugStream->println(pBD->GPSTime);
      NMEA0183HandlersDebugStream->print("Latitude="); NMEA0183HandlersDebugStream->println(pBD->Latitude,5);
      NMEA0183HandlersDebugStream->print("Longitude="); NMEA0183HandlersDebugStream->println(pBD->Longitude,5);
      NMEA0183HandlersDebugStream->print("Altitude="); NMEA0183HandlersDebugStream->println(pBD->Altitude,1);
      NMEA0183HandlersDebugStream->print("GPSQualityIndicator="); NMEA0183HandlersDebugStream->println(pBD->GPSQualityIndicator);
      NMEA0183HandlersDebugStream->print("SatelliteCount="); NMEA0183HandlersDebugStream->println(pBD->SatelliteCount);
      NMEA0183HandlersDebugStream->print("HDOP="); NMEA0183HandlersDebugStream->println(pBD->HDOP);
      NMEA0183HandlersDebugStream->print("GeoidalSeparation="); NMEA0183HandlersDebugStream->println(pBD->GeoidalSeparation);
      NMEA0183HandlersDebugStream->print("DGPSAge="); NMEA0183HandlersDebugStream->println(pBD->DGPSAge);
      NMEA0183HandlersDebugStream->print("DGPSReferenceStationID="); NMEA0183HandlersDebugStream->println(pBD->DGPSReferenceStationID);
    }
  } else if (NMEA0183HandlersDebugStream!=0) { NMEA0183HandlersDebugStream->println("Failed to parse GGA"); }
}

#define PI_2 6.283185307179586476925286766559

/*****************************************************************************
HDT - Heading - True
		1   2 3
		|   | |
 $--HDT,x.x,T*hh<CR><LF>
 Field Number:
  1) Heading Degrees, true
  2) T = True
  3) Checksum

*/
void HandleHDT(const tNMEA0183Msg &NMEA0183Msg) {
  if (pBD==0) return;
  
  if (NMEA0183ParseHDT_nc(NMEA0183Msg,pBD->TrueHeading)) {
/*
HDT		127250 Vessel Heading
*/
	  if (pNMEA2000!=0) {
      tN2kMsg N2kMsg;
      double MHeading=pBD->TrueHeading-pBD->Variation;
      while (MHeading<0) MHeading+=PI_2;
      while (MHeading>=PI_2) MHeading-=PI_2;
      SetN2kTrueHeading(N2kMsg,1,pBD->TrueHeading);
      pNMEA2000->SendMsg(N2kMsg);
    }
    if (NMEA0183HandlersDebugStream!=0) {
      NMEA0183HandlersDebugStream->print("True heading="); NMEA0183HandlersDebugStream->println(pBD->TrueHeading);
    }
  } else if (NMEA0183HandlersDebugStream!=0) { NMEA0183HandlersDebugStream->println("Failed to parse HDT"); }
}

/*****************************************************************************
VTG - Track made good and Ground speed
		1   2 3   4 5  6 7   8 9
		|   | |   | |  | |   | |
 $--VTG,x.x,T,x.x,M,x.x,N,x.x,K*hh<CR><LF>
 Field Number:
  1) Track Degrees
  2) T = True
  3) Track Degrees
  4) M = Magnetic
  5) Speed Knots
  6) N = Knots
  7) Speed Kilometers Per Hour
  8) K = Kilometers Per Hour
  9) Checksum

*/
void HandleVTG(const tNMEA0183Msg &NMEA0183Msg) {
  double MagneticCOG;
  if (pBD==0) return;
  
  if (NMEA0183ParseVTG_nc(NMEA0183Msg,pBD->COG,MagneticCOG,pBD->SOG)) {
      pBD->Variation=pBD->COG-MagneticCOG; // Save variation for Magnetic heading
/*
VTG		129026 COG & SOG, Rapid Update
*/
	  if (pNMEA2000!=0) { 
      tN2kMsg N2kMsg;
      SetN2kCOGSOGRapid(N2kMsg,1,N2khr_true,pBD->COG,pBD->SOG);
      pNMEA2000->SendMsg(N2kMsg);
      SetN2kBoatSpeed(N2kMsg,1,pBD->SOG);
      pNMEA2000->SendMsg(N2kMsg);
    }
    if (NMEA0183HandlersDebugStream!=0) {
      NMEA0183HandlersDebugStream->print("True heading="); NMEA0183HandlersDebugStream->println(pBD->TrueHeading);
    }
  } else if (NMEA0183HandlersDebugStream!=0) { NMEA0183HandlersDebugStream->println("Failed to parse VTG"); }
}

/*****************************************************************************
ZTG - UTC & Time to Destination Waypoint
		1         2         3    4
		|         |         |    |
 $--ZTG,hhmmss.ss,hhmmss.ss,c--c*hh<CR><LF>
 Field Number:
  1) Universal Time Coordinated (UTC)
  2) Time Remaining
  3) Destination Waypoint ID
  4) Checksum

*/
void HandleZTG(const tNMEA0183Msg &NMEA0183Msg) {
	double MagneticCOG;
	if (pBD == 0) return;

	if (NMEA0183ParseVTG_nc(NMEA0183Msg, pBD->COG, MagneticCOG, pBD->SOG)) {
		pBD->Variation = pBD->COG - MagneticCOG; // Save variation for Magnetic heading
 		if (pNMEA2000 != 0) {
			tN2kMsg N2kMsg;
//			SetN2kCOGSOGRapid(N2kMsg, 1, N2khr_true, pBD->COG, pBD->SOG);
//			pNMEA2000->SendMsg(N2kMsg);
//			SetN2kBoatSpeed(N2kMsg, 1, pBD->SOG);
//			pNMEA2000->SendMsg(N2kMsg);
		}
		if (NMEA0183HandlersDebugStream != 0) {
			NMEA0183HandlersDebugStream->print("True heading="); NMEA0183HandlersDebugStream->println(pBD->TrueHeading);
		}
	}
	else if (NMEA0183HandlersDebugStream != 0) { NMEA0183HandlersDebugStream->println("Failed to parse ZTG"); }
}

void printDegrees(long min4)
{
	long intPart = min4 / 60L;
	long fracPart = intPart % 10000L;
	if (fracPart < 0)
		fracPart = -fracPart;
	char frac[6];
	sprintf(frac, "%04ld", fracPart);
	NMEA0183HandlersDebugStream->print(intPart / 10000L); NMEA0183HandlersDebugStream->print("."); NMEA0183HandlersDebugStream->println(frac);
}

/*****************************************************************************
!AIVDM/VDO - AIVDM packets are reports from other ships and AIVDO packets are reports from your own ship
		1 2 3 4 5    6 7
		| | | | |    | |
 !AIVDM,n,n,n,c,c--c,n*hh<CR><LF>
 Field Number:
  1) count of fragments in the currently accumulating message. 
     The payload size of each sentence is limited by NMEA 0183’s 82-character maximum, 
	 so it is sometimes required to split a payload over several fragment sentences.
  2) fragment number of this sentence. It will be one-based. 
     A sentence with a fragment count of 1 and a fragment number of 1 is complete in itself.
  3) sequential message ID for multi-sentence messages
  4) radio channel code. AIS uses the high side of the duplex from two VHF radio channels:
     AIS Channel A is 161.975Mhz (87B); AIS Channel B is 162.025Mhz (88B). In the wild, channel codes 1 and 2 may also be encountered;
	 the standards do not prescribe an interpretation of these but it’s obvious enough.
  5) data payload
  6) number of fill bits requires to pad the data payload to a 6 bit boundary, ranging from 0 to 5.
     Equivalently, subtracting 5 from this tells how many least significant bits of the last 6-bit nibble in the data payload should be ignored.
	 Note that this pad byte has a tricky interaction with the <[ITU-1371]> requirement for byte alignment in over-the-air AIS messages.
  7) Checksum

*/
/*
!AIVDM,1,1,,B,177KQJ5000G?tO`K>RA1wUbN0TKH,0*5C
!AIVDM,1,1,,A,13u?etPv2;0n:dDPwUM1U1Cb069D,0*23
!AIVDM,1,1,,A,400TcdiuiT7VDR>3nIfr6>i00000,0*78
!AIVDM,2,1,0,A,58wt8Ui`g??r21`7S=:22058<v05Htp000000015>8OA;0sk,0*7B
!AIVDM,2,2,0,A,eQ8823mDm3kP00000000000,2*5D
!AIVDM,1,1,4,B,6>jR0600V:C0>da4P106P00,2*02
!AIVDM,2,1,9,B,61c2;qLPH1m@wsm6ARhp<ji6ATHd<C8f=Bhk>34k;S8i=3To,0*2C
!AIVDM,2,2,9,B,Djhi=3Di<2pp=34k>4D,2*03
!AIVDM,1,1,1,B,8>h8nkP0Glr=<hFI0D6??wvlFR06EuOwgwl?wnSwe7wvlOw?sAwwnSGmwvh0,0*17
*/
void HandleVDM(const tNMEA0183Msg &NMEA0183Msg) {
	uint8_t pkgCnt;
	uint8_t pkgNmb;
	unsigned int seqMessageId;
	char channel; 
	unsigned int length;
	char bitstream[60];
	unsigned int fillBits;

	uint8_t msgType;
	unsigned int msgNumeric;
	uint8_t repeat;
	unsigned long mmsi;
	long LAT;
	long LONG;
	bool accuracy;
	bool raim;
	uint8_t seconds;
	unsigned int COG;
	unsigned int SOG;
	double HDG;
	double ROT;
	uint8_t navstatus;
	char shipname[20];
	char *name ="          ";

	length = 40;

	if (NMEA0183ParseVDM_nc(NMEA0183Msg, pkgCnt, pkgNmb, seqMessageId, channel, length, bitstream, fillBits)) {
		if (NMEA0183HandlersDebugStream != 0) {
			// NMEA0183HandlersDebugStream->print("pkgCnt="); NMEA0183HandlersDebugStream->println(pkgCnt);
			// NMEA0183HandlersDebugStream->print("pkgNmb="); NMEA0183HandlersDebugStream->println(pkgNmb);
			// NMEA0183HandlersDebugStream->print("seqMessageId="); NMEA0183HandlersDebugStream->println(seqMessageId);
			// NMEA0183HandlersDebugStream->print("channel="); NMEA0183HandlersDebugStream->println(channel);
			// NMEA0183HandlersDebugStream->print("length="); NMEA0183HandlersDebugStream->println(length);
			// NMEA0183HandlersDebugStream->print("bitstream="); NMEA0183HandlersDebugStream->println(bitstream);
			// NMEA0183HandlersDebugStream->print("fillBits="); NMEA0183HandlersDebugStream->println(fillBits);
		}

		/*
		VDM/VDO		*129038	AIS Class A Position Report						AIS VHF message 1
					*129038	AIS Class A Position Report						AIS VHF message 2
					*129038	AIS Class A Position Report						AIS VHF message 3
					 129793	AIS UTC and Date Report							AIS VHF message 4
					*129794	AIS Class A Static and Voyage Rel Data			AIS VHF message 5
					 129795 AIS Addressed Binary Message					AIS VHF message 6
					 129796 AIS Acknowledge									AIS VHF message 7
 					 129797 AIS Binary Broadcast Message					AIS VHF message 8
					 129798	AIS SAR Aircraft Position Report				AIS VHF message 9
					 129800 AIS UTC/Date Inquiry							AIS VHF message 10
					 129793	AIS UTC and Date Report							AIS VHF message 11
					 129801 AIS Addressed Safety Related Message			AIS VHF message 12
							AIS	Safety related acknowledgement				AIS VHF message 13
					 129802 AIS Safety Related Broadcast Message			AIS VHF message 14
					 129803 AIS Interrogation								AIS VHF message 15
					 129804 AIS Assignment Mode Command						AIS VHF message 16
					 129792 AIS DGNSS Broadcast Binary Message				AIS VHF message 17
					*129039	AIS Class B Position Report						AIS VHF message 18
					 129040	AIS Class B Extended Position Report			AIS VHF message 19
					 129805 AIS Data Link Management Message				AIS VHF message 20
					 129041	AIS Aids to Navigation (AtoN) Report			AIS VHF message 21
							AIS	Channel management							AIS VHF message 22
					 129807 AIS Class B Group Assignment					AIS VHF message 23
					*129809	AIS Class B "CS" Static Data, Part A			AIS VHF message 24
					*129810	AIS Class B "CS" Static Data, Part B			AIS VHF message 24
							AIS	Single slot binary message					AIS VHF message 25
							AIS	Multiple slot binary msg with Comm State	AIS VHF message 26
							AIS Long-Range Broadcast						AIS VHF message 27
		*/

		if (pNMEA2000 != 0) {
			tN2kMsg N2kMsg;
			AIS ais_msg(bitstream, fillBits);

			msgType = ais_msg.get_type();
			msgNumeric = ais_msg.get_numeric_type();
			repeat = ais_msg.get_repeat();
			mmsi = ais_msg.get_mmsi();
			LAT = ais_msg.get_latitude();
			LONG = ais_msg.get_longitude();
			accuracy = ais_msg.get_posAccuracy_flag();
			raim = ais_msg.get_raim_flag();
			seconds = ais_msg.get_timeStamp();
			COG = ais_msg.get_COG();
			SOG = ais_msg.get_SOG();
			HDG = ais_msg.get_HDG();
			ROT = ais_msg.get_rot();
			navstatus = ais_msg.get_navStatus();
//			strncpy(shipname, ais_msg.get_shipname(), 20);
			name = ais_msg.get_shipname();
//			Serial.print("name:"); Serial.println(name);

			// Multi packets not supported yet
			if (pkgCnt != 1) {
				if (NMEA0183HandlersDebugStream != 0) {
					NMEA0183HandlersDebugStream->print("Failed to parse VDM/VDO, pkgCnt=");
					NMEA0183HandlersDebugStream->print(pkgCnt);
					NMEA0183HandlersDebugStream->print(", msgNumeric=");
					NMEA0183HandlersDebugStream->println(msgNumeric);
				}
				return;
			}


/*
			if (NMEA0183HandlersDebugStream != 0) {
				NMEA0183HandlersDebugStream->print("msgType="); NMEA0183HandlersDebugStream->println(msgType);
				NMEA0183HandlersDebugStream->print("msgNumeric="); NMEA0183HandlersDebugStream->println(msgNumeric);
				NMEA0183HandlersDebugStream->print("repeat="); NMEA0183HandlersDebugStream->println(repeat);
				NMEA0183HandlersDebugStream->print("mmsi="); NMEA0183HandlersDebugStream->println(mmsi);
				NMEA0183HandlersDebugStream->print("LAT="); printDegrees(LAT);
				NMEA0183HandlersDebugStream->print("LONG="); printDegrees(LONG);
				NMEA0183HandlersDebugStream->print("accuracy="); NMEA0183HandlersDebugStream->println(accuracy);
				NMEA0183HandlersDebugStream->print("raim="); NMEA0183HandlersDebugStream->println(raim);
				NMEA0183HandlersDebugStream->print("seconds="); NMEA0183HandlersDebugStream->println(seconds);
				NMEA0183HandlersDebugStream->print("COG="); NMEA0183HandlersDebugStream->print(COG/10); NMEA0183HandlersDebugStream->print("."); NMEA0183HandlersDebugStream->println(COG % 10);
				NMEA0183HandlersDebugStream->print("SOG="); NMEA0183HandlersDebugStream->print(SOG/10); NMEA0183HandlersDebugStream->print("."); NMEA0183HandlersDebugStream->println(SOG % 10);
				NMEA0183HandlersDebugStream->print("HDG="); NMEA0183HandlersDebugStream->println(HDG);
				NMEA0183HandlersDebugStream->print("ROT="); NMEA0183HandlersDebugStream->println(ROT);
				NMEA0183HandlersDebugStream->print("navstatus="); NMEA0183HandlersDebugStream->println(navstatus);
				NMEA0183HandlersDebugStream->print("shipname="); NMEA0183HandlersDebugStream->println(shipname);
			}
*/

			if ((msgNumeric == 1) || (msgNumeric == 2) || (msgNumeric == 3)) {
				// 129038 AIS Class A Position Report			AIS VHF messages 1, 2 and 3
//				if (NMEA0183HandlersDebugStream != 0) { NMEA0183HandlersDebugStream->print("msgNumeric="); NMEA0183HandlersDebugStream->println(msgNumeric); }

				SetN2kPGN129038(N2kMsg, seqMessageId, static_cast<tN2kAISRepeat>(repeat), mmsi, LAT/600000L, LONG/600000L,
					accuracy, raim, seconds, COG*degToRad/10, SOG*knToms/10, HDG*degToRad/10, ROT*degToRad/10, static_cast<tN2kAISNavStatus>(navstatus));
				pNMEA2000->SendMsg(N2kMsg);
			}
			else if (msgNumeric == 5) {
				// 129794 AIS Class A Static+Voyage Rel Data	AIS VHF message 5
				if (NMEA0183HandlersDebugStream != 0) { NMEA0183HandlersDebugStream->print("msgNumeric="); NMEA0183HandlersDebugStream->println(msgNumeric); }

				SetN2kPGN129794(N2kMsg, seqMessageId, static_cast<tN2kAISRepeat>(repeat), mmsi, 0, /*uint32_t IMOnumber,*/ "DOLD", /*char *Callsign,*/ "test", /*char *Name,*/
					70, /*uint8_t VesselType,*/ 100, /*double Length,*/	12, /*double Beam,*/ 2, /*double PosRefStbd,*/ 3, /*double PosRefBow,*/ 0, /*uint16_t ETAdate,*/ 0, /*double ETAtime,*/
					4, /*double Draught,*/ "Athen", /*char *Destination,*/ N2kaisv_ITU_R_M_1371_1, /*tN2kAISVersion AISversion,*/ N2kGNSSt_GPS, /*tN2kGNSStype GNSStype,*/
					N2kaisdte_Ready, /*tN2kAISDTE DTE,*/ N2kaisti_Channel_A_VDL_reception /*tN2kAISTranceiverInfo AISinfo*/);
				pNMEA2000->SendMsg(N2kMsg);
			}
			else if (msgNumeric == 18) {
				// 129039 AIS Class B Position Report			AIS VHF message 18
				if (NMEA0183HandlersDebugStream != 0) { NMEA0183HandlersDebugStream->print("msgNumeric="); NMEA0183HandlersDebugStream->println(msgNumeric); }

				SetN2kPGN129039(N2kMsg, seqMessageId, static_cast<tN2kAISRepeat>(repeat), mmsi, /* uint32_t UserID*/ LAT/600000L, LONG/600000L,
					accuracy, raim, seconds, COG*degToRad/10, SOG*knToms/10, HDG, (tN2kAISUnit)0, /* tN2kAISUnit Unit*/ false, false, false, false, /* bool Display, bool DSC, bool Band, bool Msg22*/
					N2kaismode_Autonomous, /*tN2kAISMode Mode*/ false /* bool State*/);
				pNMEA2000->SendMsg(N2kMsg);
			}
			else if (msgNumeric == 24) {
				// 129809 AIS Class B "CS" Static Data, Part A	AIS VHF message 24
				// 129810 AIS Class B "CS" Static Data, Part B	AIS VHF message 24
				if (NMEA0183HandlersDebugStream != 0) { NMEA0183HandlersDebugStream->print("msgNumeric="); NMEA0183HandlersDebugStream->println(msgNumeric); }

				SetN2kPGN129809(N2kMsg, msgType, static_cast<tN2kAISRepeat>(repeat), mmsi, "Test" /* char *Name */);
				pNMEA2000->SendMsg(N2kMsg);
				SetN2kPGN129810(N2kMsg, msgType, static_cast<tN2kAISRepeat>(repeat), mmsi, 0, /*uint8_t VesselType,*/ "em-trak A100", /*char *Vendor,*/ "DOLD", /*char *Callsign,*/ 
					13, 4, /*double Length, double Beam,*/ 0, /*double PosRefStbd,*/ 0, /*double PosRefBow,*/ 0 /*uint32_t MothershipID*/);
				pNMEA2000->SendMsg(N2kMsg);
			}
			else if (NMEA0183HandlersDebugStream != 0) { NMEA0183HandlersDebugStream->print("msgNumeric, no N2K msg="); NMEA0183HandlersDebugStream->println(msgNumeric); }
		}
	}
	else if (NMEA0183HandlersDebugStream != 0) { NMEA0183HandlersDebugStream->println("Failed to parse VDM/VDO"); }
}