/*
N2kDataToNMEA0183.cpp

Copyright (c) 2015-2018 Timo Lappalainen, Kave Oy, www.kave.fi

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "N2kDataToNMEA0183.h"
#include <N2kMessages.h>
#include <NMEA0183Messages.h>

Stream* N2kHandlersDebugStream = 0;
//Stream* N2kHandlersDebugStream = &Serial;


//*****************************************************************************
double DoubleToddmm(double val) {
	if (val != NMEA0183DoubleNA) {
		double intpart;
		val = modf(val, &intpart);
		val = intpart * 100 + val * 60;
	}

	return val;
}

void tN2kDataToNMEA0183::HandleMsg(const tN2kMsg &N2kMsg) {
  if (N2kHandlersDebugStream != 0) { N2kHandlersDebugStream->print("N2k message parsed: "); N2kHandlersDebugStream->println(N2kMsg.PGN); }
/*
  switch (N2kMsg.PGN) {
    case 127250UL: HandleHeading(N2kMsg);    // -> HDG
    case 127258UL: HandleVariation(N2kMsg);  // store variation
    case 128259UL: HandleBoatSpeed(N2kMsg);  // -> VHW
    case 128267UL: HandleDepth(N2kMsg);      // -> DBT
    case 129025UL: HandlePosition(N2kMsg);   // -> GLL
    case 129026UL: HandleCOGSOG(N2kMsg);     // -> VTG
//    case 129029UL: HandleGNSS(N2kMsg);       // -> GGA, GLL
	case 130306UL: HandleWindSpeed(N2kMsg);  // -> MWV
  }
*/
}

/*
Conversions Between NMEA 2000 and NMEA 0183

NMEA 2000 PGN											NMEA 0183						Comment
----------------------------------------------------------------------------------------------------------------------------
126992 System Time										ZDA, GLL						See also PGN 129033
127233 Man Overboard Notification (MOB)					MOB
127237 Heading/Track Control							APB								Use PGN 129284, 129283 if possible
127245 Rudder											RSA								Two rudders supported
*127250 Vessel Heading									HDG, HDM, HDT					See note (4) => HDG
127251 Rate of Turn										ROT
*127258 Magnetic Variation — See note (4)
127488 Engine Parameters, Rapid Update					RPM, XDR, DIN, PGN				See note (6)
127489 Engine Parameters, Dynamic						XDR, DIN						See note (6)
127493 Transmission Parameters, Dynamic					DIN, PGN						See note (6)
127505 Fluid Level										DIN, PGN						See note (6)
127508 Battery Status									DIN, PGN						See note (6)
*128259 Speed, Water referenced							VHW								Also may be used in RMC, VTG => VHW
*128267 Water Depth										DBT, DBS, DPT					DBS, DPT are off in factory settings => DBT
128275 Distance Log										VLW
*129025 Position, Rapid Update							GLL								Also use PGN 126992 or 129029
*129026 COG & SOG, Rapid Update							VTG								Also used in RMC => VTG
*129029 GNSS Position Data								GGA, GLL, RMC, ZDA				See also PGN 129033  => GGA, RMC
129033 Local Time Offset								—								Time offset is used in ZDA
129044 Datum											DTM
129283 Cross Track Error								XTE
129284 Navigation Data									RMB								Use 129283, 129029 if possible
129285 Navigation — Route/WP information				—								Waypoint names from this message are used in RMB and APB sentences
129291 Set & Drift, Rapid Update						VDR
129539 GNSS DOPs										GSA								PGN 129540 is also required
129540 GNSS Sats in View								GSV
130066 Route and WP Service — Route/WP, List Attributes	RTE								Use waypoints from 130067
130067 Route and WP Service — Route, WP Name & Position WPL
130074 Route and WP Service — WP List, WP Name&Position WPL
*130306 Wind Data										MWD, MWV						See note (3). Also used in MDA. => MWV
130310 Environmental Parameters							XDR, MTW, MDA					See note (1), (5)
130311 Environmental Parameters							XDR, MTW, MDA					See notes (1), (2), (5)
130312 Temperature										XDR, MTW, MDA					See notes (1), (2), (5)
130313 Humidity											XDR, MDA						See notes (1), (2), (5)
130314 Actual Pressure									XDR, MDA						See notes (1), (2), (5)
130316 Temperature, Extended Range						XDR, MTW, MDA					See notes (1), (2), (5)
129038 AIS Class A Position Report						VDM, VDO						AIS VHF messages 1, 2 and 3
129039 AIS Class B Position Report						VDM, VDO						AIS VHF message 18
129040 AIS Class B Extended Position Report				VDM, VDO						AIS VHF message 19
129041 AIS Aids to Navigation (AtoN) Report				VDM, VDO						AIS VHF message 21
129793 AIS UTC and Date Report							VDM, VDO						AIS VHF messages 4 and 11
129794 AIS Class A Static and Voyage Related Data		VDM, VDO						AIS VHF message 5
129798 AIS SAR Aircraft Position Report					VDM, VDO						AIS VHF message 9
129809 AIS Class B "CS" Static Data Report, Part A		VDM, VDO						AIS VHF message 24
129810 AIS Class B "CS" Static Data Report, Part B		VDM, VDO						AIS VHF message 24

Note (1): Air, dew point, inside (saloon), water and exhaust gas temperature, inside and outside humidity, barometric pressure are supported.
Note (2): Only messages with data instance 0 are converted.
Note (3): Devices with factory settings perform conversion from true to apparent wind. The MWV sentence is sent twice (one for apparent wind and one for true). See VI.11 for details.
Note (4): Magnetic variation is used in RMC, HDT, HDG, VDR, VHW, VTG. Priority of variation PGNs: 127250, 127258, 65311.
Note (5): MDA is sent only when air, dew point or water temperature, or barometric pressure or outside humidity are available. Also contains wind speed and direction.
Note (6): DIN and PGN are wrap NMEA 2000 messages according SeaSmart (v1.6.0) and MiniPlex (v2.0) specifications. Engine revolutions, boost pressure, coolant temperature, hours, fuel rate, 
          alternator voltage are also transmitted in XDR sentence. DIN, PGN and XDR sentences are off in the factory settings (see VI.3)
		
* PGN currently suported

*/

//*****************************************************************************
void tN2kDataToNMEA0183::Update() {
  //SendRMC();                                 // -> RMC
//  if ( LastHeadingTime+2000<millis() ) Heading=N2kDoubleNA;
//  if ( LastCOGSOGTime+2000<millis() ) { COG=N2kDoubleNA; SOG=N2kDoubleNA; }
//  if ( LastPositionTime+4000<millis() ) { Latitude=N2kDoubleNA; Longitude=N2kDoubleNA; }
}

//*****************************************************************************
void tN2kDataToNMEA0183::SendMessage(const tNMEA0183Msg &NMEA0183Msg) {
  if ( pNMEA0183!=0 ) pNMEA0183->SendMessage(NMEA0183Msg);
  if ( SendNMEA0183MessageCallback!=0 ) SendNMEA0183MessageCallback(NMEA0183Msg);
}

/*****************************************************************************
HDG - Heading - Deviation & Variation
        1   2   3 4   5 6 
        |   |   | |   | | 
 $--HDG,x.x,x.x,a,x.x,a*hh<CR><LF>
 Field Number:  
  1) Magnetic Sensor heading in degrees 
  2) Magnetic Deviation, degrees 
  3) Magnetic Deviation direction, E = Easterly, W = Westerly 
  4) Magnetic Variation degrees 
  5) Magnetic Variation direction, E = Easterly, W = Westerly 
  6) Checksum

*/

void tN2kDataToNMEA0183::HandleHeading(const tN2kMsg &N2kMsg) {
unsigned char SID;
tN2kHeadingReference ref;
double Deviation;
double _Variation;
tNMEA0183Msg NMEA0183Msg;

  if ( ParseN2kHeading(N2kMsg, SID, Heading, Deviation, _Variation, ref) ) {
    if ( ref==N2khr_magnetic ) {
      if ( !N2kIsNA(_Variation) ) Variation=_Variation; // Update Variation
      if ( !N2kIsNA(Heading) && !N2kIsNA(Variation) ) Heading-=Variation;
    }
    LastHeadingTime=millis();
    if ( NMEA0183SetHDG(NMEA0183Msg,Heading, NMEA0183DoubleNA, Variation) ) {
      SendMessage(NMEA0183Msg);
    }
  }
}

//*****************************************************************************
void tN2kDataToNMEA0183::HandleVariation(const tN2kMsg &N2kMsg) {
unsigned char SID;
tN2kMagneticVariation Source;

  ParseN2kMagneticVariation(N2kMsg,SID,Source,DaysSince1970,Variation);
}

/*****************************************************************************
VHW - Water speed and heading
        1   2 3   4 5   6 7   8 9 
        |   | |   | |   | |   | | 
 $--VHW,x.x,T,x.x,M,x.x,N,x.x,K*hh<CR><LF>
 Field Number:  
  1) Degress True 
  2) T = True 
  3) Degrees Magnetic 
  4) M = Magnetic 
  5) Knots (speed of vessel relative to the water) 
  6) N = Knots 
  7) Kilometers (speed of vessel relative to the water) 
  8) K = Kilometers 
  9) Checksum

*/

void tN2kDataToNMEA0183::HandleBoatSpeed(const tN2kMsg &N2kMsg) {
unsigned char SID;
double WaterReferenced;
double GroundReferenced;
tN2kSpeedWaterReferenceType SWRT;

  if ( ParseN2kBoatSpeed(N2kMsg,SID,WaterReferenced,GroundReferenced,SWRT) ) {
    tNMEA0183Msg NMEA0183Msg;
    double MagneticHeading=( !N2kIsNA(Heading) && !N2kIsNA(Variation)?Heading+Variation: NMEA0183DoubleNA);
    if ( NMEA0183SetVHW(NMEA0183Msg,Heading,MagneticHeading,WaterReferenced) ) {
      SendMessage(NMEA0183Msg);
    }
  }
}

/*****************************************************************************
DBT - Depth below transducer
        1   2 3   4 5   6 7 
        |   | |   | |   | | 
 $--DBT,x.x,f,x.x,M,x.x,F*hh<CR><LF>
 Field Number:  
  1) Depth, feet 
  2) f = feet 
  3) Depth, meters 
  4) M = meters 
  5) Depth, Fathoms 
  6) F = Fathoms 
  7) Checksum

*/

void tN2kDataToNMEA0183::HandleDepth(const tN2kMsg &N2kMsg) {
unsigned char SID;
double DepthBelowTransducer;
double Offset;
double Range;

  if ( ParseN2kWaterDepth(N2kMsg,SID,DepthBelowTransducer,Offset,Range) ) {
      tNMEA0183Msg NMEA0183Msg;
      if ( NMEA0183SetDBT(NMEA0183Msg,DepthBelowTransducer) ) {
        SendMessage(NMEA0183Msg);
      }
  }
}

/*****************************************************************************
MWV - Wind Speed and Angle

        1   2 3   4 5 
        |   | |   | | 
 $--MWV,x.x,a,x.x,a*hh<CR><LF>

 Field Number:  
  1) Wind Angle, 0 to 360 degrees 
  2) Reference, R = Relative, T = True 
  3) Wind Speed 
  4) Wind Speed Units, K/M/N 
  5) Status, A = Data Valid 
  6) Checksum

*/
  
void tN2kDataToNMEA0183::HandleWindSpeed(const tN2kMsg &N2kMsg) {
unsigned char SID;
double WindSpeed;
double WindAngle;
tN2kWindReference WindReference;

  if ( ParseN2kWindSpeed(N2kMsg,SID,WindSpeed,WindAngle,WindReference) ) {
      tNMEA0183Msg NMEA0183Msg;

     if ( WindReference==N2kWind_Apprent ) {
         if ( NMEA0183SetMWV(NMEA0183Msg,RadToDeg(WindAngle),NMEA0183Wind_Apparent,WindSpeed) ) {
             SendMessage(NMEA0183Msg);
         }
     }

     if ( WindReference==N2kWind_True_North ) {
         if ( NMEA0183SetMWV(NMEA0183Msg,RadToDeg(WindAngle),NMEA0183Wind_True,WindSpeed) ) {
             SendMessage(NMEA0183Msg);
         }
     }
  }
}

/*****************************************************************************
GLL - Geographic Position - Latitude/Longitude
	   1       2 3        4 5         6 7
	   |       | |        | |         | |
$--GLL,llll.ll,a,yyyyy.yy,a,hhmmss.ss,A*hh<CR><LF>
 Field Number:
  1) Latitude
  2) N or S (North or South)
  3) Longitude
  4) E or W (East or West)
  5) Universal Time Coordinated (UTC)
  6) Status A - Data Valid, V - Data Invalid , P - Precise
  7) Checksum

*/

void tN2kDataToNMEA0183::HandlePosition(const tN2kMsg &N2kMsg) {
tNMEA0183Msg NMEA0183Msg;

  if ( ParseN2kPGN129025(N2kMsg, Latitude, Longitude) ) {
    LastPositionTime=millis();
	if (N2kHandlersDebugStream != 0) {
		N2kHandlersDebugStream->print("Pos from 129025: ");
		N2kHandlersDebugStream->print(Latitude,8);
		N2kHandlersDebugStream->print(" ");
		N2kHandlersDebugStream->println(Longitude,8);

		N2kHandlersDebugStream->print("Pos from 129025: ");
		N2kHandlersDebugStream->print(DoubleToddmm(Latitude), 8);
		N2kHandlersDebugStream->print(" ");
		N2kHandlersDebugStream->println(DoubleToddmm(Longitude), 8);
	}
	if (NMEA0183SetGLL(NMEA0183Msg, NMEA0183DoubleNA, Latitude, Longitude, 0)) {
		SendMessage(NMEA0183Msg);
	}
  }
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

//*****************************************************************************
void tN2kDataToNMEA0183::HandleCOGSOG(const tN2kMsg &N2kMsg) {
unsigned char SID;
tN2kHeadingReference HeadingReference;
tNMEA0183Msg NMEA0183Msg;

  if ( ParseN2kCOGSOGRapid(N2kMsg,SID,HeadingReference,COG,SOG) ) {
    LastCOGSOGTime=millis();
    double MCOG=( !N2kIsNA(COG) && !N2kIsNA(Variation)?COG-Variation:NMEA0183DoubleNA );
    if ( HeadingReference==N2khr_magnetic ) {
      MCOG=COG;
      if ( !N2kIsNA(Variation) ) COG-=Variation;
    }
    if ( NMEA0183SetVTG(NMEA0183Msg,COG,MCOG,SOG) ) {
      SendMessage(NMEA0183Msg);
    }
  }
}

/*****************************************************************************
GGA - Global Positioning System Fix Data, Time, Position and fix related data fora GPS receiver.
		1         2       3 4        5 6 7  8   9  10 11 12 13  14   15
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

******************************************************************************
GLL - Geographic Position - Latitude/Longitude
	   1       2 3        4 5         6 7
	   |       | |        | |         | |
$--GLL,llll.ll,a,yyyyy.yy,a,hhmmss.ss,A*hh<CR><LF>
 Field Number:
  1) Latitude
  2) N or S (North or South)
  3) Longitude
  4) E or W (East or West)
  5) Universal Time Coordinated (UTC)
  6) Status A - Data Valid, V - Data Invalid , P - Precise
  7) Checksum

*/

//*****************************************************************************
void tN2kDataToNMEA0183::HandleGNSS(const tN2kMsg &N2kMsg) {
unsigned char SID;
tN2kGNSStype GNSStype;
tN2kGNSSmethod GNSSmethod;
unsigned char nSatellites;
double HDOP;
double PDOP;
double GeoidalSeparation;
unsigned char nReferenceStations;
tN2kGNSStype ReferenceStationType;
uint16_t ReferenceStationID;
double AgeOfCorrection;
int GPSQualityIndicator=0;
tNMEA0183Msg NMEA0183Msg;

  if ( ParseN2kGNSS(N2kMsg,SID,DaysSince1970,SecondsSinceMidnight,Latitude,Longitude,Altitude,GNSStype,GNSSmethod,
                    nSatellites,HDOP,PDOP,GeoidalSeparation,
                    nReferenceStations,ReferenceStationType,ReferenceStationID,AgeOfCorrection) ) {
    LastPositionTime=millis();

	if (N2kHandlersDebugStream != 0) {
		N2kHandlersDebugStream->print("Pos from 129029: ");
		N2kHandlersDebugStream->print(Latitude,8);
		N2kHandlersDebugStream->print(" ");
		N2kHandlersDebugStream->println(Longitude,8);
	}

    if ( NMEA0183SetGGA(NMEA0183Msg,SecondsSinceMidnight,Latitude,Longitude,GPSQualityIndicator,nSatellites,HDOP,Altitude,GeoidalSeparation,AgeOfCorrection,ReferenceStationID) ) {
      SendMessage(NMEA0183Msg);
    }
	if (NMEA0183SetGLL(NMEA0183Msg, SecondsSinceMidnight, Latitude, Longitude, 0)) {
		SendMessage(NMEA0183Msg);
	}
  }
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

//*****************************************************************************
void tN2kDataToNMEA0183::SendRMC() {
    if ( NextRMCSend<=millis() && !N2kIsNA(Latitude) && ( SecondsSinceMidnight > 0 ) ) {
      tNMEA0183Msg NMEA0183Msg;
      if ( NMEA0183SetRMC(NMEA0183Msg,SecondsSinceMidnight,Latitude,Longitude,COG,SOG,DaysSince1970,Variation) ) {
        SendMessage(NMEA0183Msg);
      }
      SetNextRMCSend();
    }
}

