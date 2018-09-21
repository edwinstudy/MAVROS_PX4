/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <ctime>

#include "ashtech.h"

GPSDriverAshtech::GPSDriverAshtech(GPSCallbackPtr callback, void *callback_user,
				   struct vehicle_gps_position_s *gps_position,
				   struct satellite_info_s *satellite_info):
	GPSHelper(callback, callback_user),
	_satellite_info(satellite_info),
	_gps_position(gps_position),
	_last_timestamp_time(0),
	_got_pashr_pos_message(false)
{
	decodeInit();
	_decode_state = NME_DECODE_UNINIT;
	_rx_buffer_bytes = 0;
}

GPSDriverAshtech::~GPSDriverAshtech()
{
}

/*
 * All NMEA descriptions are taken from
 * http://www.trimble.com/OEM_ReceiverHelp/V4.44/en/NMEA-0183messages_MessageOverview.html
 */

int GPSDriverAshtech::handleMessage(int len)
{
	char *endp;
  //  double UtcTime = 0.0;

	if (len < 7) { return 0; }

	int uiCalcComma = 0;

	for (int i = 0 ; i < len; i++) {
		if (_rx_buffer[i] == ',') { uiCalcComma++; }
	}

	char *bufptr = (char *)(_rx_buffer + 6);
	int ret = 0;

	if ((memcmp(_rx_buffer + 3, "ZDA,", 3) == 0) && (uiCalcComma == 6)) {
		/*
		UTC day, month, and year, and local time zone offset
		An example of the ZDA message string is:

		$GPZDA,172809.456,12,07,1996,00,00*45

		ZDA message fields
		Field	Meaning
		0	Message ID $GPZDA
		1	UTC
		2	Day, ranging between 01 and 31
		3	Month, ranging between 01 and 12
		4	Year
		5	Local time zone offset from GMT, ranging from 00 through 13 hours
		6	Local time zone offset from GMT, ranging from 00 through 59 minutes
		7	The checksum data, always begins with *
		Fields 5 and 6 together yield the total offset. For example, if field 5 is -5 and field 6 is +15, local time is 5 hours and 15 minutes earlier than GMT.
		*/
		double ashtech_time = 0.0;
		int day = 0, month = 0, year = 0, local_time_off_hour __attribute__((unused)) = 0,
		    local_time_off_min __attribute__((unused)) = 0;

		if (bufptr && *(++bufptr) != ',') { ashtech_time = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { day = strtol(bufptr, &endp, 10); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { month = strtol(bufptr, &endp, 10); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { year = strtol(bufptr, &endp, 10); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { local_time_off_hour = strtol(bufptr, &endp, 10); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { local_time_off_min = strtol(bufptr, &endp, 10); bufptr = endp; }


		int ashtech_hour = static_cast<int>(ashtech_time / 10000);
		int ashtech_minute = static_cast<int>((ashtech_time - ashtech_hour * 10000) / 100);
		double ashtech_sec = static_cast<float>(ashtech_time - ashtech_hour * 10000 - ashtech_minute * 100);

		/*
		 * convert to unix timestamp
		 */
		struct tm timeinfo;
		timeinfo.tm_year = year - 1900;
		timeinfo.tm_mon = month - 1;
		timeinfo.tm_mday = day;
		timeinfo.tm_hour = ashtech_hour;
		timeinfo.tm_min = ashtech_minute;
		timeinfo.tm_sec = int(ashtech_sec);

#ifndef NO_MKTIME
		time_t epoch = mktime(&timeinfo);

		if (epoch > GPS_EPOCH_SECS) {
			uint64_t usecs = static_cast<uint64_t>((ashtech_sec - static_cast<uint64_t>(ashtech_sec))) * 1000000;

			// FMUv2+ boards have a hardware RTC, but GPS helps us to configure it
			// and control its drift. Since we rely on the HRT for our monotonic
			// clock, updating it from time to time is safe.

			timespec ts;
			ts.tv_sec = epoch;
			ts.tv_nsec = usecs * 1000;

			setClock(ts);

			_gps_position->time_utc_usec = static_cast<uint64_t>(epoch) * 1000000ULL;
			_gps_position->time_utc_usec += usecs;

		} else {
			_gps_position->time_utc_usec = 0;
		}

#else
		_gps_position->time_utc_usec = 0;
#endif

		_last_timestamp_time = gps_absolute_time();
//        ret = 1;    //jim
	}

	else if ((memcmp(_rx_buffer + 3, "GGA,", 3) == 0) && (uiCalcComma == 14) && !_got_pashr_pos_message) {
		/*
		  Time, position, and fix related data
		  An example of the GBS message string is:

		  $GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0,0031*4F

		  Note - The data string exceeds the ASHTECH standard length.
		  GGA message fields
		  Field   Meaning
		  0   Message ID $GPGGA
		  1   UTC of position fix
		  2   Latitude
		  3   Direction of latitude:
		  N: North
		  S: South
		  4   Longitude
		  5   Direction of longitude:
		  E: East
		  W: West
		  6   GPS Quality indicator:
		  0: Fix not valid
		  1: GPS fix
		  2: Differential GPS fix, OmniSTAR VBS
		  4: Real-Time Kinematic, fixed integers
		  5: Real-Time Kinematic, float integers, OmniSTAR XP/HP or Location RTK
		  7   Number of SVs in use, range from 00 through to 24+
		  8   HDOP
		  9   Orthometric height (MSL reference)
		  10  M: unit of measure for orthometric height is meters
		  11  Geoid separation
		  12  M: geoid separation measured in meters
		  13  Age of differential GPS data record, Type 1 or Type 9. Null field when DGPS is not used.
		  14  Reference station ID, range 0000-4095. A null field when any reference station ID is selected and no corrections are received1.
		  15
		  The checksum data, always begins with *
		  Note - If a user-defined geoid model, or an inclined
		*/
		double ashtech_time __attribute__((unused)) = 0.0, lat = 0.0, lon = 0.0, alt = 0.0;
		int num_of_sv __attribute__((unused)) = 0, fix_quality = 0;
		double hdop __attribute__((unused)) = 99.9;
		char ns = '?', ew = '?';

		if (bufptr && *(++bufptr) != ',') { ashtech_time = strtod(bufptr, &endp); bufptr = endp; }
        PX4_INFO("GGA  UTC:%.1f",ashtech_time);
		if (bufptr && *(++bufptr) != ',') { lat = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { ns = *(bufptr++); }

		if (bufptr && *(++bufptr) != ',') { lon = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { ew = *(bufptr++); }

		if (bufptr && *(++bufptr) != ',') { fix_quality = strtol(bufptr, &endp, 10); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { num_of_sv = strtol(bufptr, &endp, 10); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { hdop = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { alt = strtod(bufptr, &endp); bufptr = endp; }

		if (ns == 'S') {
			lat = -lat;
		}

		if (ew == 'W') {
			lon = -lon;
		}

		/* convert from degrees, minutes and seconds to degrees * 1e7 */
		_gps_position->lat = static_cast<int>((int(lat * 0.01) + (lat * 0.01 - int(lat * 0.01)) * 100.0 / 60.0) * 10000000);
		_gps_position->lon = static_cast<int>((int(lon * 0.01) + (lon * 0.01 - int(lon * 0.01)) * 100.0 / 60.0) * 10000000);
		_gps_position->alt = static_cast<int>(alt * 1000);
        _gps_position->hdop = (float)hdop;  //yuli add this
        _gps_position->satellites_used = num_of_sv;  //yuli add this
		_rate_count_lat_lon++;
       //  PX4_INFO("lat:%.7f,lon:%.7f",(double)lat,(double)lon);
		if (fix_quality <= 0) {
			_gps_position->fix_type = 0;

		} else {
			/*
			 * in this NMEA message float integers (value 5) mode has higher value than fixed integers (value 4), whereas it provides lower quality,
			 * and since value 3 is not being used, I "moved" value 5 to 3 to add it to _gps_position->fix_type
			 */
			if (fix_quality == 5) { fix_quality = 3; }

			/*
			 * fix quality 1 means just a normal 3D fix, so I'm subtracting 1 here. This way we'll have 3 for auto, 4 for DGPS, 5 for floats, 6 for fixed.
			 */
			_gps_position->fix_type = 3 + fix_quality - 1;
		}

		_gps_position->timestamp = gps_absolute_time();
       // PX4_INFO("_gps_position->timestamp: %llu",_gps_position->timestamp);

//		_gps_position->vel_m_s = 0;                                  /**< GPS ground speed (m/s) */
//		_gps_position->vel_n_m_s = 0;                                /**< GPS ground speed in m/s */
//		_gps_position->vel_e_m_s = 0;                                /**< GPS ground speed in m/s */
//		_gps_position->vel_d_m_s = 0;                                /**< GPS ground speed in m/s */
//		_gps_position->cog_rad =
//			0;                                  /**< Course over ground (NOT heading, but direction of movement) in rad, -PI..PI */
//		_gps_position->vel_ned_valid = true;                         /**< Flag to indicate if NED speed is valid */
        _gps_position->c_variance_rad = 0.1f;
//		ret = 1;
        //PX4_INFO("utc:%.2f,lat:%.7f,lon:%.7f,number:%d,HDOP:%.1f",(double)ashtech_time,(double)(_gps_position->lat/10000000.0),(double)(_gps_position->lon/10000000.0),num_of_sv,(double)_gps_position->hdop);
    }
#if 0
    else if ((memcmp(_rx_buffer, "$PASHR,POS,", 11) == 0) && (uiCalcComma == 18))
    {
		_got_pashr_pos_message = true;
		/*
		Example $PASHR,POS,2,10,125410.00,5525.8138702,N,03833.9587380,E,131.555,1.0,0.0,0.007,-0.001,2.0,1.0,1.7,1.0,*34

		    $PASHR,POS,d1,d2,m3,m4,c5,m6,c7,f8,f9,f10,f11,f12,f13,f14,f15,f16,s17*cc
		    Parameter Description Range
		      d1 Position mode 0: standalone
		                       1: differential
		                       2: RTK float
		                       3: RTK fixed
		                       5: Dead reckoning
		                       9: SBAS (see NPT setting)
		      d2 Number of satellite used in position fix 0-99
		      m3 Current UTC time of position fix (hhmmss.ss) 000000.00-235959.99
		      m4 Latitude of position (ddmm.mmmmmm) 0-90 degrees 00-59.9999999 minutes
		      c5 Latitude sector N, S
		      m6 Longitude of position (dddmm.mmmmmm) 0-180 degrees 00-59.9999999 minutes
		      c7 Longitude sector E,W
		      f8 Altitude above ellipsoid +9999.000
		      f9 Differential age (data link age), seconds 0.0-600.0
		      f10 True track/course over ground in degrees 0.0-359.9
		      f11 Speed over ground in knots 0.0-999.9
		      f12 Vertical velocity in decimeters per second +999.9
		      f13 PDOP 0-99.9
		      f14 HDOP 0-99.9
              f15 VDOP 0-99.9
		      f16 TDOP 0-99.9
		      s17 Reserved no data
		      *cc Checksum
		    */
		bufptr = (char *)(_rx_buffer + 10);

		/*
		 * Ashtech would return empty space as coordinate (lat, lon or alt) if it doesn't have a fix yet
		 */
		int coordinatesFound = 0;
		double ashtech_time __attribute__((unused)) = 0.0, lat = 0.0, lon = 0.0, alt = 0.0;
		int num_of_sv __attribute__((unused)) = 0, fix_quality = 0;
		double track_true = 0.0, ground_speed = 0.0 , age_of_corr __attribute__((unused)) = 0.0;
		double hdop = 99.9, vdop = 99.9,  pdop __attribute__((unused)) = 99.9,
		       tdop __attribute__((unused)) = 99.9, vertic_vel = 0.0;
		char ns = '?', ew = '?';

		if (bufptr && *(++bufptr) != ',') { fix_quality = strtol(bufptr, &endp, 10); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { num_of_sv = strtol(bufptr, &endp, 10); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { ashtech_time = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') {
			/*
			 * if a coordinate is skipped (i.e. no fix), it either won't get into this block (two commas in a row)
			 * or strtod won't find anything and endp will point exactly where bufptr is. The same is for lon and alt.
			 */
			lat = strtod(bufptr, &endp);

			if (bufptr != endp) {coordinatesFound++;}

			bufptr = endp;
		}

		if (bufptr && *(++bufptr) != ',') { ns = *(bufptr++); }

		if (bufptr && *(++bufptr) != ',') {
			lon = strtod(bufptr, &endp);

			if (bufptr != endp) {coordinatesFound++;}

			bufptr = endp;
		}

		if (bufptr && *(++bufptr) != ',') { ew = *(bufptr++); }

		if (bufptr && *(++bufptr) != ',') {
			alt = strtod(bufptr, &endp);

			if (bufptr != endp) {coordinatesFound++;}

			bufptr = endp;
		}

		if (bufptr && *(++bufptr) != ',') { age_of_corr = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { track_true = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { ground_speed = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { vertic_vel = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { pdop = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { hdop = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { vdop = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { tdop = strtod(bufptr, &endp); bufptr = endp; }

		if (ns == 'S') {
			lat = -lat;
		}

		if (ew == 'W') {
			lon = -lon;
		}

		_gps_position->lat = static_cast<int>((int(lat * 0.01) + (lat * 0.01 - int(lat * 0.01)) * 100.0 / 60.0) * 10000000);
		_gps_position->lon = static_cast<int>((int(lon * 0.01) + (lon * 0.01 - int(lon * 0.01)) * 100.0 / 60.0) * 10000000);
		_gps_position->alt = static_cast<int>(alt * 1000);		
        _gps_position->hdop = (float)hdop / 100.0f;
		_gps_position->vdop = (float)vdop / 100.0f;
		_rate_count_lat_lon++;
        //PX4_INFO("_gps_position->hdop:%.1f",(double)_gps_position->hdop);
		if (coordinatesFound < 3) {
			_gps_position->fix_type = 0;

		} else {
			_gps_position->fix_type = 3 + fix_quality;
		}

		_gps_position->timestamp = gps_absolute_time();

		float track_rad = static_cast<float>(track_true) * M_PI_F / 180.0f;

		float velocity_ms = static_cast<float>(ground_speed) / 1.9438445f;			/** knots to m/s */
		float velocity_north = static_cast<float>(velocity_ms) * cosf(track_rad);
		float velocity_east  = static_cast<float>(velocity_ms) * sinf(track_rad);

		_gps_position->vel_m_s = velocity_ms;				/** GPS ground speed (m/s) */
		_gps_position->vel_n_m_s = velocity_north;			/** GPS ground speed in m/s */
		_gps_position->vel_e_m_s = velocity_east;			/** GPS ground speed in m/s */
		_gps_position->vel_d_m_s = static_cast<float>(-vertic_vel);				/** GPS ground speed in m/s */
		_gps_position->cog_rad =
			track_rad;				/** Course over ground (NOT heading, but direction of movement) in rad, -PI..PI */
		_gps_position->vel_ned_valid = true;				/** Flag to indicate if NED speed is valid */
		_gps_position->c_variance_rad = 0.1f;
		_rate_count_vel++;
		ret = 1;

    }
#endif
    else if((memcmp(_rx_buffer + 1, "BESTVELA,", 8) == 0) && (uiCalcComma == 16))
    {
//       PX4_INFO("BESTVELA");
        int i = 0 ,j = 0;
        for(i = 0; i < len; i++)
        {
            if(_rx_buffer[i] == ';')
            {

                break;
            }
        }
    //    bufptr = (char *)(_rx_buffer + 10);

     //   int sol_stat __attribute__((unused)) = 0;
     //   int vel_type __attribute__((unused)) = 0;
        float latency __attribute__((unused)) = 0.0f;
        float age __attribute__((unused)) = 0.0f;
        double hor_spd = 0.0;
        double trk_gnd = 0.0;
        double vert_spd = 0.0;
      //  float Reserved = 0.0f;



//        if (bufptr && *(++bufptr) != ',') { fix_quality = strtol(bufptr, &endp, 10); bufptr = endp; }
//        if (bufptr && *(++bufptr) != ',') { sol_stat = strtod(bufptr, &endp); bufptr = endp; }

//        if (bufptr && *(++bufptr) != ',') { vel_type = strtod(bufptr, &endp); bufptr = endp; }
        bufptr = (char *)(_rx_buffer + i + 1);
        //if(memcmp(bufptr, "SOL_COMPUTED",12) != 0 || memcmp(bufptr+13, "NONE",4) == 0)  return 0;
       // f(memccpy(_bufptr, "SOL_COMPUTED",13) != 0)  return 0;
       int num_douhao = 0;
        for(j = 0; j < len - i; j++)
        {
            if(_rx_buffer[i + j] == ',')
            {
                num_douhao++;
            }
            if(num_douhao == 2)
            {
                break;
            }
        }

        bufptr = (char *)(_rx_buffer + i + j + 1);

        if (bufptr && *(++bufptr) != ',') { latency = strtod(bufptr, &endp); bufptr = endp; }

        if (bufptr && *(++bufptr) != ',') { age = strtod(bufptr, &endp); bufptr = endp; }

        if (bufptr && *(++bufptr) != ',') { hor_spd = strtod(bufptr, &endp); bufptr = endp; }
        PX4_INFO("BESTVEL spd:%.2f",hor_spd);
        if (bufptr && *(++bufptr) != ',') { trk_gnd = strtod(bufptr, &endp); bufptr = endp; }

        if (bufptr && *(++bufptr) != ',') { vert_spd = strtod(bufptr, &endp); bufptr = endp; }


        float velocity_ms = static_cast<float>(hor_spd) ;			/** knots to m/s */
        float track_rad = static_cast<float>(trk_gnd)* M_PI_F / 180.0f;
        float velocity_north = static_cast<float>(velocity_ms) * cosf(track_rad);
        float velocity_east  = static_cast<float>(velocity_ms) * sinf(track_rad);

        _gps_position->vel_m_s = velocity_ms;				/** GPS ground speed (m/s) */
        _gps_position->vel_n_m_s = velocity_north;			/** GPS ground speed in m/s */
        _gps_position->vel_e_m_s = velocity_east;			/** GPS ground speed in m/s */
        _gps_position->vel_d_m_s = static_cast<float>(-vert_spd);				/** GPS ground speed in m/s */
        _gps_position->vel_ned_valid = true;
        _rate_count_vel++;
        ret = 1;
        //PX4_INFO("_gps_position utc:%.3f,_vel_n_m_s:%.3f,vel_e_m_s:%.3f,vel_d_m_s:%.3f", (double)UtcTime,(double)_gps_position->vel_n_m_s,(double)_gps_position->vel_e_m_s,(double)_gps_position->vel_d_m_s);
    }

    else if ((memcmp(_rx_buffer + 3, "GSA,", 3) == 0) && (uiCalcComma == 17))
    {
        double vdop __attribute__((unused)) = 0.0;
        int i = 0,j = 0;
        for(i = 0; i < len; i++)
        {
            if(_rx_buffer[i] == ',')
            {
                j++;
            }
            if (j == 17)   break;
        }
        bufptr = (char *)(_rx_buffer + i + 1);
        if (bufptr && *(++bufptr) != '*') { vdop = strtod(bufptr, &endp); bufptr = endp; }
        _gps_position->vdop = vdop;

    }
    else if ((memcmp(_rx_buffer + 3, "GST,", 3) == 0) && (uiCalcComma == 8)) {
		/*
		  Position error statistics
		  An example of the GST message string is:

		  $GPGST,172814.0,0.006,0.023,0.020,273.6,0.023,0.020,0.031*6A

		  The Talker ID ($--) will vary depending on the satellite system used for the position solution:

		  $GP - GPS only
		  $GL - GLONASS only
		  $GN - Combined
		  GST message fields
		  Field   Meaning
		  0   Message ID $GPGST
		  1   UTC of position fix
		  2   RMS value of the pseudorange residuals; includes carrier phase residuals during periods of RTK (float) and RTK (fixed) processing
		  3   Error ellipse semi-major axis 1 sigma error, in meters
		  4   Error ellipse semi-minor axis 1 sigma error, in meters
		  5   Error ellipse orientation, degrees from true north
		  6   Latitude 1 sigma error, in meters
		  7   Longitude 1 sigma error, in meters
		  8   Height 1 sigma error, in meters
		  9   The checksum data, always begins with *
		*/
		double ashtech_time __attribute__((unused)) = 0.0, lat_err = 0.0, lon_err = 0.0, alt_err = 0.0;
		double min_err __attribute__((unused)) = 0.0, maj_err __attribute__((unused)) = 0.0,
		deg_from_north __attribute__((unused)) = 0.0, rms_err __attribute__((unused)) = 0.0;

		if (bufptr && *(++bufptr) != ',') { ashtech_time = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { rms_err = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { maj_err = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { min_err = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { deg_from_north = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { lat_err = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { lon_err = strtod(bufptr, &endp); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { alt_err = strtod(bufptr, &endp); bufptr = endp; }

		_gps_position->eph = sqrtf(static_cast<float>(lat_err) * static_cast<float>(lat_err)
					   + static_cast<float>(lon_err) * static_cast<float>(lon_err));
		_gps_position->epv = static_cast<float>(alt_err);

		_gps_position->s_variance_m_s = 0;
//        ret = 1;    //jim

	} else if ((memcmp(_rx_buffer + 3, "GSV,", 3) == 0)) {
		/*
		  The GSV message string identifies the number of SVs in view, the PRN numbers, elevations, azimuths, and SNR values. An example of the GSV message string is:

		  $GPGSV,4,1,13,02,02,213,,03,-3,000,,11,00,121,,14,13,172,05*67

		  GSV message fields
		  Field   Meaning
		  0   Message ID $GPGSV
		  1   Total number of messages of this type in this cycle
		  2   Message number
		  3   Total number of SVs visible
		  4   SV PRN number
		  5   Elevation, in degrees, 90 maximum
		  6   Azimuth, degrees from True North, 000 through 359
		  7   SNR, 00 through 99 dB (null when not tracking)
		  8-11    Information about second SV, same format as fields 4 through 7
		  12-15   Information about third SV, same format as fields 4 through 7
		  16-19   Information about fourth SV, same format as fields 4 through 7
		  20  The checksum data, always begins with *
		*/
		/*
		 * currently process only gps, because do not know what
		 * Global satellite ID I should use for non GPS sats
		 */
		bool bGPS = false;

		if (memcmp(_rx_buffer, "$GP", 3) != 0) {
			return 0;

		} else {
			bGPS = true;
		}

		int all_msg_num = 0, this_msg_num = 0, tot_sv_visible = 0;
		struct gsv_sat {
			int svid;
			int elevation;
			int azimuth;
			int snr;
		} sat[4];
		memset(sat, 0, sizeof(sat));

		if (bufptr && *(++bufptr) != ',') { all_msg_num = strtol(bufptr, &endp, 10); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { this_msg_num = strtol(bufptr, &endp, 10); bufptr = endp; }

		if (bufptr && *(++bufptr) != ',') { tot_sv_visible = strtol(bufptr, &endp, 10); bufptr = endp; }

		if ((this_msg_num < 1) || (this_msg_num > all_msg_num)) {
			return 0;
		}

		if (this_msg_num == 0 && bGPS && _satellite_info) {
			memset(_satellite_info->svid,     0, sizeof(_satellite_info->svid));
			memset(_satellite_info->used,     0, sizeof(_satellite_info->used));
			memset(_satellite_info->snr,      0, sizeof(_satellite_info->snr));
			memset(_satellite_info->elevation, 0, sizeof(_satellite_info->elevation));
			memset(_satellite_info->azimuth,  0, sizeof(_satellite_info->azimuth));
		}

		int end = 4;

		if (this_msg_num == all_msg_num) {
			end =  tot_sv_visible - (this_msg_num - 1) * 4;
			_gps_position->satellites_used = tot_sv_visible;

			if (_satellite_info) {
				_satellite_info->count = satellite_info_s::SAT_INFO_MAX_SATELLITES;
				_satellite_info->timestamp = gps_absolute_time();
			}
		}

		if (_satellite_info) {
			for (int y = 0 ; y < end ; y++) {
				if (bufptr && *(++bufptr) != ',') { sat[y].svid = strtol(bufptr, &endp, 10); bufptr = endp; }

				if (bufptr && *(++bufptr) != ',') { sat[y].elevation = strtol(bufptr, &endp, 10); bufptr = endp; }

				if (bufptr && *(++bufptr) != ',') { sat[y].azimuth = strtol(bufptr, &endp, 10); bufptr = endp; }

				if (bufptr && *(++bufptr) != ',') { sat[y].snr = strtol(bufptr, &endp, 10); bufptr = endp; }

				_satellite_info->svid[y + (this_msg_num - 1) * 4]      = sat[y].svid;
				_satellite_info->used[y + (this_msg_num - 1) * 4]      = ((sat[y].snr > 0) ? true : false);
				_satellite_info->snr[y + (this_msg_num - 1) * 4]       = sat[y].snr;
				_satellite_info->elevation[y + (this_msg_num - 1) * 4] = sat[y].elevation;
				_satellite_info->azimuth[y + (this_msg_num - 1) * 4]   = sat[y].azimuth;
			}
		}
//        ret = 1;    //jim
	}

	if (ret > 0) {
		_gps_position->timestamp_time_relative = (int32_t)(_last_timestamp_time - _gps_position->timestamp);
	}

	return ret;
}


int GPSDriverAshtech::receive(unsigned timeout)
{
	{

        static uint8_t buf[GPS_READ_BUFFER_SIZE];
//        static int left = 0;
        static int ret = 0;
       // uint8_t bufff[GPS_READ_BUFFER_SIZE];
		/* timeout additional to poll */
		uint64_t time_started = gps_absolute_time();

        int j = 0;
		ssize_t bytes_count = 0;
//        int handled = 0;
//        bool get_gga_flag = false;
		while (true) {

			/* pass received bytes to the packet decoder */
			while (j < bytes_count) {
				int l = 0;
                if ((l = parseChar(buf[j])) > 0) {
//                    left = bytes_count -j-1;
//                    PX4_INFO("left: %d",left);
//                    PX4_INFO("leftBuf: %s",buf);
					/* return to configure during configuration or to the gps driver during normal work
					 * if a packet has arrived */

//                    handled = handleMessage(l);
//                    if(handled > 0)
//                    {
//                         get_gga_flag = true;

//                    }
                    if (handleMessage(l) > 0 ) {
                        return 1;
                    }

				}
				j++;
//                if(j == bytes_count && get_gga_flag/* && handled*/)
//                {
////                    PX4_INFO("IN HANDLE");
//                    get_gga_flag = false;
//                    return 1;
//                }
			}
			/* everything is read */
			j = bytes_count = 0;

			/* then poll or read for new data */
            ret = read(buf, sizeof(buf), timeout * 2);

//             ret = read(buf + left, sizeof(buf) - left, timeout * 2);
            // memcpy(buf,buf,GPS_READ_BUFFER_SIZE);
          //  char* str = buf;
//            PX4_INFO("bufSize: %d",ret);
//            PX4_INFO("bufReal: %s",buf);
			if (ret < 0) {
				/* something went wrong when polling */
				return -1;

			} else if (ret == 0) {
				/* Timeout while polling or just nothing read if reading, let's
				 * stay here, and use timeout below. */

			} else if (ret > 0) {
				/* if we have new data from GPS, go handle it */
				bytes_count = ret;
			}

			/* in case we get crap from GPS or time out */
			if (time_started + timeout * 1000 * 2 < gps_absolute_time()) {
				return -1;
			}
		}
	}

}
#define HEXDIGIT_CHAR(d) ((char)((d) + (((d) < 0xA) ? '0' : 'A'-0xA)))

int GPSDriverAshtech::parseChar(uint8_t b)
{
    // PX4_INFO("arrive parseChar: %c",(char)b);
    int iRet = 0;
 //   int gga_or_vela = 0;   //yuli add this
    if (b == '$'|| b =='#') {          //YULI
        _decode_state = NME_DECODE_UNINIT;
        if(b == '$' )
         {
             ga_or_vela = 1;
//             PX4_INFO("ga_or_vela = 1");
         }
         else
         {
             ga_or_vela = 2;
//             PX4_INFO("ga_or_vela = 2");
         }
    }
    switch (_decode_state)
    {
	/* First, look for sync1 */
//    PX4_INFO("_decode_state");
	case NME_DECODE_UNINIT:
//        if (b == '$'|| b =='#') {          //YULI
//             if(b == '$' )
//             {
//                 ga_or_vela = 1;
//             PX4_INFO("ga_or_vela = 1");
//             }
//             else
//             {
//                 ga_or_vela = 2;
//             PX4_INFO("ga_or_vela = 2");
//             }
            _decode_state = NME_DECODE_GOT_SYNC1;
			_rx_buffer_bytes = 0;
			_rx_buffer[_rx_buffer_bytes++] = b;
        //}

		break;

	case NME_DECODE_GOT_SYNC1:
//        if (b == '$'||b == '#')
//        {          // YULI
//            if(b == '$' )
//            {
//                ga_or_vela = 1;
//                PX4_INFO("ga_or_vela = 11");


//            }
//            else
//            {
//                ga_or_vela = 2;
//                PX4_INFO("ga_or_vela = 22");
//            }
//			_decode_state = NME_DECODE_GOT_SYNC1;
//			_rx_buffer_bytes = 0;

//        }
       /* else*/ if (b == '*') {
			_decode_state = NME_DECODE_GOT_ASTERIKS;
//            PX4_INFO("get the star");
		}

		if (_rx_buffer_bytes >= (sizeof(_rx_buffer) - 5)) {
			_decode_state = NME_DECODE_UNINIT;
			_rx_buffer_bytes = 0;

		} else {
			_rx_buffer[_rx_buffer_bytes++] = b;
       //     PX4_INFO("get the gps");
		}

		break;

	case NME_DECODE_GOT_ASTERIKS:
		_rx_buffer[_rx_buffer_bytes++] = b;
        if( ga_or_vela == 1)
        {
            _decode_state = NME_DECODE_GOT_FIRST_CS_BYTE;
//            break;
        }
        else if( ga_or_vela == 2)
        {
            NUM_ASTERIKS++;
        }
        if(NUM_ASTERIKS == 7)
        {
            _decode_state = NME_DECODE_GOT_FIRST_CS_BYTE;
       //      PX4_INFO("get VELA");
             NUM_ASTERIKS = 0;

        }
            break;
	case NME_DECODE_GOT_FIRST_CS_BYTE:
//return -1;
        _rx_buffer[_rx_buffer_bytes++] = b;




    if(ga_or_vela == 1)
    {
        uint8_t checksum = 0;
		uint8_t *buffer = _rx_buffer + 1;
		uint8_t *bufend = _rx_buffer + _rx_buffer_bytes - 3;

		for (; buffer < bufend; buffer++) { checksum ^= *buffer; }

		if ((HEXDIGIT_CHAR(checksum >> 4) == *(_rx_buffer + _rx_buffer_bytes - 2)) &&
		    (HEXDIGIT_CHAR(checksum & 0x0F) == *(_rx_buffer + _rx_buffer_bytes - 1))) {
            iRet = _rx_buffer_bytes;

           // PX4_INFO("$$$$$$$$$$$");
//             PX4_INFO("buf: %s",_rx_buffer);
          //  _decode_state = NME_DECODE_UNINIT;
          //  _rx_buffer_bytes = 0;
           // return iRet;
            //break;
        }
        _decode_state = NME_DECODE_UNINIT;
        _rx_buffer_bytes = 0;
        break;
    }
    if(ga_or_vela == 2)
    {
        uint8_t m[8];
        unsigned long  a = CalculateBlockCRC32(_rx_buffer_bytes - 10,_rx_buffer + 1);
        m[0] = a&0x0f;
        m[1] = (a>>4)&0x0f;
        m[2] = (a>>8)&0x0f;
        m[3] = (a>>12)&0x0f;
        m[4] = (a>>16)&0x0f;
        m[5] = (a>>20)&0x0f;
        m[6] = (a>>24)&0x0f;
        m[7] = (a>>28)&0x0f;
      //  PX4_INFO("m[0]: %c,CRC_last: %c", HEXDIGIT_CHAR_S(m[0]),(char)*(_rx_buffer + _rx_buffer_bytes - 1));

        if (HEXDIGIT_CHAR_S(m[7]) == *(_rx_buffer + _rx_buffer_bytes - 8)&&
            HEXDIGIT_CHAR_S(m[6]) == *(_rx_buffer + _rx_buffer_bytes - 7)&&
            HEXDIGIT_CHAR_S(m[5]) == *(_rx_buffer + _rx_buffer_bytes - 6)&&
            HEXDIGIT_CHAR_S(m[4]) == *(_rx_buffer + _rx_buffer_bytes - 5)&&
            HEXDIGIT_CHAR_S(m[3]) == *(_rx_buffer + _rx_buffer_bytes - 4)&&
            HEXDIGIT_CHAR_S(m[2]) == *(_rx_buffer + _rx_buffer_bytes - 3)&&
            HEXDIGIT_CHAR_S(m[1]) == *(_rx_buffer + _rx_buffer_bytes - 2)&&
            HEXDIGIT_CHAR_S(m[0]) == *(_rx_buffer + _rx_buffer_bytes - 1))
        {
            iRet = _rx_buffer_bytes;
           // PX4_INFO("###############");
          //  _decode_state = NME_DECODE_UNINIT;
          //  _rx_buffer_bytes = 0;
//             PX4_INFO("get CRC");
//             PX4_INFO("buf: %s",_rx_buffer);
           // return iRet;
            //break;
        }
          _decode_state = NME_DECODE_UNINIT;
          _rx_buffer_bytes = 0;
          break;
	}

    //return iRet;
    }
    return iRet;
}

void GPSDriverAshtech::decodeInit(void)
{

}

/*
 * ashtech board configuration script
 */

const char comm[] = "$PASHS,POP,20\r\n"\
		    "$PASHS,NME,ZDA,B,ON,3\r\n"\
		    "$PASHS,NME,GGA,B,OFF\r\n"\
		    "$PASHS,NME,GST,B,ON,3\r\n"\
		    "$PASHS,NME,POS,B,ON,0.05\r\n"\
		    "$PASHS,NME,GSV,B,ON,3\r\n"\
		    "$PASHS,SPD,A,8\r\n"\
            "$PASHS,SPD,B,9\r\n";

int GPSDriverAshtech::configure(unsigned &baudrate, OutputMode output_mode)
{
	if (output_mode != OutputMode::GPS) {
		GPS_WARN("ASHTECH: Unsupported Output Mode %i", (int)output_mode);
		return -1;
	}

	/* try different baudrates */
	const unsigned baudrates_to_try[] = {9600, 38400, 19200, 57600, 115200};


	for (unsigned int baud_i = 0; baud_i < sizeof(baudrates_to_try) / sizeof(baudrates_to_try[0]); baud_i++) {
		baudrate = baudrates_to_try[baud_i];
		setBaudrate(baudrate);

		if (write(comm, sizeof(comm)) != sizeof(comm)) {
			return -1;
		}
	}

	return setBaudrate(115200);
}


#define CRC32_POLYNOMIAL 0xEDB88320L
/* --------------------------------------------------------------------------
Calculate a CRC value to be used by CRC calculation functions.
-------------------------------------------------------------------------- */
unsigned long GPSDriverAshtech::CRC32Value(int i)
{
    int j;
    unsigned long ulCRC;
    ulCRC = i;
    for ( j = 8 ; j > 0; j-- )
    {
        if ( ulCRC & 1 )
         ulCRC = ( ulCRC >> 1 ) ^ CRC32_POLYNOMIAL;
        else
        ulCRC >>= 1;
    }
    return ulCRC;
} /*
--------------------------------------------------------------------------
Calculates the CRC-32 of a block of data all at once
-------------------------------------------------------------------------- */
unsigned long GPSDriverAshtech::CalculateBlockCRC32(unsigned long ulCount, /* Number of bytes in the data block */unsigned char *ucBuffer ) /* Data block */
{
    unsigned long ulTemp1;
    unsigned long ulTemp2;
    unsigned long ulCRC = 0;
    while ( ulCount-- != 0 )
    {
        ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
        ulTemp2 = CRC32Value( ((int) ulCRC ^ *ucBuffer++ ) & 0xff );
        ulCRC = ulTemp1 ^ ulTemp2;
    }
    return( ulCRC );
}
