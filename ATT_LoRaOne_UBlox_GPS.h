/*
* Copyright (c) 2016 SODAQ. All rights reserved.
*
* This file is part of Sodaq_UBlox_GPS.
*
* Sodaq_UBlox_GPS is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published
* by the Free Software Foundation, either version 3 of the License, or (at
* your option) any later version.
*
* Sodaq_UBlox_GPS is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public
* License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Sodaq_UBlox_GPS.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _SODAQ_UBLOX_GPS_H
#define _SODAQ_UBLOX_GPS_H

#include <WString.h>
#include <stdint.h>

class Sodaq_UBlox_GPS
{
public:
    Sodaq_UBlox_GPS();

    void init();
	void startScan(uint32_t start, uint32_t timeout);
	bool tryScan(int nrTries);
	bool scanTimedOut();
	void endScan();
	uint32_t getScanStart();
	
    String getDateTimeString();
    double getLat() { return _lat; }
    double getLon() { return _lon; }
	double getAlt() { return _alt; }
	double getSpeed() { return _speed; }
    uint8_t getNumberOfSatellites() { return _numSatellites; }
    uint16_t getYear() { return (uint16_t)_yy + 2000; }         // 2016..
    uint8_t getMonth() { return _MM; }         // 1..
    uint8_t getDay() { return _dd; }           // 1..
    uint8_t getHour() { return _hh; }          // 0..
    uint8_t getMinute() { return _mm; }        // 0..
    uint8_t getSecond() { return _ss; }        // 0..


    // Sets the optional "Diagnostics and Debug" stream.
    void setDiag(Stream &stream) { _diagStream = &stream; }
    void setDiag(Stream *stream) { _diagStream = stream; }

private:
    void on();
    void off();

    // Read one byte
    uint8_t read();
    bool readLine(uint32_t timeout = 10000);
    bool parseLine(const char * line);
    bool parseGPGGA(const String & line);
    bool parseGPGSA(const String & line);
    bool parseGPRMC(const String & line);
    bool parseGPGSV(const String & line);
    bool parseGPGLL(const String & line);
    bool parseGPVTG(const String & line);
    bool parseGPTXT(const String & line);
    bool computeCrc(const char * line, bool do_logging=false);
    uint8_t getHex2(const char * s, size_t index);
    String num2String(int num, size_t width);
    String getField(const String & data, int index);
    double convertDegMinToDecDeg(const String & data);

    void setDateTime(const String & date, const String & time);

    void beginTransmission();
    void endTransmission();

    void resetValues();

    // The (optional) stream to show debug information.
    Stream *    _diagStream;

    uint8_t     _addr;

    bool        _seenLatLon;
    uint8_t     _numSatellites;
    double      _lat;
    double      _lon;
	double      _alt;
	double      _speed;

    bool        _seenTime;
    uint8_t     _yy;
    uint8_t     _MM;
    uint8_t     _dd;
    uint8_t     _hh;
    uint8_t     _mm;
    uint8_t     _ss;

    bool        _trans_active;

    static const char _fieldSep;
    char *      _inputBuffer;
    size_t      _inputBufferSize;
	
	uint32_t _scanStart;
	uint32_t _scanTimeout;
};

extern Sodaq_UBlox_GPS sodaq_gps;

#endif // _SODAQ_UBLOX_GPS_H
