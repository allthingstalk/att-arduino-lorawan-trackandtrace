/*
Copyright (c) 2016, SODAQ
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef CONFIG_H_
#define CONFIG_H_

#include <stdint.h>
#include <Arduino.h>

typedef void(*VoidCallbackMethodPtr)(void);

struct ConfigParams
{
    uint16_t _header;

    uint8_t _fixIntervalMinutes;
    uint8_t _fixIntervalSeconds;
    uint16_t _gpsFixTimeout;
	uint8_t _acceleroSensitivity;

	uint8_t _useAccelero;
    uint8_t _isLedEnabled;
	uint8_t _isDebugMode;
    
    char _devAddrOrEUI[16 + 1];
    char _appSKeyOrEUI[32 + 1];
    char _nwSKeyOrAppKey[32 + 1];


    uint16_t _crc16;

public:
    void read();
    void commit(bool forced = false);
    void reset();

    bool execCommand(const char* line);

    uint8_t getFixIntervalMinutes() const { return _fixIntervalMinutes; }
    uint8_t getFixIntervalSeconds() const { return _fixIntervalSeconds; }
	uint16_t getGpsFixTimeout() const { return _gpsFixTimeout; }
	uint8_t getAcceleroSensitivity() const { return _acceleroSensitivity; }

    bool getIsLedEnabled() const { return _isLedEnabled; } 
	bool getUseAccelero() const { return _useAccelero; }
	bool getIsDebugMode() const { return _isDebugMode; }
    
    const char* getDevAddrOrEUI() const { return _devAddrOrEUI; }
    const char* getAppSKeyOrEUI() const { return _appSKeyOrEUI; }
    const char* getNwSKeyOrAppKey() const { return _nwSKeyOrAppKey; }
	
	//returns true when devAddr, network and application session keys have been specified.
	bool keysAndAddrSpecified();

    static void showConfig(Stream* stream);
    bool checkConfig(Stream& stream);
};

extern ConfigParams params;

#endif
