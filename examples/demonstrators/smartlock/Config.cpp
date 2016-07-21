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

#include "Config.h"
#include "Command.h"
#include "FlashStorage.h"

#define DEFAULT_HEADER 0xBEEF


#define DEBUG

#ifndef DEBUG
	#define MOVING_WAKEUP_EVERY_SEC 59              //seconds part of the clock that wakes up the device while moving, to verity if the device is still moving.
	#define WAKEUP_EVERY_MIN 4                      //minutes part of the clock that wakes up the device while moving, to verity if the device is still moving.
#else
	#define MOVING_WAKEUP_EVERY_SEC 30              
	#define WAKEUP_EVERY_MIN 0                      
#endif



#define ACCELERO_SENSITIVY 4                    //sensitivity of the accelerometer when in deep sleep mode (waiting for first movment), the closer to 0, the more sensitive. Each point represent 16 milli g force.
#define GPS_FIX_TIMEOUT 360                    //nr of seconds after which the GPS will time out and no GPS position will be sent.

ConfigParams params;

FlashStorage(flash, ConfigParams);
static bool needsCommit;
static VoidCallbackMethodPtr configResetCallback;

static uint16_t crc16ccitt(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0;
    while (len--) {
        crc ^= (*buf++ << 8);
        for (uint8_t i = 0; i < 8; ++i) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            }
            else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

void ConfigParams::read()
{
    flash.read(this);

    // check header and CRC
    uint16_t calcCRC16 = crc16ccitt((uint8_t*)this, sizeof(ConfigParams) - sizeof(_crc16));
    if (_header != DEFAULT_HEADER || _crc16 != calcCRC16) {
        reset();
    }
}

void ConfigParams::reset()
{
    _fixIntervalMinutes = WAKEUP_EVERY_MIN;
    _fixIntervalSeconds = MOVING_WAKEUP_EVERY_SEC;
    _gpsFixTimeout = GPS_FIX_TIMEOUT;
    _acceleroSensitivity = ACCELERO_SENSITIVY;
    _isLedEnabled = true; 
	_useAccelero = true;
	
    memset(_devAddrOrEUI, 0x30, sizeof(_devAddrOrEUI) - 1);
    _devAddrOrEUI[sizeof(_devAddrOrEUI) - 1] = '\0';

    memset(_appSKeyOrEUI, 0x30, sizeof(_appSKeyOrEUI) - 1);
    _appSKeyOrEUI[sizeof(_appSKeyOrEUI) - 1] = '\0';

    memset(_nwSKeyOrAppKey, 0x30, sizeof(_nwSKeyOrAppKey) - 1);
    _nwSKeyOrAppKey[sizeof(_nwSKeyOrAppKey) - 1] = '\0';

    needsCommit = true;
}

bool ConfigParams::keysAndAddrSpecified()
{
	bool netKey = false;
	bool appKey = false;
	bool addrOk = false;
	for(int i = 0; i < sizeof(_devAddrOrEUI) -1; i++){
		if( _devAddrOrEUI[i] != 0x30){
			addrOk = true;
			break;
		}
	}
	for(int i = 0; i < sizeof(_appSKeyOrEUI) -1; i++){
		if( _appSKeyOrEUI[i] != 0x30){
			appKey = true;
			break;
		}
	}
	for(int i = 0; i < sizeof(_nwSKeyOrAppKey) - 1; i++){
		if( _nwSKeyOrAppKey[i] != 0x30){
			netKey = true;
			break;
		}
	}
	return addrOk && appKey && netKey;
}

/*
 * Write the configuration parameters to NVM / Dataflash
 */
void ConfigParams::commit(bool forced)
{
    if (!forced && !needsCommit) {
        return;
    }

    _header = DEFAULT_HEADER;
    _crc16 = crc16ccitt((uint8_t*)this, sizeof(ConfigParams) - sizeof(_crc16));

    flash.write(*this);

    needsCommit = false;
}

static const Command args[] = {
	{ "Use accelerometer (OFF=0 / ON=1)", "ac=", Command::set_uint8, Command::show_uint8, &params._useAccelero },
	{ "Accelerometer sensitivity (1-10)", "acs=", Command::set_uint8, Command::show_uint8, &params._acceleroSensitivity },
	{ "Fix Interval (seconds part)     ", "fis=", Command::set_uint8, Command::show_uint8, &params._fixIntervalSeconds },
    { "Fix Interval (minutes part)     ", "fim=", Command::set_uint8, Command::show_uint8, &params._fixIntervalMinutes },
    { "GPS Fix Timeout (sec)           ", "gft=", Command::set_uint16, Command::show_uint16, &params._gpsFixTimeout },
	
    { "DevAddr / DevEUI                ", "dev=", Command::set_string, Command::show_string, params._devAddrOrEUI, sizeof(params._devAddrOrEUI) },
    { "AppSKey / AppEUI                ", "app=", Command::set_string, Command::show_string, params._appSKeyOrEUI, sizeof(params._appSKeyOrEUI) },
    { "NWSKey / AppKey                 ", "key=", Command::set_string, Command::show_string, params._nwSKeyOrAppKey, sizeof(params._nwSKeyOrAppKey) },

    { "Status LED (OFF=0 / ON=1)       ", "led=", Command::set_uint8, Command::show_uint8, &params._isLedEnabled }
};

void ConfigParams::showConfig(Stream* stream)
{
    stream->println();
    stream->println("Settings:");
    for (size_t i = 0; i < sizeof(args) / sizeof(args[0]); ++i) {
        const Command* a = &args[i];
        if (a->show_func) {
            a->show_func(a, stream);
        }
    }
}

/*
 * Execute a command from the commandline
 *
 * Return true if it was a valid command
 */
bool ConfigParams::execCommand(const char* line)
{
    bool done = Command::execCommand(args, sizeof(args) / sizeof(args[0]), line);
    if (done) {
        needsCommit = true;
    }

    return done;
}

/*
 * Check if all required config parameters are filled in
 */
bool ConfigParams::checkConfig(Stream& stream)
{
    bool fail = false;
	
	stream.write(27);   		//Print "esc"
	stream.print("[2J");		// clear screen command
	stream.write(27);
	stream.print("[H");     	// cursor to home command
	
	if (_useAccelero > 1) {
        stream.println("\n\nERROR: \"ac\" must be either 0 or 1");
        fail = true;
    }
	
	if (_isLedEnabled > 1) {
        stream.println("\n\nERROR: \"led\" must be either 0 or 1");
        fail = true;
    }

    if (_fixIntervalMinutes > 59) {
        stream.println("\n\nERROR: \"fim. (minutes part)\" must not be more than 59");
        fail = true;
    }

    if (_fixIntervalSeconds > 59) {
        stream.println("\n\nERROR: \"fis. (seconds part)\" must not be more than 59");
        fail = true;
    }


    if (_acceleroSensitivity > 10) {
        stream.println("\n\nERROR: \"Alt. (1-10)\" must not be more than 10");
        fail = true;
    }

	if(fail)
		delay(7000);
    return !fail;
}

