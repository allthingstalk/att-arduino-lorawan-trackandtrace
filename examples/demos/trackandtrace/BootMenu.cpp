/*
Copyright (c) 2016, SODAQ
All rights reserved.

Reworked by Jan Bogaerts, AllThingsTalk, 2016

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

#include "BootMenu.h"

#include "Command.h"
#include "Config.h"

static bool _runMenu = true;

static bool isTimedOut(uint32_t ts)
{
    return (long)(millis() - ts) >= 0;
}

static void commitAndQuit(const Command* a, const char* line)
{
    params.commit();
	_runMenu = false;
}

static void quitMenu(const Command* a, const char* line)
{
    _runMenu = false;
}

static void reset(const Command* a, const char* line)
{
	params.reset();
}


static const Command args[] = {
	{ "quit", "q", quitMenu, Command::show_string },
    { "save & quit", "s", commitAndQuit, Command::show_string },
	{ "factory reset", "r", reset, Command::show_string }
};

static void showMyCommands(Stream* stream)
{
    size_t nr_cmnds = sizeof(args) / sizeof(args[0]);
    if (nr_cmnds == 0) {
        return;
    }
    stream->println("\r\nCommands:");
    for (size_t i = 0; i < nr_cmnds; ++i) {
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
static bool execCommand(const char* line)
{
    return Command::execCommand(args, sizeof(args) / sizeof(args[0]), line);
}

static int readLine(Stream* stream, char line[], size_t size)
{
    int c;
    size_t len = 0;
    bool seenCR = false;
    uint32_t ts_waitLF = 0;
	
	bool found = false;
    while (!found) {
        if (seenCR) {
            c = stream->peek();
            // ts_waitLF is guaranteed to be non-zero
            if ((c == -1 && isTimedOut(ts_waitLF)) || (c != -1 && c != '\n')) {		//could be \r\n or \r, after that comes \LF
                found = true;
				break;
            }
        }
        c = stream->read();
        if (c <= 0)								// Ignore NUL bytes too
            continue;
        if (c == '\r') {
			seenCR = true;
            stream->write((char)c);
            ts_waitLF = millis() + 50; 			// Wait another .05 sec for an optional LF
        }
        else if (c == '\n') {
            stream->write((char)c);
            found = true;
			break;
        }
        else if (c == 0x08 || c == 0x7f) {		// Erase the last character
            if (len > 0) {
                stream->write("\b \b");
                --len;
            }
        }
        else {
            if (len < size - 1) {				// Any "normal" character is stored in the line buffer
                if (c >= ' ' && c < 0x7f) {
                    stream->write((char)c);
                    line[len++] = c;
                }
            }
        }
    }
    line[len] = '\0';							//found input
	return len == 0 ? -1 : len;
}

static void showCommandPrompt(Stream* stream)
{
	stream->write(27);   		//Print "esc"
	stream->print("[2J");		// clear screen command
	stream->write(27);
	stream->print("[H");     	// cursor to home command
	stream->println("LoRaWAN track and trace menu");
	stream->println("============================");
    showMyCommands(stream);
    ConfigParams::showConfig(stream);
    stream->print("Enter command: ");
}

void showBootMenu(Stream& stream)
{
    char buffer[200 + 1];
    int size;
    bool needPrompt;
	int scanCount = 0;						//keep track of how many times we scaned the same screen. When too high, do a page refresh.

    needPrompt = true;
	_runMenu = true;
    while (_runMenu) {
        if (needPrompt) {
            showCommandPrompt(&stream);
            needPrompt = false;
			scanCount = 0;
        }

        size = -1;
        while (stream.available() && stream.peek() == 0) {
            stream.read(); // Ignore initial NUL bytes on input
        }

        if (stream.available()) {
            size = readLine(&stream, buffer, sizeof(buffer));
        }
        if (strcasecmp(buffer, "ok") == 0) {
            break;
        }

		if(size > 0){
			// Is this a command for us?
			if (!execCommand(buffer)) {
				if(params.execCommand(buffer))
					needPrompt = true;
			}
			else{
				needPrompt = true;
			}
		}
		else{
			scanCount++;
			needPrompt = scanCount > 1999999;
		}
    }
    stream.println();
	stream.println("configuration done");
}
