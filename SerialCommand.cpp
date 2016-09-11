/*******************************************************************************
SerialCommand - An Arduino library to tokenize and parse commands received over
a serial port.
Copyright (C) 2011-2013 Steven Cogswell  <steven.cogswell@gmail.com>
http://awtfy.com

See SerialCommand.h for version history.

This library is free software; you can redistribute it and/or
modify it under the DEFAULT_TERMINATORs of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
***********************************************************************************/

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "SerialCommand.h"


#include <string.h>
#ifndef SERIALCOMMAND_HARDWAREONLY
#include <SoftwareSerial.h>
#endif


// Constructor makes sure some things are set.
SerialCommand::SerialCommand()
{
	usingSoftwareSerial=0;
	strncpy_P(delim, PSTR(" "), sizeof(delim)-1);  // strtok_r needs a null-DEFAULT_TERMINATORinated string
	term=DEFAULT_TERMINATOR;   // return character, default DEFAULT_TERMINATORinator for commands
	isEcho=DEFAULT_ECHO;	// echo flag
	numCommand=0;    		// Number of callback handlers installed
	clearBuffer();
}

#ifndef SERIALCOMMAND_HARDWAREONLY
// Constructor to use a SoftwareSerial object
SerialCommand::SerialCommand(SoftwareSerial &_SoftSer)
{
	usingSoftwareSerial=1;
	SoftSerial = &_SoftSer;
	strncpy_P(delim, PSTR(" "), sizeof(delim)-1);  // strtok_r needs a null-DEFAULT_TERMINATORinated string
	term=DEFAULT_TERMINATOR;   // return character, default DEFAULT_TERMINATORinator for commands
	isEcho=DEFAULT_ECHO;	// echo flag
	numCommand=0;    // Number of callback handlers installed
	clearBuffer();
}
#endif


//
// Initialize the command buffer being processed to all null characters
//
void SerialCommand::clearBuffer()
{
	memset(buffer, '\0', sizeof(buffer));
	bufPos=0;
}

// Retrieve the next token ("word" or "argument") from the Command buffer.
// returns a NULL if no more tokens exist.
char *SerialCommand::next()
{
	char *nextToken;
	nextToken = strtok_r(NULL, delim, &last);
	return nextToken;
}

// This checks the Serial stream for characters, and assembles them into a buffer.
// When the DEFAULT_TERMINATORinator character (default '\r') is seen, it starts parsing the
// buffer for a prefix command, and calls handlers setup by addCommand() member
void SerialCommand::readSerial()
{
	// If we're using the Hardware port, check it.   Otherwise check the user-created SoftwareSerial Port
	#ifdef SERIALCOMMAND_HARDWAREONLY
	while (Serial.available() > 0)
	#else
	while ((usingSoftwareSerial==0 && Serial.available() > 0) || (usingSoftwareSerial==1 && SoftSerial->available() > 0) )
	#endif
	{
		int i;
		boolean matched;
		if (usingSoftwareSerial==0) {
			// Hardware serial port
			inChar=Serial.read();   // Read single available character, there may be more waiting
		} else {
			#ifndef SERIALCOMMAND_HARDWAREONLY
			// SoftwareSerial port
			inChar = SoftSerial->read();   // Read single available character, there may be more waiting
			#endif
		}
		if ( isEcho ) {
			// Echo back to serial stream
			Serial.print(inChar);
		}
		if (inChar==term) {     // Check for the DEFAULT_TERMINATORinator (default '\r') meaning end of command
			if ( isEcho ) {
				Serial.println();   // Echo back to serial stream
			}
			#ifdef SERIALCOMMANDDEBUG
			Serial.print(F("Received: "));
			Serial.println(buffer);
		    #endif
			bufPos=0;           // Reset to start of buffer
			token = strtok_r(buffer,delim,&last);   // Search for command at start of buffer
			if (token == NULL) return;
			matched=false;
			for (i=0; i<numCommand; i++) {
				#ifdef SERIALCOMMANDDEBUG
				Serial.print(F("Comparing ["));
				Serial.print(token);
				Serial.print(F("] to ["));
				Serial.print(CommandList[i].command);
				Serial.println(F("]"));
				#endif
				// Compare the found command against the list of known commands for a match
				if (strncasecmp(token,CommandList[i].command,MAXSERIALCOMMANDLEN) == 0)
				{
					#ifdef SERIALCOMMANDDEBUG
					Serial.print(F("Matched Command: "));
					Serial.println(token);
					#endif
					// Execute the stored handler function for the command
					(*CommandList[i].function)();
					clearBuffer();
					matched=true;
					break;
				}
			}
			if (matched==false) {
				(*defaultHandler)();
				clearBuffer();
			}

		}
		if (isprint(inChar) )   // Only printable characters into the buffer
		{
			if ( (size_t)bufPos < sizeof(buffer)-1 ) {
				buffer[bufPos++]=inChar;   	// Put character into buffer
				buffer[bufPos]='\0';  		// Null DEFAULT_TERMINATORinate
			}
		}
	}
}

// Adds a "command" and a handler function to the list of available commands.
// "command" is a pointer to PROGMEM.
// This is used for matching a found token in the buffer, and gives the pointer
// to the handler function to deal with it.
void SerialCommand::addCommand(const char *command, void (*function)())
{
	if (numCommand < MAXSERIALCOMMANDS) {
		#ifdef SERIALCOMMANDDEBUG
		Serial.print(numCommand);
		Serial.print(F("-"));
		Serial.print(F("Adding command for "));
		Serial.println(command);
		#endif

		strncpy_P(CommandList[numCommand].command, command, MAXSERIALCOMMANDLEN);
		CommandList[numCommand].function = function;
		numCommand++;
	} else {
		// In this case, you tried to push more commands into the buffer than it is compiled to hold.
		// Not much we can do since there is no real visible error assertion, we just ignore adding
		// the command
		#ifdef SERIALCOMMANDDEBUG
		Serial.println(F("Too many handlers - recompile changing MAXSERIALCOMMANDS"));
		#endif
	}
}

// Set the function to check if echo is enabled
void SerialCommand::setEcho(unsigned char fEcho)
{
	isEcho = fEcho;
}

// Set the function to check if echo is enabled
void SerialCommand::setTerm(char cTerm)
{
	term = cTerm;
}

// Get the function to check if echo is enabled
char SerialCommand::getTerm(void)
{
	return term;
}

// This sets up a handler to be called in the event that the receveived command string
// isn't in the list of things with handlers.
void SerialCommand::addDefaultHandler(void (*function)())
{
	defaultHandler = function;
}
