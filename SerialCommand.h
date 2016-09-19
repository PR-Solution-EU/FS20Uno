/***********************************************************************
 * Fork of SerialCommand Arduino library
 * from Steven Cogswell  <steven.cogswell@gmail.com>
 * http://awtfy.com
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the DEFAULT_TERMINATORs of the
 * GNU Lesser General Public License as published by the
 * Free Software Foundation; either version 2.1 of the License, or
 * (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301 USA
 **********************************************************************/
#ifndef SerialCommand_h
#define SerialCommand_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <string.h>

// If you want to use SerialCommand with the hardware serial port only, and want to disable
// SoftwareSerial support, and thus don't have to use "#include <SoftwareSerial.h>" in your
// sketches, then uncomment this define for SERIALCOMMAND_HARDWAREONLY, and comment out the
// corresponding #undef line.
#define SERIALCOMMANDBUFFER 64		// Len of the buffer which stores the received input
#define MAXSERIALCOMMANDLEN 9		// max len of the command name
#define MAXSERIALCOMMANDS	21		// max number of commands which can be defined
#define MAXDELIMETER		3		// max. number of delimiters

#define DEFAULT_TERMINATOR	'\r'
#define DEFAULT_ECHO		false

class SerialCommand
{
	public:
		SerialCommand();      // Constructor

		void clearBuffer();   // Sets the command buffer to all '\0' (nulls)
		char *next();         // returns pointer to next token found in command buffer (for getting arguments to commands)
		void readSerial();    // Main entry point.
		void addCommand(const char *, void(*)());   // Add commands to processing dictionary
		void setEcho(unsigned char fEcho);
		void setTerm(char cTerm);
		char getTerm(void);
		void addDefaultHandler(void (*function)(char *));    // A handler to call when no valid command received.

	private:
		char inChar;          				// A character read from the serial stream
		char buffer[SERIALCOMMANDBUFFER];   // Buffer of stored characters while waiting for DEFAULT_TERMINATORinator character
		int  bufPos;                        // Current position in the buffer
		char delim[MAXDELIMETER];           // null-DEFAULT_TERMINATORinated list of character to be used as delimeters for tokenizing (default " ")
		char term;            				// Character that signals end of command (default '\r')
		unsigned char isEcho;				// true if echo is available
		char *token;                        // Returned token from the command buffer as returned by strtok_r
		char *last;                         // State variable used by strtok_r during processing
		typedef struct _callback {
			char command[MAXSERIALCOMMANDLEN+1];
			void (*function)();
		} SerialCommandCallback;            // Data structure to hold Command/Handler function key-value pairs
		int numCommand;
		SerialCommandCallback CommandList[MAXSERIALCOMMANDS];   // Actual definition for command/handler array
		void (*defaultHandler)(char*);           // Pointer to the default handler function
		int usingSoftwareSerial;            // Used as boolean to see if we're using SoftwareSerial object or not
};

#endif //SerialCommand_h
