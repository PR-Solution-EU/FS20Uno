/* ===================================================================
 * FS20Uno Helper Functions
 * ===================================================================*/

// Max output size of SerialPrintf* functions
#define MAX_PRINTF_BUFFER	160

// If using strncasecmp_P direct in code compiler reports warnings for
// F() macro. So we use strnicmp_P() macro with same paramater without
// warnings
#define strnicmp_P(str1, str2, n) strncasecmp_P(str1, (const char *)str2, n)

/* ===================================================================
 * Function:    strReplaceChar
 * Return:
 * Arguments:	s        string where to replace char
 *              find     char to find
 *              replace  char to replace
 * Description: Replace all chars <find> with char <replace>
 *              within a given string
 * ===================================================================*/
void strReplaceChar(char *s, char find, char replace)
{
    while (*s != 0) {
        if (*s == find)
        *s = replace;
        s++;
    }
}

/* *******************************************************************
 * HIGH-LEVEL Functions
 * ********************************************************************/

/* ===================================================================
 * Function:    printProgramInfo
 * Return:
 * Arguments:	copyright - print copyright info too if true
 * Description: Print program info
 * ===================================================================*/
void printProgramInfo(bool copyright)
{
	SerialPrintfln(F("%S v%S (build %S)"), PROGRAM, VERSION, REVISION);
	SerialPrintfln(F("compiled on %s %s (GnuC%S %s)"), __DATE__, __TIME__, __GNUG__?F("++ "):F(" "), __VERSION__);
	SerialPrintfln(F("using avr library %s (%s)"),  __AVR_LIBC_VERSION_STRING__, __AVR_LIBC_DATE_STRING__);
	if( copyright ) {
		SerialPrintfln(F("(c) 2016 www.p-r-solution.de - Norbert Richter <norbert.richter@p-r-solution.de>"));
	}
}


/* ===================================================================
 * Function:    watchdogInit
 * Return:
 * Arguments:
 * Description: Init watchdog
 * ===================================================================*/
void watchdogInit(void)
{
	#ifdef WATCHDOG_ENABLED
	#ifndef DEBUG_WATCHDOG
	Watchdog.enable(WATCHDOG_TIME);
	#else
	int countdownMS = Watchdog.enable(WATCHDOG_TIME);
	SerialTimePrintfln(F("setup - Enabled the watchdog with max countdown of %d ms"), countdownMS);
	#endif
	#endif

}

/* ===================================================================
 * Function:    watchdogReset
 * Return:
 * Arguments:
 * Description: Reset watchdog
 * ===================================================================*/
void watchdogReset(void)
{
	#ifdef WATCHDOG_ENABLED
	Watchdog.reset();
	#endif
}

/* ===================================================================
 * Function:    cmdGetString
 * Return:		number of character read
 * Arguments:	buf       pointer to the char buffer to store the chars
 *              buflen    max number of chars to read
 *              term      terminate character
 *              echo      true: echos input char
 *              hideEcho  true: display an asterisk instead of input char
 * Description: Reads an input string from serial interface until
 *              a terminate character is read
 * ===================================================================*/
size_t SerialGetString(char *buf, size_t buflen, char term, bool echo, bool hideEcho)
{
	char inChar;
	char *ptr = buf;

	while ( true ) {
		watchdogReset();

		while (Serial.available() > 0) {
			inChar=Serial.read();   // Read single available character, there may be more waiting

			if ( echo ) {
				if( hideEcho ) {
					Serial.print(F("*"));
				}
				else {
					Serial.print(inChar);
				}
			}
			if ( inChar==term ) {
				return (ptr - buf);
			}
			else {
				if ( (size_t)(ptr - buf) < buflen ) {
					*ptr++ = inChar;
				}
			}
		}
	}
}

/* ===================================================================
 * Function:    vaSerialPrint
 * Return:
 * Arguments:	printf arguments
 * Description: Serial output message
 * ===================================================================*/
void vaSerialPrint(const __FlashStringHelper *fmt, va_list argp)
{
	char buf[MAX_PRINTF_BUFFER]; // resulting string limited to 128 chars

	#ifdef __AVR__
	// progmem for AVR
	vsnprintf_P(buf, sizeof(buf), (const char *)fmt, argp);
	#else
	// for the rest of the world
	vsnprintf(buf, sizeof(buf), (const char *)fmt, argp);
	#endif
	Serial.print(buf);
	watchdogReset();
}

/* ===================================================================
 * Function:    printCRLF
 * Return:
 * Arguments:	
 * Description: Serial output new line
 * ===================================================================*/
void printCRLF()
{
	Serial.println();
}


/* ===================================================================
 * Function:    SerialPrintf
 * Return:
 * Arguments:	printf arguments
 * Description: Serial output message
 * ===================================================================*/
void SerialPrintf(const __FlashStringHelper *fmt, ... )
{
	va_list args;
	va_start (args, fmt);
	vaSerialPrint(fmt, args);
	va_end(args);
}

/* ===================================================================
 * Function:    SerialPrintfln
 * Return:
 * Arguments:	printf arguments
 * Description: Serial output message with new line
 * ===================================================================*/
void SerialPrintfln(const __FlashStringHelper *fmt, ... )
{
	va_list args;
	va_start (args, fmt);
	vaSerialPrint(fmt, args);
	va_end(args);
	printCRLF();
}

/* ===================================================================
 * Function:    sec
 * Return:
 * Arguments:	Pointer to a unsigned short variable or NULL
 * Description: Returns the number of sec system is up
 *              If milli is not NULL, returns also ms frac
 * ===================================================================*/
unsigned long sec(uint16_t *milli)
{
	unsigned long t;

	/* max of millis() (unsigned long) is 49 days 17:02:47.295 */
	t = millis();

	/* return ms, if return var pointer exists */
	if ( milli != NULL ) {
		*milli = (uint16_t)(t % 1000UL);
	}
	// t in sec
	t /= 1000;
	/* Compressed the current uptime to 3 bytes:
	 * 4294967295                  = 0xFFFFFFFF
	 * 4294967295 / 1000 = 4294967 = 0x418937
	 * Now we are able to add the days
	 * Overflow of running time will be after ~134 years */
	return t + ((unsigned long)millisOverflow * 4294967L);
}


/* ===================================================================
 * Function:    SerialPrintUptime
 * Return:
 * Arguments:
 * Description: Print system uptime
 * ===================================================================*/
void SerialPrintUptime(void)
{
	uint32_t t, ot;
	uint16_t days, milli;
	uint8_t hours, minutes, seconds;
	char timebuf[48];

	t = ot = sec(&milli);
	days    = (uint16_t)(t      / (24UL * 60UL * 60UL));
	t      -= (uint32_t)days    * (24UL * 60UL * 60UL);
	hours   = (uint8_t) (t      / (60UL * 60UL));
	t      -= (uint32_t)hours   * (60UL * 60UL);
	minutes = (uint8_t) (t      / (60UL));
	t      -= (uint32_t)minutes * (60UL);
	seconds = (uint8_t) t;

	if ( days ) {
		snprintf_P(timebuf, sizeof(timebuf), PSTR("%d day%S "), days, days==1?F(""):F("s"));
		Serial.print(timebuf);
	}
	snprintf_P(timebuf, sizeof(timebuf), PSTR("%02d:%02d:%02d.%-3d (%ld%03d ms) "), hours, minutes, seconds, milli, ot, milli);
	Serial.print(timebuf);
}

/* ===================================================================
 * Function:    SerialTimePrintf
 * Return:
 * Arguments:	printf arguments
 * Description: Serial output message with timestamp
 * ===================================================================*/
void SerialTimePrintf(const __FlashStringHelper *fmt, ... )
{
	SerialPrintUptime();

	va_list args;
	va_start (args, fmt);
	vaSerialPrint(fmt, args);
	va_end(args);
}

/* ===================================================================
 * Function:    SerialTimePrintfln
 * Return:
 * Arguments:	printf arguments
 * Description: Serial output message with timestamp and newline
 * ===================================================================*/
void SerialTimePrintfln(const __FlashStringHelper *fmt, ... )
{
	SerialPrintUptime();

	va_list args;
	va_start (args, fmt);
	vaSerialPrint(fmt, args);
	va_end(args);
	printCRLF();
}

/* ===================================================================
 * Function:    sendStatus
 * Return:
 * Arguments:	send  if true, message will be send regardless
 *                    if eeprom.SendStatus is false
 *              type  statusType enum
 *              fmt   variable format string like printf
 * Description: Send status using serial interface
 *              type    Status type text output
 *                      0: SYSTEM
 *                      1: MOTOR 01..xx
 *                      2: FS20OUT 01..xx
 *                      3: FS20IN 01..xx
 *                      4: PUSHBUTTON 01..xx
 *                      5: RAIN
 * ===================================================================*/
void sendStatus(bool send, statusType type, const __FlashStringHelper *fmt, ... )
{
	if( send || eeprom.SendStatus ) {
		switch (type) {
		case SYSTEM:
			Serial.print(F("0 SYSTEM "));
			break;
		case MOTOR:
			Serial.print(F("1 MOTOR "));
			break;
		case FS20OUT:
			Serial.print(F("2 FS20OUT "));
			break;
		case FS20IN:
			Serial.print(F("3 FS20IN "));
			break;
		case PUSHBUTTON:
			Serial.print(F("4 PUSHBUTTON "));
			break;
		case RAIN:
			Serial.print(F("5 RAIN "));
			break;
		}
		watchdogReset();
		
		va_list args;
		va_start (args, fmt);
		vaSerialPrint(fmt, args);
		va_end(args);
		printCRLF();
	}
}

/* ===================================================================
 * Function:    sendMotorStatus
 * Return:
 * Arguments:	send   if true, message will be send regardless
 *                     if eeprom.SendStatus is false
 *              motor  motor number 0..x
 * Description: Send motor OFF status for a given motor from
 *              IRQ subroutine
 * ===================================================================*/
void sendMotorStatus(bool send, int motor)
{
	byte runTimePercent = (byte)((long)MotorPosition[motor]*100L / (long)(eeprom.MaxRuntime[motor] / TIMER_MS));

	if( runTimePercent<1 && MotorPosition[motor]>0 ) {
		runTimePercent=1;
	}
	if( runTimePercent>100 ) {
		runTimePercent=100;
	}
	sendStatus(send,MOTOR,F("%02d %-7S %3d%% %-7S (%s)")
				,motor+1
				,runTimePercent==0?F("CLOSE"):(runTimePercent==100?F("OPEN"):F("BETWEEN"))
				,runTimePercent
				,getMotorDirection(motor)==MOTOR_OFF?F("OFF"):(getMotorDirection(motor)>=MOTOR_OPEN)?F("OPENING"):F("CLOSING")
				,(char *)eeprom.MotorName[motor]);
}

/* ===================================================================
 * Function:     sendMotorOffStatus
 * Return:
 * Arguments:
 * Description:  Send motor OFF status for all motors from
 *               IRQ subroutine
 * ===================================================================*/
void sendMotorOffStatus(void)
{
	for (byte i = 0; i < MAX_MOTORS; i++) {
		if ( bitRead(sendStatusMOTOR_OFF, i) ) {
			sendMotorStatus(false,i);
			bitClear(sendStatusMOTOR_OFF, i);
		}
	}
}

/* ===================================================================
 * Function:    setMotorPosition
 * Return:
 * Arguments:   motorNum    - motor # [0..x]
 *              destPercent - destinaton position in percent [0..100]
 *                              0% means CLOSE
 *                            100% means OPEN
 * Description: Set motor to a defined percent position
 * ===================================================================*/
void setMotorPosition(byte motorNum, byte destPercent)
{
	#ifdef DEBUG_MOTOR
	SerialTimePrintfln(F("setMotorPosition- Motor %d current pos=%d, destPercent=%d%%"), motorNum+1, MotorPosition[motorNum], destPercent);
	#endif
	if ( destPercent>0 && destPercent<100 ) {
		destMotorPosition[motorNum] = (MOTOR_TIMER)((long)(eeprom.MaxRuntime[motorNum] / TIMER_MS) * (long)destPercent / 100L);

		#ifdef DEBUG_MOTOR
		SerialTimePrintfln(F("setMotorPosition- Motor %d current pos=%d, destMotorPosition=%d"), motorNum+1, MotorPosition[motorNum], destMotorPosition[motorNum]);
		#endif

		if ( destMotorPosition[motorNum] > MotorPosition[motorNum] ) {
			setMotorDirection(motorNum, MOTOR_OPEN);
		}
		else {
			setMotorDirection(motorNum, MOTOR_CLOSE);
		}
	}
	else if ( destPercent==0 ) {
		setMotorDirection(motorNum, MOTOR_CLOSE);
	}
	else if ( destPercent>=100 ) {
		setMotorDirection(motorNum, MOTOR_OPEN);
	}
}

/* ===================================================================
 * Function:    setMotorDirection
 * Return:
 * Arguments:   motorNum      motor # [0..x]
 *              newDirection  MOTOR_OPEN
 *                            MOTOR_CLOSE
 *                            MOTOR_OFF
 * Description: Set motor direction
 * ===================================================================*/
void setMotorDirection(byte motorNum, MOTOR_CTRL newDirection)
{
	if ( motorNum < MAX_MOTORS ) {

		// Neue Richtung: Öffnen
		if ( newDirection >= MOTOR_OPEN ) {
			// Motor läuft auf Schliessen
			if (MotorCtrl[motorNum] <= MOTOR_CLOSE) {
				// Motor auf Öffnen mit Umschaltdelay
				MotorCtrl[motorNum] = MOTOR_OPEN_DELAYED;
				sendMotorStatus(false,motorNum);
			}
			// Motor läuft auf Öffnen
			else if (MotorCtrl[motorNum] >= MOTOR_OPEN) {
				// Motor aus
				MotorCtrl[motorNum] = MOTOR_OFF;
				sendMotorStatus(false,motorNum);
			}
			// Motor ist aus
			else {
				// Motor auf öffnen ohne Umschaltdelay
				MotorCtrl[motorNum] = MOTOR_OPEN;
				sendMotorStatus(false,motorNum);
			}
		}
		// Neue Richtung: Schliessen
		else if ( newDirection <= MOTOR_CLOSE ) {
			// Motor läuft auf Öffnen
			if (MotorCtrl[motorNum] >= MOTOR_OPEN) {
				// Motor auf Schliessen mit Umschaltdelay
				MotorCtrl[motorNum] = MOTOR_CLOSE_DELAYED;
				sendMotorStatus(false,motorNum);
			}
			// Motor läuft auf Schliessen
			else if (MotorCtrl[motorNum] <= MOTOR_CLOSE) {
				// Motor aus
				MotorCtrl[motorNum] = MOTOR_OFF;
				sendMotorStatus(false,motorNum);
			}
			// Motor ist aus
			else {
				// Motor auf Schliessen ohne Umschaltdelay
				MotorCtrl[motorNum] = MOTOR_CLOSE;
				sendMotorStatus(false,motorNum);
			}
		}
		// Neue Richtung: AUS
		else {
			// Motor AUS
			MotorCtrl[motorNum] = MOTOR_OFF;
			sendMotorStatus(false,motorNum);
		}
	}
}

/* ===================================================================
 * Function:    getMotorDirection
 * Return:		Motor direction:
 *                MOTOR_OPEN,  MOTOR_OPEN_DELAYED
 *                MOTOR_CLOSE, MOTOR_CLOSE_DELAYED
 *                MOTOR_OFF
 * Arguments:   motorNum      motor # [0..x]
 * Description: Get motor direction
 * ===================================================================*/
char getMotorDirection(byte motorNum)
{
	if (MotorCtrl[motorNum] == MOTOR_OPEN) {
		return MOTOR_OPEN;
	}
	else if (MotorCtrl[motorNum] > MOTOR_OPEN) {
		return MOTOR_OPEN_DELAYED;
	}
	else if (MotorCtrl[motorNum] == MOTOR_CLOSE) {
		return MOTOR_CLOSE;
	}
	else if (MotorCtrl[motorNum] < MOTOR_CLOSE) {
		return MOTOR_CLOSE_DELAYED;
	}
	return MOTOR_OFF;
}

/* ===================================================================
 * Function:    setMotorType
 * Return:
 * Arguments:   motorNum   motor # [0..x]
 *              mType      type (WINDOW, JALOUSIE)
 * Description: Set motor type
 * ===================================================================*/
void setMotorType(byte motorNum, mtype mType)
{
	if( mType == WINDOW ) {
		bitSet(eeprom.MTypeBitmask, motorNum);
	}
	else {
		bitClear(eeprom.MTypeBitmask, motorNum);
	}
}

/* ===================================================================
 * Function:    getMotorType
 * Return:      mtype      type (WINDOW, JALOUSIE)
 * Arguments:   motorNum   motor # [0..x]
 *              
 * Description: Get motor type
 * ===================================================================*/
mtype getMotorType(byte motorNum)
{
	if ( bitRead(eeprom.MTypeBitmask, motorNum) ) {
		return WINDOW;
	}
	else {
		return JALOUSIE;
	}
}
