/* ===================================================================
 * FS20Uno Helper Functions
 * ===================================================================*/

#define MAX_PRINTF_BUFFER	160
#define strnicmp(str1, str2, n) strncasecmp_P(str1, (const char *)str2, n)

/* *******************************************************************
 * LOW-LEVEL Functions
 * ********************************************************************/

/* ===================================================================
 * Function:    strnicmp
 * Return:      Returns a negative value if str1<str2;
 *              0 if str1 and str2 are identical;
 *              and positive value if str1>str2.
 * Arguments:	str1, str2 - strings to comparre
 * Description: compares at most n characters of str2 to str1,
 *              lexicographically, by ignoring case.
 * ===================================================================*/
//~ int strnicmp(const char *str1, const __FlashStringHelper *Fstr2, size_t Count)
//~ {
	//~ return strncasecmp_P(str1, (const char *)Fstr2, Count);
//~ }


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
	SerialPrintf(F("\r\n%s v%s (build %s)\r\n"), PROGRAM, VERSION, REVISION);
	SerialPrintf(F("compiled on %s %s (GnuC%s %s)\r\n"), __DATE__, __TIME__, __GNUG__?"++ ":" ", __VERSION__);
	SerialPrintf(F("using avr library %s (%s)\r\n"),  __AVR_LIBC_VERSION_STRING__, __AVR_LIBC_DATE_STRING__);
	if( copyright ) {
		SerialPrintf(F("(c) 2016 www.p-r-solution.de - Norbert Richter <n.richter@p-r-solution.de>\r\n"));
	}
	//SerialTimePrintf(F("EEPROM: %d byte\r\n"), EEPROM.length());
	watchdogReset();
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
	#ifndef DEBUG_OUTPUT_WATCHDOG
	Watchdog.enable(WATCHDOG_TIME);
	#else
	int countdownMS = Watchdog.enable(WATCHDOG_TIME);
	SerialTimePrintf(F("setup - Enabled the watchdog with max countdown of %d ms\r\n"), countdownMS);
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
 * Function:    SerialPrintf
 * Return:
 * Arguments:	printf arguments
 * Description: Serial output message
 * ===================================================================*/
void SerialPrintf(const __FlashStringHelper *fmt, ... )
{
	char buf[MAX_PRINTF_BUFFER]; // resulting string limited to 128 chars

	va_list args;
	va_start (args, fmt);
	#ifdef __AVR__
	// progmem for AVR
	vsnprintf_P(buf, sizeof(buf), (const char *)fmt, args);
	#else
	// for the rest of the world
	vsnprintf(buf, sizeof(buf), (const char *)fmt, args);
	#endif
	va_end(args);
	Serial.print(buf);
	watchdogReset();
}

/* ===================================================================
 * Function:    SerialTimePrintf
 * Return:
 * Arguments:	printf arguments
 * Description: Serial output message with timestamp
 * ===================================================================*/
void SerialTimePrintf(const __FlashStringHelper *fmt, ... )
{
	char buf[MAX_PRINTF_BUFFER]; // resulting string limited to 128 chars
	uint32_t t;
	uint16_t days;
	uint8_t hours, minutes, seconds;

	va_list args;
	va_start (args, fmt);

	t = millis();
	/* div_t div(int number, int denom) */
	days = (uint16_t)(t / (24L * 60L * 60L * 1000L));
	t -= (uint32_t)days * (24L * 60L * 60L * 1000L);
	hours = (uint8_t)(t / (60L * 60L * 1000L));
	t -= (uint32_t)hours * (60L * 60L * 1000L);
	minutes = (uint8_t)(t / (60L * 1000L));
	t -= (uint32_t)minutes * (60L * 1000L);
	seconds = (uint8_t)(t / 1000L);
	t -= (uint32_t)seconds * 1000L;
	if ( days ) {
		sprintf_P(buf, PSTR("%d day%s, %02d:%02d:%02d.%03ld "), days, days==1?"":"s", hours, minutes, seconds, t);
		Serial.print(buf);
	}
	else {
		sprintf_P(buf, PSTR("%02d:%02d:%02d.%03ld "), hours, minutes, seconds, t);
		Serial.print(buf);
	}
	#ifdef __AVR__
	// progmem for AVR
	vsnprintf_P(buf, sizeof(buf), (const char *)fmt, args);
	#else
	// for the rest of the world
	vsnprintf(buf, sizeof(buf), (const char *)fmt, args);
	#endif
	va_end(args);

	Serial.print(buf);
	watchdogReset();
}

/* ===================================================================
 * Function:    sendStatus
 * Return:
 * Arguments:	str - Statusmeldung, die ausgegeben werden soll
 * Description: Statusmeldungen via RS232 senden, falls
 *              Statusmeldungen 'enabled'
 * ===================================================================*/
void sendStatus(const __FlashStringHelper *fmt, ... )
{
	if( eepromCmdSendStatus ) {
		char buf[MAX_PRINTF_BUFFER]; // resulting string limited to 128 chars

		va_list args;
		va_start (args, fmt);
		#ifdef __AVR__
		// progmem for AVR
		vsnprintf_P(buf, sizeof(buf), (const char *)fmt, args);
		#else
		// for the rest of the world
		vsnprintf(buf, sizeof(buf), (const char *)fmt, args);
		#endif
		va_end(args);
		Serial.print(buf);
		Serial.print("\r\n");
		watchdogReset();
	}
}



/* ===================================================================
 * Function:    setMotorDirection
 * Return:
 * Arguments:
 * Description: Motor in neue Laufrichtung (oder AUS) schalten
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
				sendStatus(F("01 M%i OPENING DELAYED"), motorNum+1);
			}
			// Motor läuft auf Öffnen
			else if (MotorCtrl[motorNum] >= MOTOR_OPEN) {
				// Motor aus
				MotorCtrl[motorNum] = MOTOR_OFF;
				sendStatus(F("01 M%i OFF"), motorNum+1);
			}
			// Motor ist aus
			else {
				// Motor auf öffnen ohne Umschaltdelay
				MotorCtrl[motorNum] = MOTOR_OPEN;
				sendStatus(F("01 M%i OPENING"), motorNum+1);
			}
		}
		// Neue Richtung: Schliessen
		else if ( newDirection <= MOTOR_CLOSE ) {
			// Motor läuft auf Öffnen
			if (MotorCtrl[motorNum] >= MOTOR_OPEN) {
				// Motor auf Schliessen mit Umschaltdelay
				MotorCtrl[motorNum] = MOTOR_CLOSE_DELAYED;
				sendStatus(F("01 M%i CLOSING DELAYED"), motorNum+1);
			}
			// Motor läuft auf Schliessen
			else if (MotorCtrl[motorNum] <= MOTOR_CLOSE) {
				// Motor aus
				MotorCtrl[motorNum] = MOTOR_OFF;
				sendStatus(F("01 M%i OFF"), motorNum+1);
			}
			// Motor ist aus
			else {
				// Motor auf Schliessen ohne Umschaltdelay
				MotorCtrl[motorNum] = MOTOR_CLOSE;
				sendStatus(F("01 M%i CLOSING"), motorNum+1);
			}
		}
		// Neue Richtung: AUS
		else {
			// Motor AUS
			MotorCtrl[motorNum] = MOTOR_OFF;
			sendStatus(F("01 M%i OFF"), motorNum+1);
		}
	}
}

/* ===================================================================
 * Function:    getMotorDirection
 * Return:		Laufrichtung
 *              MOTOR_OPEN, MOTOR_OPEN_DELAYED
 *              MOTOR_CLOSE, MOTOR_CLOSE_DELAYED
 *              MOTOR_OFF
 * Arguments:	motorNum - die Motorennummer [0..x]
 * Description: Motor Laufrichtung zurückgeben
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
 * Arguments:	motorNum - die Motorennummer [0..x]
 *              mType    - Motortyp: WINDOW, JALOUSIE
 * Description: Motortyp setzen
 * ===================================================================*/
void setMotorType(byte motorNum, mtype mType)
{
	if( mType == WINDOW ) {
		bitSet(eepromMTypeBitmask, motorNum);
	}
	else {
		bitClear(eepromMTypeBitmask, motorNum);
	}
}

/* ===================================================================
 * Function:    setMotorType
 * Return:
 * Arguments:	motorNum - die Motorennummer [0..x]
 *              mType    - Motortyp: WINDOW, JALOUSIE
 * Description: Motortyp setzen
 * ===================================================================*/
mtype getMotorType(byte motorNum)
{
	return bitRead(eepromMTypeBitmask, motorNum) ? WINDOW : JALOUSIE;
}
