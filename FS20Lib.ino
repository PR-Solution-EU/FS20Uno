/* ===================================================================
 * FS20Uno Helper Functions
 * ===================================================================*/

#define MAX_PRINTF_BUFFER	160
#define strnicmp(str1, str2, n) strncasecmp_P(str1, (const char *)str2, n)


void strReplaceChar(char *s, char find, char replace)
{
    while (*s != 0) {
        if (*s == find)
        *s = replace;
        s++;
    }
}

void printCRLF()
{
	Serial.println();
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
	SerialPrintfln(F("%s v%s (build %s)"), PROGRAM, VERSION, REVISION);
	SerialPrintfln(F("compiled on %s %s (GnuC%S %s)"), __DATE__, __TIME__, __GNUG__?F("++ "):F(" "), __VERSION__);
	SerialPrintfln(F("using avr library %s (%s)"),  __AVR_LIBC_VERSION_STRING__, __AVR_LIBC_DATE_STRING__);
	if( copyright ) {
		SerialPrintfln(F("(c) 2016 www.p-r-solution.de - Norbert Richter <n.richter@p-r-solution.de>"));
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
	#ifndef DEBUG_OUTPUT_WATCHDOG
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

	/* max of millis() (unsigned long) ist 49 days 17:02:47.295 */
	t = millis();

	/* ms zurückliefern, falls Zeiger auf Variable vorhanden */
	if ( milli != NULL ) {
		*milli = (uint16_t)(t % 1000UL);
	}
	// t in Sek.
	t /= 1000;
	/* Benötigt jetzt nur noch 3 Byte:
	 * 4294967295                  = 0xFFFFFFFF
	 * 4294967295 / 1000 = 4294967 = 0x418937
	 * So können wir die Tage aufmultiplizieren
	 * Ein Überlauf findet nun erst in 134 Jahren statt */
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
 * Arguments:	str - Statusmeldung, die ausgegeben werden soll
 * Description: Statusmeldungen via RS232 senden, falls
 *              Statusmeldungen 'enabled'.
 *              Format: t type status
 *                t       Status Typ Nr
 *                        0: Systemmeldung
 *                        1: Motormeldungen
 *                        2: FS20 Ausgang Meldungen
 *                        3: FS20 Eingang Meldungen
 *                        4: Wandtaster Meldungen
 *                        5: Regensensor Meldungen
 *                type    Status Typ in Textform
 *                        0: SYSTEM
 *                        1: MOTOR 01..xx
 *                        2: FS20OUT 01..xx
 *                        3: FS20IN 01..xx
 *                        4: PUSHBUTTON 01..xx
 *                        5: RAIN
 *                status  0: FS20Uno version|START|RUNNING xx h
 *                        1: TIMEOUT xx|OPENING [DELAYED]|CLOSING [DELAYED]|OFF
 *                        2: ON|OFF
 *                        3: ON|OFF
 *                        4: ON|OFF
 *                        5: AUTO|MANUAL ENABLED|DISABLED DRY|WET
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
 * Function:     sendMotorStatus
 * Return:
 * Arguments:
 * Description:  Motor OFF Status aus IRQ senden
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
 * Description:  Motor OFF Status aus IRQ senden
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
 * Arguments:   motorNum    - Motornummer [0..x]
 *              destPercent - Zielposition [0..100]
 *                            wobei  0 = CLOSE
 *                            und  100 = OPEN
 * Description: Motor auf bestimmte Position schalten
 * ===================================================================*/
void setMotorPosition(byte motorNum, byte destPercent)
{
	#ifdef DEBUG_OUTPUT_MOTOR
	SerialTimePrintfln(F("setMotorPosition- Motor %d current pos=%d, destPercent=%d%%"), motorNum+1, MotorPosition[motorNum], destPercent);
	#endif
	if ( destPercent>0 && destPercent<100 ) {
		destMotorPosition[motorNum] = (MOTOR_TIMER)((long)(eeprom.MaxRuntime[motorNum] / TIMER_MS) * (long)destPercent / 100L);

		#ifdef DEBUG_OUTPUT_MOTOR
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
		bitSet(eeprom.MTypeBitmask, motorNum);
	}
	else {
		bitClear(eeprom.MTypeBitmask, motorNum);
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
	if ( bitRead(eeprom.MTypeBitmask, motorNum) ) {
		return WINDOW;
	}
	else {
		return JALOUSIE;
	}
}
