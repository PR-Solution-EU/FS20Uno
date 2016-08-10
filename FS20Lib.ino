/* ===================================================================
 * FS20Uno Helper Functions
 * ===================================================================*/

#define MAX_PRINTF_BUFFER	128

/* *******************************************************************
 * LOW-LEVEL Functions
 * ********************************************************************/

/* ===================================================================
 * Function:    stricmp
 * Return:      Returns a negative value if str1<str2;
 *              0 if str1 and str2 are identical;
 *              and positive value if str1>str2.
 * Arguments:	str1, str2 - strings to comparre
 * Description: compares str1 and str2 lexicographically without
 *              regards to case
 * ===================================================================*/
int stricmp(const char * str1, const char *str2)
{
    char c1, c2;
    int v;

    do {
        c1 = *str1++;
        c2 = *str2++;
        v = (unsigned int) tolower(c1) - (unsigned int) tolower(c2);
    } while ((v == 0) && (c1 != '\0') && (c2 != '\0') );

    return v;
}

/* ===================================================================
 * Function:    strnicmp
 * Return:      Returns a negative value if str1<str2;
 *              0 if str1 and str2 are identical;
 *              and positive value if str1>str2.
 * Arguments:	str1, str2 - strings to comparre
 * Description: compares at most n characters of str2 to str1,
 *              lexicographically, by ignoring case.
 * ===================================================================*/
int strnicmp(const char *str1, const char *str2, size_t Count)
{
    char c1, c2;
    int v;

    if (Count == 0)
        return 0;

    do {
        c1 = *str1++;
        c2 = *str2++;
        /* the casts are necessary when str1 is shorter & char is signed */
        v = (unsigned int) tolower(c1) - (unsigned int) tolower(c2);
    } while ((v == 0) && (c1 != '\0') && (--Count > 0));

    return v;
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
	SerialPrintf(F("%s v%s (build %s)\r\n"), PROGRAM, VERSION, REVISION);
	SerialPrintf(F("compiled on %s %s (GnuC%s %s)\r\n"), __DATE__, __TIME__, __GNUG__?"++ ":" ", __VERSION__);
	if( copyright ) {
		SerialPrintf(F("(c) 2016 www.p-r-solution.de - Norbert Richter <n.richter@p-r-solution.de>\r\n"));
	}
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
	}
}
