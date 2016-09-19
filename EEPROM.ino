/* ===================================================================
 * EEPROM Functions
 * ===================================================================*/
/* EEPROM structure
 *    8 Byte: EEPROM_MAGIC_WORD
 *    4 Byte: Write Count
 *  310 Byte: MYEEPROM struct
 *
 * This structure block can be moved within EEPROM by wear-leveling.
 * The EEPROM_MAGIC_WORD is used to find the struct within the
 * ATmega328/P EEPROM.
 * 
 * General: "Writing" to EEPROM means "Update" always, means only bytes
 * which are different are realy written.
 *
 * The MYEEPROM struct contains an own CRC32 checksum at the beginning
 * followed by a byte containing data version number. Whenever vars
 * within MYEEPROM struct is changed, the function eepromWriteVars()
 * is called to check if the CRC32 is valid. If it becomes invalid,
 * new CRC32 is stored within MYEEPROM struct and the complete struct
 * will be written to ATmega328/P EEPROM.
 *
 * Each write is counted in "Write Count".
 * Function eepromWearLeveling() is called if "Write Count" exceeds
 * EEPROM_WEARLEVELING_WRITECNT value. This function will determine
 * a new EEPROM position for the whole structure using random(), moves
 * the structure to new position and reset the "Write Count".
 * 
 */

#define EEPROM_MAGIC_WORD 885418781762039 // = 0x000325489FF6E1F7
int dataAddr = -1;


/* ===================================================================
 * Function:	eepromWearLeveling
 * Return:
 * Arguments:
 * Description: Do EEPROM wear-leveling my moving MAGIC WORD to another
 *              position
 * ===================================================================*/
#ifdef DEBUG_EEPROM
const char fstrEepromWearLeveling[]	PROGMEM = "EEPROM - eepromWearLeveling() ";
#endif
void eepromWearLeveling(void)
{
	int oldAddr;
	int newAddr;
	struct MYEEPROMHEADER header;

	oldAddr = eepromStartAddr();
	newAddr = oldAddr;
	while ( newAddr == oldAddr ) {
		randomSeed(millis());
		newAddr = random(sizeof(header),EEPROM.length()-sizeof(eeprom));
		watchdogReset();
	}

	#ifdef DEBUG_EEPROM
	SerialTimePrintfln(F("%SoldAddr=%d, newAddr=%d"), fstrEepromWearLeveling, oldAddr, newAddr);
	#endif

	// invalidate old header
	#ifdef DEBUG_EEPROM
	SerialTimePrintfln(F("%S invalidate header @%d"), fstrEepromWearLeveling, oldAddr-sizeof(header));
	#endif
	for(int i=oldAddr-sizeof(header); i<oldAddr; i++) {
		EEPROM.update(i, 0xFF);
	}

	if ( newAddr < oldAddr ) {
		#ifdef DEBUG_EEPROM
		SerialTimePrintfln(F("%S copy from %d-%d to %d-%d"), fstrEepromWearLeveling, oldAddr+sizeof(eeprom), oldAddr, newAddr+sizeof(eeprom), newAddr);
		#endif
		// copy from end of data to new address
		for(int i=sizeof(eeprom); i>=0; i--) {
			EEPROM.update(newAddr+i, EEPROM.read(oldAddr+i));
			watchdogReset();
		}
	}
	else {
		#ifdef DEBUG_EEPROM
		SerialTimePrintfln(F("%S copy from %d-%d to %d-%d"), fstrEepromWearLeveling, oldAddr, oldAddr+sizeof(eeprom), newAddr, newAddr+sizeof(eeprom));
		#endif
		// copy from begin of data to new address
		for(size_t i=0; i<sizeof(eeprom); i++) {
			EEPROM.update(newAddr+i, EEPROM.read(oldAddr+i));
			watchdogReset();
		}
	}

	// validate new header
	#ifdef DEBUG_EEPROM
	SerialTimePrintfln(F("%S validate new header @%d"), fstrEepromWearLeveling, newAddr-sizeof(header));
	#endif
	header.Magic      = EEPROM_MAGIC_WORD;
	header.WriteCount = 0;
	EEPROM.put(newAddr-sizeof(header), header);
	dataAddr = newAddr;

}

/* ===================================================================
 * Function:	eepromStartAddr
 * Return:		Current EEPROM data start address
 * Arguments:
 * Description: returns the current EEPROM start address
 *              Address will change dynamically due to wear-leveling
 * ===================================================================*/
#ifdef DEBUG_EEPROM
const char fstrEepromStartAddr[]	PROGMEM = "EEPROM - eepromStartAddr() ";
#endif
int eepromStartAddr(void)
{
	struct MYEEPROMHEADER header;
	bool found;

	if ( dataAddr == -1 ) {
		#ifdef DEBUG_EEPROM
		SerialTimePrintfln(F("%SEEPROM.length() %d"), fstrEepromStartAddr, EEPROM.length());
		SerialTimePrintfln(F("%Ssizeof(eeprom) %d"), fstrEepromStartAddr, sizeof(eeprom));
		SerialTimePrintfln(F("%Ssizeof(header) %d"), fstrEepromStartAddr, sizeof(header));
		SerialTimePrintfln(F("%Ssearch start %d"), fstrEepromStartAddr, EEPROM.length()-sizeof(eeprom)-sizeof(header));
		#endif

		for (found=false, dataAddr=EEPROM.length()-sizeof(eeprom)-sizeof(header); dataAddr>=0; dataAddr-- ) {
			EEPROM.get(dataAddr, header);
			if ( header.Magic==EEPROM_MAGIC_WORD ) {
				found=true;
				break;
			}
		}
		#ifdef DEBUG_EEPROM
		SerialTimePrintfln(F("%S%S found: dataAddr=%d, Magic=0x%08lx%08lx, WriteCount=%d"), fstrEepromStartAddr, found?F(""):F("not"), dataAddr, (uint32_t)(header.Magic>>32), (uint32_t)(header.Magic & 0xFFFFFFFF), header.WriteCount);
		#endif

		if ( found==0 ) {
			#ifdef DEBUG_EEPROM
			SerialTimePrintfln(F("%Smagic not found"), fstrEepromStartAddr);
			#endif
			randomSeed(millis());
			dataAddr = random(0,EEPROM.length()-sizeof(eeprom)-sizeof(header));
			header.Magic      = EEPROM_MAGIC_WORD;
			header.WriteCount = 0;
		}
		if ( header.WriteCount < 0 ) {
			header.WriteCount=0;
		}
		#ifdef DEBUG_EEPROM
		SerialTimePrintfln(F("%Sends with dataAddr=%d, Magic=0x%08lx%08lx, WriteCount=%d"), fstrEepromStartAddr, dataAddr, (uint32_t)(header.Magic>>32), (uint32_t)(header.Magic & 0xFFFFFFFF), header.WriteCount);
		#endif

		EEPROM.put(dataAddr, header);
		dataAddr += sizeof(header);
	}
	#ifdef DEBUG_EEPROM
	SerialTimePrintfln(F("%Sreturns %d"), fstrEepromStartAddr, dataAddr);
	#endif

	return dataAddr;
}

/* ===================================================================
 * Function:	eepromInitVars
 * Return:
 * Arguments:
 * Description: Initalize program settings from EEPROM
 *              Set default values if EEPROM data are invalid
 * ===================================================================*/
#ifdef DEBUG_EEPROM
const char fstrEepromInitVars[]	PROGMEM = "EEPROM - eepromInitVars() ";
#endif
void eepromInitVars()
{
	uint32_t crc32RAM;

	// First write DataVersion struct element direct into EEPROM
	byte dataversion = DATAVERSION;
	#ifdef DEBUG_EEPROM
	SerialTimePrintfln(F("%SEEPROM.put(%d,%d)"),fstrEepromInitVars,eepromStartAddr()+offsetof(MYEEPROM,DataVersion), dataversion);
	#endif
	EEPROM.put(eepromStartAddr()+offsetof(MYEEPROM,DataVersion), dataversion);

	// if DataVersion is not equal EEPROM stored value, the next
	// step will be invalid and

	// Read out EEPROM data
	#ifdef DEBUG_EEPROM
	SerialTimePrintfln(F("%Sread"),fstrEepromInitVars);
	#endif
	DEBUG_RUNTIME_START(mseepromReadDefaults);
	EEPROM.get(eepromStartAddr(), eeprom);
	DEBUG_RUNTIME_END("eepromInitVars() read",mseepromReadDefaults);

	// Calc CRC checksum from data in RAM
	crc32RAM = CalcCRC( (byte *)&eeprom+offsetof(MYEEPROM,DataVersion)
					    ,sizeof(eeprom)-sizeof(eeprom.CRC32) );

	#ifdef DEBUG_EEPROM
	SerialTimePrintfln(F("%Scalced CRC32: 0x%08lx"),fstrEepromInitVars, crc32RAM);
	SerialTimePrintfln(F("%Seeprom.CRC32: 0x%08lx"),fstrEepromInitVars, eeprom.CRC32);
	#endif

	if ( eeprom.CRC32 != crc32RAM) {
		#ifdef DEBUG_EEPROM
		SerialTimePrintfln(F("%Sset defaults!"),fstrEepromInitVars);
		#endif
		sendStatus(true,SYSTEM, F("SET DEFAULT VALUES"));
		eeprom.LEDBitCount  	= DEFAULT_LED_BITCOUNT;
		eeprom.LEDBitLenght 	= DEFAULT_LED_BITLENGTH;
		eeprom.LEDPatternNormal = DEFAULT_LED_NORMAL;
		eeprom.LEDPatternRain	= DEFAULT_LED_RAIN;
		eeprom.MTypeBitmask  	= DEFAULT_MOTORTYPE;
		byte window=1;
		byte jalousie=1;
		for(int i=0; i<MAX_MOTORS; i++) {
			eeprom.MaxRuntime[i] = bitRead(DEFAULT_MOTORTYPE,i)!=0?MOTOR_WINDOW_MAXRUNTIME:MOTOR_JALOUSIE_MAXRUNTIME;
			eeprom.OvertravelTime[i] = bitRead(DEFAULT_MOTORTYPE,i)!=0?MOTOR_WINDOW_OVERTRAVELTIME:MOTOR_JALOUSIE_OVERTRAVELTIME;
			if ( getMotorType(i)==WINDOW ) {
				sprintf_P((char *)eeprom.MotorName[i], PSTR("Window %d"), window);
				window++;
			}
			else {
				sprintf_P((char *)eeprom.MotorName[i], PSTR("Jalousie %d"), jalousie);
				jalousie++;
			}
			// assumtion: windows are closed, jalousie are opened
			eeprom.MotorPosition[i] = getMotorType(i)==WINDOW ? 0 : (eeprom.MaxRuntime[i]/TIMER_MS);
		}
		bitSet(eeprom.Rain,   RAIN_BIT_AUTO);
		bitClear(eeprom.Rain, RAIN_BIT_ENABLE);
		eeprom.RainResumeTime 	= DEFAULT_RAINRESUMETIME;
		eeprom.SendStatus  		= DEFAULT_SENDSTATUS;
		eeprom.Echo        		= DEFAULT_ECHO;
		eeprom.Term        		= DEFAULT_TERM;
		eeprom.OperatingHours 	= 0;
		eeprom.LoginTimeout		= DEFAULT_LOGIN_TIMEOUT;
		for(size_t i=0; i<(sizeof(eeprom.EncryptKey)/sizeof(eeprom.EncryptKey[0])); i++) {
			eeprom.EncryptKey[i] = random(0, INT32_MAX);
		}
		memset(eeprom.Password, 0, sizeof(eeprom.Password));
		strncpy_P(eeprom.Password, DEFAULT_PASSWORD, sizeof(eeprom.Password));
		cryptPassword(eeprom.Password, eeprom.EncryptKey, ENCRYPT);

		eepromWriteVars();
	}
	#ifdef DEBUG_EEPROM
	else {
		SerialTimePrintfln(F("%SCRC32 valid"),fstrEepromInitVars);
	}
	#endif

	#ifdef DEBUG_EEPROMx
	SerialTimePrintfln(F("%Svalues:"),fstrEepromInitVars);
	SerialTimePrintfln(F("\t\teeprom.LEDPatternNormal:0x%08lx"),	eeprom.LEDPatternNormal);
	SerialTimePrintfln(F("\t\teeprom.LEDPatternRain:\t0x%08lx"),	eeprom.LEDPatternRain);
	SerialTimePrintfln(F("\t\teeprom.LEDBitCount:\t%d"),         	eeprom.LEDBitCount);
	SerialTimePrintfln(F("\t\teeprom.LEDBitLenght:\t%d"),			eeprom.LEDBitLenght);
	SerialTimePrintfln(F("\t\teeprom.MTypeBitmask:\t0x%02x"),		eeprom.MTypeBitmask);
	SerialTimePrintf  (F("\t\teeprom.MaxRuntime:\t"));
	for(int i=0; i<MAX_MOTORS; i++) {
		SerialPrintf(F("%s%ld"), i?",":"", eeprom.MaxRuntime[i]);
	}
	printCRLF();
	SerialTimePrintf  (F("\t\teeprom.MotorName:\t"));
	for(int i=0; i<MAX_MOTORS; i++) {
		SerialPrintf(F("%s%s"), i?",":"", (char *)eeprom.MotorName[i]);
	}
	printCRLF();
	SerialTimePrintf  (F("\t\teeprom.MotorPosition:\t"));
	for(int i=0; i<MAX_MOTORS; i++) {
		SerialPrintf(F("%s%d"), i?",":"", eeprom.MotorPosition[i]);
	}
	printCRLF();
	SerialTimePrintfln(F("\t\teeprom.Rain:\t\t0x%02x"), eeprom.Rain);
	SerialTimePrintfln(F("\t\teeprom.RainResumeTime:\t%d"), eeprom.RainResumeTime);
	SerialTimePrintfln(F("\t\teeprom.SendStatus:\t%s"), eeprom.SendStatus?"yes":"no");
	SerialTimePrintfln(F("\t\teeprom.Echo:\t\t%s"), eeprom.Echo?"yes":"no");
	SerialTimePrintfln(F("\t\teeprom.Term:\t\t%s"), eeprom.Term=='\r'?"CR":"LF");
	SerialTimePrintfln(F("\t\teeprom.OperatingHours:\t%ld"), eeprom.OperatingHours);
	SerialTimePrintfln(F("\t\teeprom.LoginTimeout:\t%d"), eeprom.LoginTimeout);
	SerialTimePrintf(  F("\t\teeprom.EncryptKey:\t") );
	for(size_t i=0; i<(sizeof(eeprom.EncryptKey)/sizeof(eeprom.EncryptKey[0])); i++) {
		SerialPrintf(F("0x%08lx "), eeprom.EncryptKey[i]);
	}
	printCRLF();
	SerialTimePrintf(  F("\t\teeprom.Password:\t") );
	for(size_t i=0; i<(sizeof(eeprom.Password)/sizeof(eeprom.Password[0])); i++) {
		SerialPrintf(F("%02x "), (byte)eeprom.Password[i]);
	}
	printCRLF();
	printCRLF();

	#endif
}


/* ===================================================================
 * Function:    eepromWriteVars()
 * Return:
 * Arguments:
 * Description: Writes program settings into EEPROM (if changed)
 * ===================================================================*/
#ifdef DEBUG_EEPROM
const char fstrEepromWriteVars[]	PROGMEM = "EEPROM - eepromWriteVars() ";
#endif
void eepromWriteVars(void)
{
	uint32_t crc32RAM;
	uint32_t crc32EEPROM;

	// Calc CRC checksum from data in RAM
	crc32RAM = CalcCRC( (byte *)&eeprom+offsetof(MYEEPROM,DataVersion)
					    ,sizeof(eeprom)-sizeof(eeprom.CRC32) );
	// Readout CRC checksum from EEPROM direct
	EEPROM.get(eepromStartAddr(), crc32EEPROM);

	#ifdef DEBUG_EEPROM
	SerialTimePrintfln(F("%Scrc32RAM:\t0x%08lx (current)"),fstrEepromWriteVars, crc32RAM);
	SerialTimePrintfln(F("%Seeprom.CRC32:0x%08lx (struct)"),fstrEepromWriteVars, eeprom.CRC32);
	SerialTimePrintfln(F("%Scrc32EEPROM:\t0x%08lx (EEPROM)"),fstrEepromWriteVars, crc32EEPROM);
	#endif

	// Even if checksum stored in struct or checksum stored in EEPROM
	// does not match the current checksum from data in RAM
	if ( crc32RAM != eeprom.CRC32 || crc32RAM != crc32EEPROM) {
		struct MYEEPROMHEADER header;

		// Write new EEPROM data
		#ifdef DEBUG_EEPROM
		SerialTimePrintfln(F("%SData changed, write data"),fstrEepromWriteVars);
		#endif
		// Store new checksum in struct
		eeprom.CRC32 = crc32RAM;
		// Write complete struct to EEPROM
		EEPROM.put(eepromStartAddr(), eeprom);

		// Read out eeprom write counter
		EEPROM.get(eepromStartAddr()-sizeof(header.WriteCount), header.WriteCount);
		if ( header.WriteCount < 0 ) {
			header.WriteCount=0;
		}
		header.WriteCount++;
		#ifdef DEBUG_EEPROM
		SerialTimePrintfln(F("%SNew write counter:\t%d"),fstrEepromWriteVars, header.WriteCount);
		#endif
		if ( header.WriteCount <= EEPROM_WEARLEVELING_WRITECNT ) {
			EEPROM.put(eepromStartAddr()-sizeof(header.WriteCount), header.WriteCount);
		}
		else {
			eepromWearLeveling();
		}
	}
}
