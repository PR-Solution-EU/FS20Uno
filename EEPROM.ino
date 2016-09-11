/* ===================================================================
 * EEPROM Functions
 * ===================================================================*/


/* *******************************************************************
 * LOW-LEVEL Functions
 * ********************************************************************/

/* ===================================================================
 * Function:	CalcCRC
 * Return:
 * Arguments:	addr - Startadresse des Bereichs
 * 				size - Größe des Bereichs in Byte
 * Description: Berechnet CRC32 Summe über alle RAM Daten
 * ===================================================================*/
// CRC table
const PROGMEM unsigned long ccrc_table[16] = {
	0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
	0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
	0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
	0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};
unsigned long CalcCRC(crcType type, byte *addr, size_t size)
{
	unsigned long crc  = ~0L;
	unsigned int k;
	byte data;

	DEBUG_RUNTIME_START(msCalcCRC);
	for (size_t index=0; index < size; ++index) {
		data = (type==RAMCRC?*(addr+index):EEPROM.read((unsigned int)addr+index));
		k = ((crc ^ data) & 0x0f);
		crc = (unsigned long)pgm_read_dword_near( ccrc_table + k ) ^ (crc >> 4);
		k = ((crc ^ (data >> 4)) & 0x0f);
		crc = (unsigned long)pgm_read_dword_near( ccrc_table + k ) ^ (crc >> 4);
		crc = ~crc;
	}
	DEBUG_RUNTIME_END("CalcCRC()",msCalcCRC);
	#ifdef DEBUG_OUTPUT_EEPROM
	SerialTimePrintfln(F("EEPROM - CalcCRC(%p, %ld) returns 0x%08lx"), addr, (unsigned long)size, crc);
	#endif
	return crc;
}


/* *******************************************************************
 * HIGH-LEVEL Functions
 * ********************************************************************/


/* ===================================================================
 * Function:	eepromInitVars
 * Return:
 * Arguments:
 * Description: Initalisiere Standard Werte einiger Programmvariablen
 *              aus EEPROM-Daten
 * ===================================================================*/
void eepromInitVars()
{
	int i;
	unsigned long dataCRC;
	unsigned long eepromCRC;

	// Write data version into EEPROM before checking CRC32
	byte dataversion = DATAVERSION;
	EEPROM.put(EEPROM_ADDR_DATAVERSION, dataversion);

	// Vergleiche kalkulierte CRC32 mit gespeicherter CRC32
	dataCRC = CalcCRC(EEPROMCRC, (byte *)(EEPROM_ADDR_CRC32+4), EEPROM.length()-4);
	EEPROM.get(EEPROM_ADDR_CRC32, eepromCRC);

	#ifdef DEBUG_OUTPUT_EEPROM
	//Print the result of calling eepromCRC()
	SerialTimePrintfln(F("EEPROM - values CRC32: 0x%08lx"), dataCRC);
	SerialTimePrintfln(F("EEPROM - stored CRC32: 0x%08lx"), eepromCRC);
	#endif

	if ( dataCRC != eepromCRC ) {
		#ifdef DEBUG_OUTPUT_EEPROM
		SerialTimePrintfln(F("EEPROM - CRC32 not matching, set defaults..."));
		#endif
		eeprom.BlinkInterval 	= LED_BLINK_INTERVAL;
		eeprom.BlinkLen      	= LED_BLINK_LEN;
		eeprom.MTypeBitmask  	= MTYPE_BITMASK;
		byte window=1;
		byte jalousie=1;
		for(i=0; i<MAX_MOTORS; i++) {
			eeprom.MaxRuntime[i] = bitRead(MTYPE_BITMASK,i)!=0?MOTOR_WINDOW_MAXRUNTIME:MOTOR_JALOUSIE_MAXRUNTIME;
			if ( getMotorType(i)==WINDOW ) {
				sprintf_P((char *)eeprom.MotorName[i], PSTR("Window %d"), window);
				window++;
			}
			else {
				sprintf_P((char *)eeprom.MotorName[i], PSTR("Jalousie %d"), jalousie);
				jalousie++;
			}
			// Annahme: Fenster sind geschlossen, Jalousien offen
			eeprom.MotorPosition[i] = getMotorType(i)==WINDOW ? 0 : (eeprom.MaxRuntime[i]/TIMER_MS);
		}
		bitSet(eeprom.Rain,   RAIN_BIT_AUTO);
		bitClear(eeprom.Rain, RAIN_BIT_ENABLE);
		eeprom.RainResumeTime 	= DEFAULT_RAINRESUMETIME;
		eeprom.SendStatus  		= DEFAULT_SendStatus;
		eeprom.Echo        		= DEFAULT_Echo;
		eeprom.Term        		= DEFAULT_Term;
		eeprom.OperatingHours 	= 0;

		eepromWriteVars();
	}
	#ifdef DEBUG_OUTPUT_EEPROM
	else {
		SerialTimePrintfln(F("EEPROM - CRC232 is valid"));
	}
	#endif

	#ifdef DEBUG_OUTPUT_EEPROM
	SerialTimePrintfln(F("EEPROM - read defaults..."));
	#endif
	DEBUG_RUNTIME_START(mseepromReadDefaults);
	EEPROM.get(EEPROM_ADDR_EEPROMDATA, eeprom);
	DEBUG_RUNTIME_END("eepromInitVars() read defaults",mseepromReadDefaults);
	
	#ifdef DEBUG_OUTPUT_EEPROM
	SerialTimePrintfln(F("EEPROM - values:"));
	SerialTimePrintfln(F("EEPROM -   eeprom.BlinkInterval: %d"),     eeprom.BlinkInterval);
	SerialTimePrintfln(F("EEPROM -   eeprom.BlinkLen:      %d"),     eeprom.BlinkLen);
	SerialTimePrintfln(F("EEPROM -   eeprom.MTypeBitmask:  0x%02x"), eeprom.MTypeBitmask);
	SerialTimePrintf  (F("EEPROM -   eeprom.MaxRuntime:    "));
	for(i=0; i<MAX_MOTORS; i++) {
		SerialPrintf(F("%s%ld"), i?",":"", eeprom.MaxRuntime[i]);
	}
	printCRLF();
	SerialTimePrintf  (F("EEPROM -   eeprom.MotorName:     "));
	for(i=0; i<MAX_MOTORS; i++) {
		SerialPrintf(F("%s%s"), i?",":"", (char *)eeprom.MotorName[i]);
	}
	printCRLF();
	SerialTimePrintf  (F("EEPROM -   eeprom.MotorPosition: "));
	for(i=0; i<MAX_MOTORS; i++) {
		SerialPrintf(F("%s%d"), i?",":"", eeprom.MotorPosition[i]);
	}
	printCRLF();
	SerialTimePrintfln(F("EEPROM -   eeprom.Rain:          0x%02x"), eeprom.Rain);
	SerialTimePrintfln(F("EEPROM -   eeprom.RainResumeTime:%d"), eeprom.RainResumeTime);
	SerialTimePrintfln(F("EEPROM -   eeprom.SendStatus: %s"), eeprom.SendStatus?"yes":"no");
	SerialTimePrintfln(F("EEPROM -   eeprom.Echo:       %s"), eeprom.Echo?"yes":"no");
	SerialTimePrintfln(F("EEPROM -   eeprom.Term:       %s"), eeprom.Term=='\r'?"CR":"LF");
	SerialTimePrintfln(F("EEPROM -   eeprom.OperatingHours:%ld"), eeprom.OperatingHours);
	#endif
}


/* ===================================================================
 * Function:    eepromWriteVars()
 * Return:
 * Arguments:
 * Description: Schreibt ggf. geänderte EEPROM Daten ins EEPROM
 * ===================================================================*/
void eepromWriteVars(void)
{
	static unsigned long prevEEPROMDataCRC32 = 0;
	unsigned long curEEPROMDataCRC32 = 0;
	unsigned long eepromCRC;
	
	DEBUG_RUNTIME_START(mseepromWriteVars);
	curEEPROMDataCRC32 = CalcCRC(RAMCRC, (byte *)&eeprom, sizeof(eeprom));
	DEBUG_RUNTIME_END("eepromWriteVars()", mseepromWriteVars);

	EEPROM.get(EEPROM_ADDR_CRC32, eepromCRC);

	#ifdef DEBUG_OUTPUT_EEPROM
	SerialTimePrintfln(F("EEPROM - prevEEPROMDataCRC32: 0x%08lx"), prevEEPROMDataCRC32);
	SerialTimePrintfln(F("EEPROM - curEEPROMDataCRC32:  0x%08lx"), curEEPROMDataCRC32);
	SerialTimePrintfln(F("EEPROM - eepromCRC:           0x%08lx"), eepromCRC);
	#endif

	if( curEEPROMDataCRC32 != prevEEPROMDataCRC32 ) {
		#ifdef DEBUG_OUTPUT_EEPROM
		SerialTimePrintfln(F("EEPROM - Data changed, write EEPROM data"));
		#endif
		// Write new EEPROM data
		EEPROM.put(EEPROM_ADDR_EEPROMDATA, eeprom);

		// Wire new CRC
		eepromCRC = CalcCRC(EEPROMCRC, (byte *)(EEPROM_ADDR_CRC32+4), EEPROM.length()-4);
		EEPROM.put(EEPROM_ADDR_CRC32, eepromCRC);
		#ifdef DEBUG_OUTPUT_EEPROM
		SerialTimePrintfln(F("EEPROM - eepromCRC:           0x%08lx"), eepromCRC);
		#endif

		prevEEPROMDataCRC32 = curEEPROMDataCRC32;
	}
}
