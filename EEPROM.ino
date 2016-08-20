/* ===================================================================
 * EEPROM Functions
 * ===================================================================*/


/* *******************************************************************
 * LOW-LEVEL Functions
 * ********************************************************************/

/* ===================================================================
 * Function:	eepromCalcCRC
 * Return:
 * Arguments:
 * Description: Berechnet CRC32 Summe über alle Daten
 *              (exklusive der gespeicherten CRC32 Summe selbst)
 * ===================================================================*/
const PROGMEM unsigned long ccrc_table[16] = {
	0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
	0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
	0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
	0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};
unsigned long eepromCalcCRC(void)
{
	unsigned long crc  = ~0L;

	for (unsigned int index = 0 ; index < EEPROM.length()  ; ++index) {
		if ( index<EEPROM_ADDR_CRC32 || index>=(EEPROM_ADDR_CRC32+4)) {
			unsigned int k;
			k = ((crc ^ EEPROM[index]) & 0x0f);
			crc = (unsigned long)pgm_read_dword_near( ccrc_table + k ) ^ (crc >> 4);
			k = ((crc ^ (EEPROM[index] >> 4)) & 0x0f);
			crc = (unsigned long)pgm_read_dword_near( ccrc_table + k ) ^ (crc >> 4);
			crc = ~crc;
		}
	}
	#ifdef DEBUG_OUTPUT_EEPROM
	SerialTimePrintf(F("EEPROM - eepromCalcCRC() returns 0x%08lx\r\n"), crc);
	#endif
	return crc;
}


/* ===================================================================
 * Function:	eepromReadArray
 * Return:		
 * Arguments:   EEPROM Adresse
 *              Adresse auf das Array
 *              Größe des Array in Byte
 * Description: Liest n Byte aus dem EEPROM in ein Array
 * ===================================================================*/
void eepromReadArray(int address, byte *array, size_t size)
{
	#ifdef DEBUG_OUTPUT_EEPROM
	SerialTimePrintf(F("EEPROM - eepromReadArray(0x%04x, %p, %d)\r\n"), address, array, size);
	#endif
	for(size_t i=0; i<size; i++) {
		*array = EEPROM[address];
		#ifdef DEBUG_OUTPUT_EEPROM_DETAILS
		SerialTimePrintf(F("           EEPROM[0x%04x] returns 0x%02x [%d]\r\n"), address, *array, *array);
		#endif
		array++;
		address++;
	}
}

/* ===================================================================
 * Function:	eepromWriteArray
 * Return:
 * Arguments:   EEPROM Adresse
 *              Adresse auf das Array
 *              Größe des Array in Byte
 * Description: Schreibt n Byte aus einem Array in das EEPROM
 * ===================================================================*/
void eepromWriteArray(int address, byte *array, size_t size)
{
	#ifdef DEBUG_OUTPUT_EEPROM
	SerialTimePrintf(F("EEPROM - eepromWriteArray(0x%04x, %p, %d)\r\n"), address, array, size);
	#endif
	for(size_t i=0; i<size; i++) {
		#ifdef DEBUG_OUTPUT_EEPROM_DETAILS
		SerialTimePrintf(F("           EEPROM.update(0x%04x, 0x%02x [%d])\r\n"), address, *array, *array);
		#endif
		EEPROM.update(address++, *array++);
	}
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
	eepromWriteArray(EEPROM_ADDR_DATAVERSION, (byte *)&dataversion, sizeof(dataversion));

	dataCRC = eepromCalcCRC();
	eepromReadArray(EEPROM_ADDR_CRC32, (byte *)&eepromCRC, sizeof(eepromCRC));

	#ifdef DEBUG_OUTPUT_EEPROM
	//Print length of data to run CRC on.
	SerialTimePrintf(F("EEPROM - length: %d\r\n"), EEPROM.length());
	//Print the result of calling eepromCRC()
	SerialTimePrintf(F("EEPROM - values CRC32: 0x%08lx\r\n"), dataCRC);
	SerialTimePrintf(F("EEPROM - stored CRC32: 0x%08lx\r\n"), eepromCRC);
	#endif

	if ( dataCRC != eepromCRC ) {
		#ifdef DEBUG_OUTPUT_EEPROM
		SerialTimePrintf(F("EEPROM - CRC32 not matching, set defaults...\r\n"));
		#endif

		eeprom.BlinkInterval = LED_BLINK_INTERVAL;
		eeprom.BlinkLen = LED_BLINK_LEN;
		eeprom.MTypeBitmask = MTYPE_BITMASK;
		for(i=0; i<MAX_MOTORS; i++) {
			eeprom.MaxRuntime[i] = bitRead(MTYPE_BITMASK,i)!=0?MOTOR_WINDOW_MAXRUNTIME:MOTOR_JALOUSIE_MAXRUNTIME;
			sprintf_P((char *)eeprom.MotorName[i], PSTR("MOTOR %02d"), i);
		}
		bitSet(eeprom.Rain, RAIN_BIT_AUTO);
		bitClear(eeprom.Rain, RAIN_BIT_ENABLE);
		eeprom.CmdSendStatus = DEFAULT_CMDSENDSTATUS;
		eeprom.CmdEcho = DEFAULT_CMDECHO;
		eeprom.CmdTerm = DEFAULT_CMDTERM;

		eepromWriteVars(EEPROM_ALL);
	}
	#ifdef DEBUG_OUTPUT_EEPROM
	else {
		SerialTimePrintf(F("EEPROM - CRC232 is valid\r\n"));
	}
	#endif
	#ifdef DEBUG_OUTPUT_EEPROM
	SerialTimePrintf(F("EEPROM - read defaults...\r\n"));
	#endif

	eepromReadVars();

	#ifdef DEBUG_OUTPUT_EEPROM
	SerialTimePrintf(F("EEPROM - values:\r\n"));
	SerialTimePrintf(F("EEPROM -   eeprom.BlinkInterval: %d\r\n"),     eeprom.BlinkInterval);
	SerialTimePrintf(F("EEPROM -   eeprom.BlinkLen:      %d\r\n"),     eeprom.BlinkLen);
	SerialTimePrintf(F("EEPROM -   eeprom.MTypeBitmask:  0x%02x\r\n"), eeprom.MTypeBitmask);
	SerialTimePrintf(F("EEPROM -   eeprom.MaxRuntime:    "));
	for(i=0; i<MAX_MOTORS; i++) {
		SerialPrintf(F("%s%ld"), i?",":"", eeprom.MaxRuntime[i]);
	}
	SerialTimePrintf(F("\r\n"));
	SerialTimePrintf(F("EEPROM -   eeprom.MotorName:     "));
	for(i=0; i<MAX_MOTORS; i++) {
		SerialPrintf(F("%s%s"), i?",":"", (char *)eeprom.MotorName[i]);
	}
	SerialPrintf(F("\r\n"));
	SerialTimePrintf(F("EEPROM -   eeprom.Rain:          0x%02x\r\n"), eeprom.Rain);
	SerialTimePrintf(F("EEPROM -   eeprom.CmdSendStatus: %s\r\n"), eeprom.CmdSendStatus?"yes":"no");
	SerialTimePrintf(F("EEPROM -   eeprom.CmdEcho:       %s\r\n"), eeprom.CmdEcho?"yes":"no");
	SerialTimePrintf(F("EEPROM -   eeprom.CmdTerm:       %s\r\n"), eeprom.CmdTerm=='\r'?"CR":"LF");
	#endif
}


/* ===================================================================
 * Function:	eepromWriteVars
 * Return:
 * Arguments:
 * Description: Schreibe alle EEPROM Programmvariablen in EEPROM
 * ===================================================================*/
void eepromWriteVars(WORD varType)
{
	#ifdef DEBUG_OUTPUT_EEPROM
	SerialTimePrintf(F("EEPROM - eepromWriteVars()\r\n"));
	#endif

	eepromWriteArray(EEPROM_ADDR_EEPROMDATA, (byte *)&eeprom, sizeof(eeprom));
	unsigned long eepromCRC = eepromCalcCRC();
	eepromWriteArray(EEPROM_ADDR_CRC32, (byte *)&eepromCRC, sizeof(eepromCRC));
}

/* ===================================================================
 * Function:	eepromReadVars
 * Return:
 * Arguments:
 * Description: Lese alle EEPROM Programmvariablen aus EEPROM
 * ===================================================================*/
void eepromReadVars()
{
	#ifdef DEBUG_OUTPUT_EEPROM
	SerialTimePrintf(F("EEPROM - eepromReadVars()\r\n"));
	#endif
	eepromReadArray(EEPROM_ADDR_EEPROMDATA, (byte *)&eeprom, sizeof(eeprom));
}
