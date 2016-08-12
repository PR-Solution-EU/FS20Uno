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
 * Function:	eepromReadLong
 * Return:		EEPROM Datum
 * Arguments:   EEPROM Adresse
 * Description: Liest 4 Byte als Zahl aus dem EEPROM
 * ===================================================================*/
unsigned long eepromReadLong(int address)
{
	unsigned long data = 0L;

	for (byte i=0 ; i<4; ++i) {
		data <<= 8;
		data &= 0xffffff00;
		data |= EEPROM[address+i];
	}
	#ifdef DEBUG_OUTPUT_EEPROM
	SerialTimePrintf(F("EEPROM - eepromReadLong(0x%04x) returns 0x%08lx [%ld]\r\n"), address, data, data);
	#endif
	return data;
}


/* ===================================================================
 * Function:	eepromWriteLong
 * Return:
 * Arguments:   EEPROM Adresse, EEPROM Datum
 * Description: Schreibt 4 Byte als Zahl in das EEPROM
 * ===================================================================*/
void eepromWriteLong(int address, unsigned long data)
{
	#ifdef DEBUG_OUTPUT_EEPROM
	SerialTimePrintf(F("EEPROM - eepromWriteLong(0x%04x, 0x%08lx [%ld] )\r\n"), address, data, data);
	#endif
	for (char i=3 ; i>=0; --i) {
		EEPROM.update(address+i, (byte)(data & 0xff));
		data >>= 8;
	}
}

/* ===================================================================
 * Function:	eepromReadWord
 * Return:		EEPROM Datum
 * Arguments:   EEPROM Adresse
 * Description: Liest 2 Byte als Zahl aus dem EEPROM
 * ===================================================================*/
unsigned int eepromReadWord(int address)
{
	unsigned int data = 0;

	for (byte i=0 ; i<2; ++i) {
		data <<= 8;
		data &= 0xff00;
		data |= EEPROM[address+i];
	}
	#ifdef DEBUG_OUTPUT_EEPROM
	SerialTimePrintf(F("EEPROM - eepromReadWord(0x%04x) returns 0x%04x [%d]\r\n"), address, data, data);
	#endif
	return data;
}


/* ===================================================================
 * Function:	eepromWriteWord
 * Return:
 * Arguments:   EEPROM Adresse, EEPROM Datum
 * Description: Schreibt 2 Byte als Zahl in das EEPROM
 * ===================================================================*/
void eepromWriteWord(int address, unsigned int data)
{
	#ifdef DEBUG_OUTPUT_EEPROM
	SerialTimePrintf(F("EEPROM - eepromWriteWord(0x%04x, 0x%04x [%d] )\r\n"), address, data, data);
	#endif
	for (char i=1 ; i>=0; --i) {
		EEPROM.update(address+i, (byte)(data & 0xff));
		data >>= 8;
	}
}

/* ===================================================================
 * Function:	eepromReadByte
 * Return:		EEPROM Datum
 * Arguments:   EEPROM Adresse
 * Description: Liest 1 Byte als Zahl aus dem EEPROM
 * ===================================================================*/
byte eepromReadByte(int address)
{
	byte data = 0;

	data = EEPROM[address];
	#ifdef DEBUG_OUTPUT_EEPROM
	SerialTimePrintf(F("EEPROM - eepromReadByte(0x%04x) returns 0x%02x [%d]\r\n"), address, data, data);
	#endif
	return data;
}


/* ===================================================================
 * Function:	eepromWriteByte
 * Return:
 * Arguments:   EEPROM Adresse, EEPROM Datum
 * Description: Schreibt 1 Byte als Zahl in das EEPROM
 * ===================================================================*/
void eepromWriteByte(int address, byte data)
{
	#ifdef DEBUG_OUTPUT_EEPROM
	SerialTimePrintf(F("EEPROM - eepromWriteByte(0x%04x, 0x%02x [%d] )\r\n"), address, data, data);
	#endif
	EEPROM.update(address, data);
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

	// Write data version into EEPROM before checking CRC32
	eepromWriteLong(EEPROM_ADDR_DATAVERSION, DATAVERSION);

	unsigned long dataCRC = eepromCalcCRC();
	unsigned long eepromCRC = eepromReadLong(EEPROM_ADDR_CRC32);

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

		eepromBlinkInterval = LED_BLINK_INTERVAL;
		eepromBlinkLen = LED_BLINK_LEN;
		eepromMTypeBitmask = MTYPE_BITMASK;
		for(i=0; i<MAX_MOTORS; i++) {
			eepromMaxRuntime[i] = bitRead(MTYPE_BITMASK,i)!=0?MOTOR_WINDOW_MAXRUNTIME:MOTOR_JALOUSIE_MAXRUNTIME;
		}
		bitSet(eepromRain, RAIN_BIT_AUTO);
		bitClear(eepromRain, RAIN_BIT_ENABLE);
		eepromCmdSendStatus = DEFAULT_CMDSENDSTATUS;
		eepromCmdEcho = DEFAULT_CMDECHO;
		eepromCmdTerm = DEFAULT_CMDTERM;

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
	SerialTimePrintf(F("EEPROM -   eepromBlinkInterval: %d\r\n"),     eepromBlinkInterval);
	SerialTimePrintf(F("EEPROM -   eepromBlinkLen:      %d\r\n"),     eepromBlinkLen);
	SerialTimePrintf(F("EEPROM -   eepromMTypeBitmask:  0x%02x\r\n"), eepromMTypeBitmask);
	SerialTimePrintf(F("EEPROM -   eepromMaxRuntime:    "));
	for(i=0; i<MAX_MOTORS; i++) {
		SerialPrintf(F("%s%ld"), i?",":"", eepromMaxRuntime[i]);
	}
	SerialPrintf(F("\r\n"));
	SerialTimePrintf(F("EEPROM -   eepromRain:          0x%02x\r\n"), eepromRain);
	SerialTimePrintf(F("EEPROM -   eepromCmdSendStatus: %s\r\n"), eepromCmdSendStatus?"yes":"no");
	SerialTimePrintf(F("EEPROM -   eepromCmdEcho:       %s\r\n"), eepromCmdEcho?"yes":"no");
	SerialTimePrintf(F("EEPROM -   eepromCmdTerm:       %s\r\n"), eepromCmdTerm=='\r'?"CR":"LF");
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

	if ( varType & EEPROM_LED_BLINK_INTERVAL ) {
		eepromWriteLong(EEPROM_ADDR_LED_BLINK_INTERVAL, eepromBlinkInterval);
	}
	if ( varType & EEPROM_LED_BLINK_LEN ) {
		eepromWriteLong(EEPROM_ADDR_LED_BLINK_LEN, eepromBlinkLen);
	}
	if ( varType & EEPROM_MTYPE_BITMASK ) {
		eepromWriteLong(EEPROM_ADDR_MTYPE_BITMASK, eepromMTypeBitmask);
	}
	if ( varType & EEPROM_MOTOR_MAXRUNTIME ) {
		for(byte i=0; i<MAX_MOTORS; i++) {
			eepromWriteLong(EEPROM_ADDR_MOTOR_MAXRUNTIME+(4*i), eepromMaxRuntime[i]);
		}
	}
	if ( varType & EEPROM_RAIN ) {
		eepromWriteByte(EEPROM_ADDR_RAIN, eepromRain);
	}
	if ( varType & EEPROM_SENDSTATUS ) {
		eepromWriteByte(EEPROM_ADDR_SENDSTATUS, eepromCmdSendStatus);
	}
	if ( varType & EEPROM_ECHO ) {
		eepromWriteByte(EEPROM_ADDR_ECHO, eepromCmdEcho);
	}
	if ( varType & EEPROM_TERM ) {
		eepromWriteByte(EEPROM_ADDR_TERM, eepromCmdTerm);
	}
	eepromWriteLong(EEPROM_ADDR_CRC32, eepromCalcCRC());
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
	eepromBlinkInterval	= eepromReadLong(EEPROM_ADDR_LED_BLINK_INTERVAL);
	eepromBlinkLen		= eepromReadLong(EEPROM_ADDR_LED_BLINK_LEN);
	eepromMTypeBitmask 	= eepromReadLong(EEPROM_ADDR_MTYPE_BITMASK);
	for(byte i=0; i<MAX_MOTORS; i++) {
		eepromMaxRuntime[i]	= eepromReadLong(EEPROM_ADDR_MOTOR_MAXRUNTIME+(4*i));
	}
	eepromRain = eepromReadByte(EEPROM_ADDR_RAIN);
	eepromCmdSendStatus = eepromReadByte(EEPROM_ADDR_SENDSTATUS);
	eepromCmdEcho = eepromReadByte(EEPROM_ADDR_ECHO);
	eepromCmdTerm = eepromReadByte(EEPROM_ADDR_TERM);
}
