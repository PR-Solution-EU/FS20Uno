// EEPROM Funktionen

/*	====================================================================
	Function:	 eepromCalcCRC
	Return:
	Arguments:
	Description: Berechnet CRC32 Summe über alle Daten
				 (exklusive der gespeicherten CRC32 Summe selbst)
	====================================================================
*/
unsigned long eepromCalcCRC(void)
{
	const unsigned long crc_table[16] = {
		0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
		0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
		0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
		0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
	};
	unsigned long crc = ~0L;

	for (int index = 0 ; index < EEPROM.length()  ; ++index) {
		if( index<EEPROM_ADDR_CRC32 || index>=(EEPROM_ADDR_CRC32+4)) {
			crc = crc_table[(crc ^ EEPROM[index]) & 0x0f] ^ (crc >> 4);
			crc = crc_table[(crc ^ (EEPROM[index] >> 4)) & 0x0f] ^ (crc >> 4);
			crc = ~crc;
		}
	}
#ifdef DEBUG_OUTPUT_EEPROM
	printUptime();
	Serial.print(F("eepromCalcCRC() returns 0x"));
	Serial.println(crc,HEX);
#endif
	return crc;
}

/*	====================================================================
	Function:	 eepromReadLong
	Return:		 EEPROM Datum
	Arguments:   EEPROM Adresse
	Description: Liest 4 Byte als Zahl aus dem EEPROM
	====================================================================
*/
unsigned long eepromReadLong(int address)
{
	unsigned long data = 0L;

	for (byte i=0 ; i<4; ++i) {
		data <<= 8;
		data &= 0xffffff00;
		data |= EEPROM[address+i];
	}
#ifdef DEBUG_OUTPUT_EEPROM
	printUptime();
	Serial.print(F("eepromReadLong(0x"));
	Serial.print(address,HEX);
	Serial.print(F(") returns 0x"));
	Serial.println(data,HEX);
#endif
	return data;
}


/*	====================================================================
	Function:	 eepromWriteLong
	Return:		 
	Arguments:   EEPROM Adresse, EEPROM Datum
	Description: Schreibt 4 Byte als Zahl in das EEPROM
	====================================================================
*/
void eepromWriteLong(int address, unsigned long data)
{
#ifdef DEBUG_OUTPUT_EEPROM
	printUptime();
	Serial.print(F("eepromWriteLong(0x"));
	Serial.print(address,HEX);
	Serial.print(F(",0x"));
	Serial.print(data,HEX);
	Serial.println(F(")"));
#endif
	for (char i=3 ; i>=0; --i) {
		EEPROM.update(address+i, (byte)(data & 0xff));
		data >>= 8;
	}
}
