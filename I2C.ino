/*	====================================================================
	Function:	 expanderWriteWord
	Return:		 
	Arguments:   
	Description: write a word to the expander
	====================================================================
*/
void expanderWriteWord(const byte address, const byte reg, const unsigned int data ) 
{
	Wire.beginTransmission((const byte)(0x20|address));
	Wire.write(reg);
	Wire.write( (byte)(data & 0xff) );  		// port A
	Wire.write( (byte)((data>>8) & 0xff) ); 	// port B
	Wire.endTransmission();
}

/*	====================================================================
	Function:	 expanderWriteBoth
	Return:		 
	Arguments:   
	Description: write a byte to both ports of the expander
	====================================================================
*/
void expanderWriteBoth(const byte address, const byte reg, const byte data ) 
{
	Wire.beginTransmission((const byte)(0x20|address));
	Wire.write(reg);
	Wire.write(data);  // port A
	Wire.write(data);  // port B
	Wire.endTransmission();
}

/*	====================================================================
	Function:	 expanderWrite
	Return:		 
	Arguments:   
	Description: write a byte to the expander
	====================================================================
*/
void expanderWrite(const byte address, const byte reg, const byte data ) 
{
	Wire.beginTransmission ((const byte)(0x20|address));
	Wire.write(reg);
	Wire.write(data);  // port A
	Wire.endTransmission();
}

/*	====================================================================
	Function:	 expanderRead
	Return:		 
	Arguments:   
	Description: read a byte from the expander
	====================================================================
*/
byte expanderRead(const byte address, const byte reg) 
{
	Wire.beginTransmission((const byte)(0x20|address));
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom((const byte)(0x20|address), (byte)1);
	return Wire.read();
}

/*	====================================================================
	Function:	  expanderReadWord
	Return:		 
	Arguments:   
	Description: read a word from the expander
	====================================================================
*/
unsigned int expanderReadWord(const byte address, const byte reg)
{
	return expanderRead(address, reg) | (expanderRead(address, reg+1)<<8);
}
