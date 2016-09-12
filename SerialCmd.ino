/* ===================================================================
 * Command Interface Functions
 * ===================================================================*/
SerialCommand SCmd;   		// SerialCommand object


void setupSerialCommand(void)
{
	// Setup callbacks for SerialCommand commands
	SCmd.addCommand(PSTR("?"),cmdHelp);
	SCmd.addCommand(PSTR("HELP"),cmdHelp);
	SCmd.addCommand(PSTR("INFO"),cmdInfo);
	SCmd.addCommand(PSTR("ECHO"),cmdEcho);
	SCmd.addCommand(PSTR("TERM"),cmdTerm);
	SCmd.addCommand(PSTR("UPTIME"),cmdUptime);
	SCmd.addCommand(PSTR("FS20"),cmdFS20);
	SCmd.addCommand(PSTR("BUTTON"),cmdWallButton);
	SCmd.addCommand(PSTR("PUSHBUTTON"),cmdWallButton);
	SCmd.addCommand(PSTR("WALLBUTTON"),cmdWallButton);
	SCmd.addCommand(PSTR("MOTOR"),cmdMotor);
	SCmd.addCommand(PSTR("MOTORNAME"),cmdMotorName);
	SCmd.addCommand(PSTR("MOTORTIME"),cmdMotorTime);
	SCmd.addCommand(PSTR("MOTORTYPE"),cmdMotorType);
	SCmd.addCommand(PSTR("RAINSENSOR"),cmdRainSensor);
	SCmd.addCommand(PSTR("RAIN"),cmdRainSensor);
	SCmd.addCommand(PSTR("STATUS"),cmdStatus);
	SCmd.addCommand(PSTR("LED"),cmdLed);
	SCmd.addCommand(PSTR("FACTORYRESET"),cmdFactoryReset);
	SCmd.addCommand(PSTR("BACKUP"),cmdBackup);
	SCmd.addCommand(PSTR("RESTORE"),cmdRestore);
	SCmd.addCommand(PSTR("RESET"),cmdReset);
	SCmd.addDefaultHandler(unrecognized);   // Handler for command that isn't matched  (says "What?")
	SCmd.setEcho(eeprom.Echo);
	SCmd.setTerm(eeprom.Term);
}

void processSerialCommand(void)
{
	SCmd.readSerial();     // We don't do much, just process serial commands
	watchdogReset();
}


void cmdHelp()
{
	#ifndef DEBUG_OUTPUT
	#ifndef CMDHELP_LONG
	Serial.print(F(
		"Command List\r\n"
		"-------------\r\n"
		"HELP\r\n"
		"\tThis help\r\n"
		"INFO\r\n"
		"\tProgram version\r\n"
		"ECHO [ON|OFF]\r\n"
		"\tLocal echo\r\n"
		"TERM [CR|LF]\r\n"
		"\tCommand terminator\r\n"
		"STATUS [ON|OFF]\r\n"
		"\tStatus messages\r\n"
		"UPTIME [h]\r\n"
		"\tSystem uptime\r\n"
		"FS20 [<ch> [ON|OFF|PRG]]\r\n"
		"\tFS20 control\r\n"
		"LED [<interval> <flash>]\r\n"
		"\tLED blinking\r\n"
		"MOTOR [<m> [[T]OPEN|[T]CLOSE|TOOGLE|GOTO <pos>|<pos>|OFF|SYNC|STATUS]\r\n"
		"\tMotor control\r\n"
		"MOTORNAME [<m> [<name>]\r\n"
		"\tMotor names\r\n"
		"MOTORTIME [<m> [<sec> [<overtravel>]]\r\n"
		"\tMotor runtime\r\n"
		"MOTORTYPE [<m> [WINDOW|JALOUSIE]\r\n"
		"\tMotor type\r\n"
		"RAIN, RAINSENSOR [AUTO|ENABLE|DISABLE|WET|ON|DRY|OFF|RESUME <delay>|FORGET]\r\n"
		"\tRain sensor function\r\n"
		"BUTTON, PUSHBUTTON, WALLBUTTON [<b> [ON|OFF]]\r\n"
		"\tWall pushbutton status\r\n"
		"BACKUP\r\n"
		"\tOutput EEPROM data for backup\r\n"
		"RESTORE <addr> <data>\r\n"
		"\tWrite data into EEPROM\r\n"
		"FACTORYRESET\r\n"
		"\tSet factory defaults\r\n"
		"RESET\r\n"
		"\tRestart controller\r\n"
		"\r\n"
		));
	#else
	char *arg;

	arg = SCmd.next();
	if (arg == NULL) {
		Serial.print(F(
			"Command List\r\n"
			"-------------\r\n"
			"HELP         Command help\r\n"
			"INFO         Program version\r\n"
			"ECHO         Local echo\r\n"
			"TERM         Command terminator\r\n"
			"STATUS       Status messages\r\n"
			"UPTIME       System uptime\r\n"
			"FS20         FS20 control\r\n"
			"LED          LED alive blinking parameter\r\n"
			"MOTOR        Motor control\r\n"
			"MOTORNAME    Motor names\r\n"
			"MOTORTIME    Motor runtime\r\n"
			"MOTORTYPE    Motor type\r\n"
			"RAINSENSOR   Rain sensor function\r\n"
			"PUSHBUTTON   Wall pushbutton status\r\n"
			"BACKUP       Output EEPROM data for backup\r\n"
			"RESTORE      Write data into EEPROM\r\n"
			"FACTORYRESET Set factory defaults\r\n"
			"RESET        Restart controller\r\n"
			"\r\n"
			"Get value: Use cmd without parameter\r\n"
			"Set value: Use cmd with parameters.\r\n"
			"\r\n"
			"For cmd details use 'HELP <cmd>' (e.g. HELP INFO)\r\n"
			"\r\n"
			));
	}
	else if ( strnicmp(arg, F("EC"),2)==0 ) {
		Serial.print(F(
			"ECHO [ON|OFF]\r\n"
			"\tSwitch echo on or off\r\n"
			));
	}
	else if ( strnicmp(arg, F("FA"),2)==0 ) {
		Serial.print(F(
			"FACTORYRESET\r\n"
			"\tSet factory defaults\r\n"
			));
	}
	else if ( strnicmp(arg, F("FS"),2)==0 ) {
		Serial.print(F(
			"FS20 [<ch> [ON|OFF|PRG]]\r\n"
			"\tFS20 receiver channel\r\n"
			"\t\t<ch> FS20 channel number [1..c]\r\n"
			"\t\tON   switch channel <ch> ON\r\n"
			"\t\tOFF  switch channel <ch> OFF\r\n"
			"\t\tPRG  switch channel <ch> into programming mode\r\n"
			));
	}
	else if ( strnicmp(arg, F("HE"),2)==0 ) {
		Serial.print(F(
			"HELP, ?\r\n"
			"\tPrint help\r\n"
			));
	}
	else if ( strnicmp(arg, F("IN"),2)==0 ) {
		Serial.print(F(
			"INFO\r\n"
			"\tDisplay system info\r\n"
			));
	}
	else if ( strnicmp(arg, F("LE"),2)==0 ) {
		Serial.print(F(
			"LED [<int> <flash>]\r\n"
			"\tLED alive blinking parameter\r\n"
			"\t\tint   blinkinterval in ms\r\n"
			"\t\tflash flash duration in ms\r\n"
			));
	}
	else if ( strnicmp(arg, F("MOTOR"),5)==0 ) {
		Serial.print(F(
			"MOTOR [<m> [<cmd> [<p>]]\r\n"
			"\tSet/Get motor status\r\n"
			"\t\t<m>      Motor number [1..m]\r\n"
			"\t\t<cmd>    can be\r\n"
			"\t\t\tOPEN     Motor in OPEN direction\r\n"
			"\t\t\tCLOSE    Motor in CLOSE direction\r\n"
			"\t\t\tTOPEN    Toogle OPEN direction\r\n"
			"\t\t\tTCLOSE   Toogle CLOSE direction\r\n"
			"\t\t\tTOOGLE   Toogle direction\r\n"
			"\t\t\tGOTO <p> Goto position <p> (in %, 0-100)\r\n"
			"\t\t\t<p>      Goto position <p> (in %, 0-100)\r\n"
			"\t\t\tOFF      Stop motor\r\n"
			"\t\t\tSYNC     Set motor in a default defined state\r\n"
			"\t\t\tSTATUS   Return the current status\r\n"
			));
			
	}
	else if ( strnicmp(arg, F("MOTORNA"),7)==0 ) {
		Serial.print(F(
			"MOTORNAME [<m> [<name>]\r\n"
			"\tSet/Get motor name\r\n"
			"\t\t<m>   Motor number [1..m]\r\n"
			"\t\t<name Motor name\r\n"
			));
	}
	else if ( strnicmp(arg, F("MOTORTI"),7)==0 ) {
		Serial.print(F(
			"MOTORTIME [<m> [<sec> [<overtravel>]]\r\n"
			"\tSet/Get motor maximum runtime\r\n"
			"\t\t<m>          Motor number [1..m]\r\n"
			"\t\t<sec>        Maximum runtime [s]\r\n"
			"\t\t<overtravel> Overtravel time [s]\r\n"
			));
	}
	else if ( strnicmp(arg, F("MOTORTY"),7)==0 ) {
		Serial.print(F(
			"MOTORTYPE [<m> [<cmd>]\r\n"
			"\tSet/Get motor type\r\n"
			"\t\t<m>   Motor number [1..m]\r\n"
			"\t\t<cmd> can be WINDOW or JALOUSIE\r\n"
			));
	}
	else if ( strnicmp(arg, F("RA"),2)==0 ) {
		Serial.print(F(
			"RAIN, RAINSENSOR [<cmd>]\r\n"
			"\tSet/Get rain sensor\r\n"
			"\t\t<cmd>    can be\r\n"
			"\t\t\tAUTO       Rain detection enabled from input signals\r\n"
			"\t\t\tENABLE     Enable rain detection\r\n"
			"\t\t\tDISABLE    Disable rain detection\r\n"
			"\t\t\tWET|ON     Raining\r\n"
			"\t\t\tDRY|OFF    Dry\r\n"
			"\t\t\tRESUME <s> Resume window position after rain was gone\r\n"
			"\t\t\t           <s> is the delay (sec) before resume starts\r\n"
			"\t\t\tFORGET     Don not resume, keep windows closed\r\n"
			"ENABLE, DISABLE, ON, OFF disables AUTO"
			));
			
	}
	else if ( strnicmp(arg, F("PU"),2)==0 ) {
		Serial.print(F(
			"BUTTON, PUSHBUTTON, WALLBUTTON [<b> [ON|OFF]]\r\n"
			"\tSet/Get wall pushbutton status\r\n"
			"\t\t<b>    Wall button number [1..m]\r\n"
			"\t\tON|OFF set new value\r\n"
			));
	}
	else if ( strnicmp(arg, F("ST"),2)==0 ) {
		Serial.print(F(
			"STATUS [ON|OFF]\r\n"
			"\tSystem status message\r\n"
			"\t\tON  Status messages enabled\r\n"
			"\t\t\t  Automatically send status messages when system status changes\r\n"
			"\t\tOFF Status messages disabled\r\n"
			"\t\t\t  No messages are send on system status changes\r\n"
			));
	}
	else if ( strnicmp(arg, F("TE"),2)==0 ) {
		Serial.print(F(
			"TERM [CR|LF]\r\n"
			"\tCommand terminator\r\n"
			));
	}
	else if ( strnicmp(arg, F("UP"),2)==0 ) {
		Serial.print(F(
			"UPTIME [<uptime> [<h>]]\r\n"
			"\tSystem uptime\r\n"
			"\t<uptime> Set uptime (in ms since start)\r\n"
			"\t<h>      Set operation hours (in h)\r\n"
			));
	}
	else if ( strnicmp(arg, F("BA"),2)==0 ) {
		Serial.print(F(
			"BACKUP\r\n"
			"\tOutput EEPROM data for backup\r\n"
			));
	}
	else if ( strnicmp(arg, F("RESTO"),5)==0 ) {
		Serial.print(F(
			"RESTORE <addr> <data>\r\n"
			"\tWrite data into EEPROM (format see BACKUP)\r\n"
			"\t\taddr  4 digit hex address for restoring data\r\n"
			"\t\tdata  string of data to write\r\n"
			));
	}
	else if ( strnicmp(arg, F("RESET"),5)==0 ) {
		Serial.print(F(
			"RESET\r\n"
			"\tSoft restart the controller\r\n"
			));
	}
	#endif
	#else
	Serial.print(F("Debug enabled, no help"));

	#endif
	watchdogReset();
	
	printCRLF();
}

void cmdBackup(void)
{
	byte m;

	SerialPrintf(F("Binary data (%d byte):\r\n"), (int)sizeof(eeprom));

	for(size_t i=0; i<sizeof(eeprom)+4; i++) {
		if ( (i % 16)==0 ) {
			SerialPrintf(F("RESTORE %04x "), i);
		}
		SerialPrintf(F("%02x"), EEPROM.read(i) );
		if ( (i % 16)==15 ) {
			printCRLF();
		}
	}
	printCRLF();
	printCRLF();

	// ECHO
	SerialPrintf(F("ECHO %S\r\n"), eeprom.Echo?fstrON:fstrOFF);
	// TERM
	SerialPrintf(F("TERM %S\r\n"), eeprom.Term=='\r'?F("CR"):F("LF"));
	// STATUS
	SerialPrintf(F("STATUS %S\r\n"), eeprom.SendStatus?fstrON:fstrOFF);
	// UPTIME
	SerialPrintf(F("UPTIME %ld %ld\r\n"), millis(), eeprom.OperatingHours);
	// LED
	SerialPrintf(F("LED %d %d\r\n"), eeprom.BlinkInterval, eeprom.BlinkLen);

	// RAIN
	SerialPrintf(F("RAIN RESUME %d\r\n"), eeprom.RainResumeTime);
	if( !bitRead(eeprom.Rain, RAIN_BIT_RESUME) ) { 
		SerialPrintf(F("RAIN FORGET\r\n"));
	}
	SerialPrintf(F("RAIN %S\r\n"), bitRead(eeprom.Rain, RAIN_BIT_ENABLE) ? F("ENABLE") : F("DISABLE"));
	if( bitRead(eeprom.Rain, RAIN_BIT_AUTO) ) {
		SerialPrintf(F("RAIN AUTO\r\n"));
	}

	// MOTORNAME
	for(m=0; m<MAX_MOTORS; m++) {
		char sName[MAX_NAMELEN];
		
		strcpy(sName, eeprom.MotorName[m]);
		strReplaceChar(sName, ' ', '_');
		SerialPrintf(F("MOTORNAME %d %s\r\n"), m+1, sName);
	}
	// MOTORTYPE
	for(m=0; m<MAX_MOTORS; m++) {
		SerialPrintf(F("MOTORTYPE %d %S\r\n"), m+1, bitRead(MTYPE_BITMASK,m)!=0?F("WIN"):F("JAL"));
	}
	// MOTORTIME
	for(m=0; m<MAX_MOTORS; m++) {
		SerialPrintf(F("MOTORTIME %d %d.%03d %d.%03d\r\n")
			,m+1
			,(int)(eeprom.MaxRuntime[m] / 1000)
			,(int)(eeprom.MaxRuntime[m] % 1000)
			,(int)(eeprom.OvertravelTime[m] / 1000)
			,(int)(eeprom.OvertravelTime[m] % 1000)
			);
	}

	// MOTOR POSITIONS
	for(m=0; m<MAX_MOTORS; m++) {
		SerialPrintf(F("MOTOR %d SYNC\r\n"), m+1);
	}

	cmdOK();
}
void cmdRestore(void)
{
//~ "RESTORE <addr> <data>\r\n"
//~ "\tWrite data into EEPROM (format see BACKUP)\r\n"
//~ "\t\taddr  4 digit hex address for restoring data\r\n"
//~ "\t\tdata  string of data to write\r\n"
	char *arg;

	arg = SCmd.next();
	if (arg == NULL ) {
		cmdError(F("Missing address"));
	}
	else {
		unsigned int addr;
		unsigned long strVal;

		strVal = strtoul(arg, NULL, 16);
		if ( errno == ERANGE ) {
			cmdError(F("Wrong address syntax"));
		}
		else {
			addr = (unsigned int)strVal;
			arg = SCmd.next();
			if (arg == NULL || strlen(arg)<2 || (strlen(arg) % 2)!=0 ) {
				cmdError(F("Wrong data"));
			}
			else {
				bool fError = false;
				for(byte i=0; i<strlen(arg) && !fError; i+=2) {
					char sByte[3];
					byte data;
					
					sByte[0] = *(arg+i);
					sByte[1] = *(arg+i+1);
					sByte[2] = '\0';
					strVal = strtoul(sByte, NULL, 16);
					if ( errno == ERANGE ) {
						cmdError(F("Wrong data"));
						fError = true;
					}
					else {
						data = (byte)strVal;
						#ifdef DEBUG_OUTPUT_CMD_RESTORE
						SerialTimePrintfln(F("RESTORE %04x: %02x"), addr, data);
						#endif
						EEPROM.put(addr++, data);
					}
				}
				if ( !fError ) {
					cmdOK();
				}
			}
		}
	}
}

void cmdInfo()
{
	printProgramInfo(false);
	cmdUptime();
	SerialPrintfln(F("EEPROM CRC32: %08lx"), CalcCRC(RAMCRC, (byte *)&eeprom, sizeof(eeprom)));
	cmdOK();
}

void cmdUptime()
{
//~ "UPTIME [<uptime> [<h>]]\r\n"
//~ "\tSystem uptime\r\n"
//~ "\t<uptime> Set uptime (in ms since start)\r\n"
//~ "\t<h>      Set operation hours (in h)\r\n"
	char *arg;
	
	arg = SCmd.next();
	if (arg != NULL) {

		cli(); //halt the interrupts
		timer0_millis =  (unsigned long)atol(arg);
		sei(); //re-enable the interrupts
		operatonHours(true);

		arg = SCmd.next();
		if (arg != NULL) {
			eeprom.OperatingHours=atol(arg);
			eepromWriteVars();
		}
	}
	SerialPrintf(  F("Uptime:       "));
	SerialPrintUptime();;
	printCRLF();
	SerialPrintfln(F("Operating:    %ld h"), eeprom.OperatingHours);
	cmdOK();
}

void cmdFS20()
{
//~ "    fs20 [<ch> [<cmd> [ON|OFF]]]\r\n"
//~ "         Set/Get FS20 control\r\n"
//~ "         <ch>     FS20 channel number [1..c]\r\n"
//~ "         ON       switch channel <ch> ON\r\n"
//~ "         OFF      switch channel <ch> OFF\r\n"
//~ "         PRG      switch channel <ch> into programming mode\r\n"
	int channel;
	char *arg;

	arg = SCmd.next();
	if (arg == NULL) {
		for(channel=0; channel<IOBITS_CNT; channel++) {
			sendStatus(true,FS20IN, F("%02d %S"), channel+1, bitRead(curSM8Status,channel)?fstrON:fstrOFF);
		}
		cmdOK();
	}
	else {
		// Channel number entered
		channel=atoi(arg)-1;
		if ( channel<0 || channel>=IOBITS_CNT ) {
			cmdError(F("Channel number out of range"));
		}
		else {
			// Channel number ok
			arg = SCmd.next();
			if (arg != NULL) {
				// ON|OFF entered?
				if      ( strnicmp(arg, fstrON,2)==0 ) {
					if( !bitRead(curSM8Status, channel) ) {
						bitClear(valSM8Button, channel);
					}
					cmdOK();
				}
				else if ( strnicmp(arg, F("OF"),2)==0 ) {
					if( bitRead(curSM8Status, channel) ) {
						bitClear(valSM8Button, channel);
					}
					cmdOK();
				}
				else if ( strnicmp(arg, F("PRG"),3)==0 ) {
					bitSet(SM8StatusIgnore, channel);
					bitClear(valSM8Button, channel);
					expanderWriteWord(MPC_SM8BUTTON,   GPIO, valSM8Button);
					SerialPrintf(F("Setting channel %d into program mode, please wait:  "), channel+1);
					for(byte d=0; d<(FS20_SM8_IN_PROGRAMMODE/1000); d++) {
						SerialPrintf(F("\b%1d"), (FS20_SM8_IN_PROGRAMMODE/1000)-d);
						delay(1000);
						watchdogReset();
					}
					bitSet(SM8StatusIgnore, channel);
					bitClear(valSM8Button, channel);
					expanderWriteWord(MPC_SM8BUTTON,   GPIO, valSM8Button);
					SerialPrintfln(F("\r\nChannel %d now in programming mode"), channel+1);
					cmdOK();
				}
				else {
					cmdErrorParameter(F("'ON', 'OFF' or 'PRG'"));
				}
			}
			else {
				sendStatus(true,FS20IN, F("%02d %S"), channel+1, bitRead(curSM8Status,channel)?fstrON:fstrOFF);
				cmdOK();
			}
		}
	}
}

void cmdWallButton()
{
//~ "    button | wall | wallbutton [<b> [ON|OFF]]\r\n"
//~ "         Set/Get wall pushbuttons\r\n"
//~ "         <b>      Wall button number [1..m]\r\n"
//~ "         ON|OFF   set new value\r\n"
	int button;
	char *arg;

	arg = SCmd.next();
	if (arg == NULL) {
		for(button=0; button<IOBITS_CNT; button++) {
			sendStatus(true,PUSHBUTTON, F("%02d %S"), button+1, bitRead(curWallButton, button)?fstrON:fstrOFF);
		}
		cmdOK();
	}
	else {
		// Button number entered
		button=atoi(arg)-1;
		if ( button<0 || button>=IOBITS_CNT ) {
			cmdError(F("Pushbutton number out of range"));
		}
		else {
			// Button number ok
			arg = SCmd.next();
			if (arg != NULL) {
				// ON|OFF entered?
				if      ( strnicmp(arg, fstrON,2)==0 ) {
					bitSet(curWallButton, button);
					cmdOK();
				}
				else if ( strnicmp(arg, F("OF"),2)==0 ) {
					bitClear(curWallButton, button);
					cmdOK();
				}
				else {
					cmdErrorParameter(F("'ON' or 'OFF'"));
				}
			}
			else {
				sendStatus(true,PUSHBUTTON, F("%02d %S"), button+1, bitRead(curWallButton, button)?fstrON:fstrOFF);
				cmdOK();
			}
		}
	}
}


void cmdMotor()
{
//~ "MOTOR [<m> [<cmd> [<param>]]\r\n"
//~ "  Set/Get motor status\r\n"
//~ "     <m>      Motor number [1..m]\r\n"
//~ "     <cmd>    can be\r\n"
//~ "              OPEN     - motor in OPEN direction\r\n"
//~ "              CLOSE    - motor in CLOSE direction\r\n"
//~ "              TOPEN    - toogle OPEN direction\r\n"
//~ "              TCLOSE   - toogle CLOSE direction\r\n"
//~ "              TOOGLE   - toogle direction\r\n"
//~ "              GOTO <p> - goto <p> percent\r\n"
//~ "              <p>      - Goto position <p> (in %, 0-100)\r\n"
//~ "              OFF      - stop motor\r\n"
//~ "              SYNC     - set motor in a default defined state\r\n"
//~ "              STATUS   - return the current status\r\n"
	int motor;
	char *arg;

	arg = SCmd.next();
	if (arg == NULL) {
		for(motor=0; motor<MAX_MOTORS; motor++) {
			sendMotorStatus(true,motor);
		}
		cmdOK();
	}
	else {
		motor=atoi(arg)-1;
		if ( motor<0 || motor>=MAX_MOTORS ) {
			cmdError(F("Motor number out of range"));
		}
		else {
			arg = SCmd.next();
			if (arg != NULL) {
				if      ( strnicmp(arg, F("OP"),2)==0 ) {
					if( getMotorDirection(motor)<MOTOR_OPEN ) {
						setMotorDirection(motor, MOTOR_OPEN);
					}
					cmdOK();
				}
				else if ( strnicmp(arg, F("CL"),2)==0 ) {
					if( getMotorDirection(motor)>MOTOR_CLOSE ) {
						setMotorDirection(motor, MOTOR_CLOSE);
					}
					cmdOK();
				}
				else if ( strnicmp(arg, F("OF"),2)==0 ) {
					if( getMotorDirection(motor)!=MOTOR_OFF ) {
						setMotorDirection(motor, MOTOR_OFF);
					}
					cmdOK();
				}
				else if ( strnicmp(arg, F("GO"),2)==0 ) {
					arg = SCmd.next();
					int percent=atoi(arg);

					if ( percent<0 || percent>100 ) {
						cmdError(F("Parameter out of range (0-100)"));
					}
					else {
						setMotorPosition(motor, percent);
						cmdOK();
					}
				}
				else if ( strnicmp(arg, F("TOP"),3)==0 ) {
					if( getMotorDirection(motor)==MOTOR_OFF || getMotorDirection(motor)<=MOTOR_CLOSE ) {
						setMotorDirection(motor, MOTOR_OPEN);
					}
					else {
						setMotorDirection(motor, MOTOR_OFF);
					}
					cmdOK();
				}
				else if ( strnicmp(arg, F("TCL"),3)==0 ) {
					if( getMotorDirection(motor)==MOTOR_OFF || getMotorDirection(motor)>=MOTOR_OPEN  ) {
						setMotorDirection(motor, MOTOR_CLOSE);
					}
					else {
						setMotorDirection(motor, MOTOR_OFF);
					}
					cmdOK();
				}
				else if ( strnicmp(arg, F("TOG"),3)==0 ) {
					if( getMotorDirection(motor)==MOTOR_OFF ) {
						setMotorDirection(motor, MOTOR_OPEN);
					}
					else if( getMotorDirection(motor)>MOTOR_CLOSE ) {
						setMotorDirection(motor, MOTOR_CLOSE);
					}
					else if( getMotorDirection(motor)<MOTOR_OPEN ) {
						setMotorDirection(motor, MOTOR_OPEN);
					}
					cmdOK();
				}
				else if ( strnicmp(arg, F("SY"),2)==0 ) {
					setMotorDirection(motor, MOTOR_CLOSE);
					cmdOK();
				}
				else if ( (atoi(arg)>=0 && atoi(arg)<=100) ) {
					setMotorPosition(motor, (byte)atoi(arg) );
					cmdOK();
				}
				else {
					cmdErrorParameter(F("'HELP MOTOR' for more info"));
				}
			}
			if (arg == NULL || strnicmp(arg, F("ST"),2)==0 ) {
				sendMotorStatus(true,motor);
				cmdOK();
			}
		}
	}
}


void cmdMotorTime()
{
//~ "MOTORTIME [<m> [<sec> [<delay>]]\r\n"
//~ "\tSet/Get motor maximum runtime\r\n"
//~ "\t\t<m>     Motor number [1..m]\r\n"
//~ "\t\t<sec>   Maximum runtime [s]\r\n"
//~ "\t\t<delay> Overtravel time [s]\r\n"
	int motor;
	double runtime;
	char *arg;

	arg = SCmd.next();
	if (arg == NULL) {
		for(motor=0; motor<MAX_MOTORS; motor++) {
			cmdMotorTimePrintStatus(motor);
		}
		cmdOK();
	}
	else {
		motor=atoi(arg)-1;
		if ( motor<0 || motor>=MAX_MOTORS ) {
			cmdError(F("Motor number out of range"));
		}
		else {
			arg = SCmd.next();

			if (arg == NULL) {
				// Status
				cmdMotorTimePrintStatus(motor);
				cmdOK();
			}
			else {
				// Set new runtime value
				runtime = atof(arg);
				if ( runtime<=0.0 || runtime>(double)(MOTOR_MAXRUNTIME/TIMER_MS) ) {
					cmdError(F("Motor runtime out of range"));
				}
				else {
					eeprom.MaxRuntime[motor] = (DWORD)(runtime*1000.0);
					eepromWriteVars();
					arg = SCmd.next();
					if (arg == NULL) {
						cmdMotorTimePrintStatus(motor);
						cmdOK();
					}
					else {
						runtime = atof(arg);
						if ( runtime<0.0 || runtime>(double)(MOTOR_MAXRUNTIME/TIMER_MS) ) {
							cmdError(F("Motor overtravel time out of range"));
						}
						else {
							eeprom.OvertravelTime[motor] = (DWORD)(runtime*1000.0);
							eepromWriteVars();
							cmdMotorTimePrintStatus(motor);
							cmdOK();
						}
					}
				}
			}
		}
	}
}
void cmdMotorTimePrintStatus(int motor)
{
	sendStatus(true,MOTOR,F("%02d TIME %d.%03d %d.%03d (%s)")
				,(int)(motor+1)
				,(int)(eeprom.MaxRuntime[motor] / 1000)
				,(int)(eeprom.MaxRuntime[motor] % 1000)
				,(int)(eeprom.OvertravelTime[motor] / 1000)
				,(int)(eeprom.OvertravelTime[motor] % 1000)
				,(char *)eeprom.MotorName[motor]
				);
}


void cmdMotorName()
{
//~ "MOTORNAME [<m> [<name>]\r\n"
//~ "  Set/Get motor name\r\n"
//~ "     <m>      Motor number [1..m]\r\n"
//~ "     <name    Motor name\r\n"
	int motor;
	char *arg;

	arg = SCmd.next();
	if (arg == NULL) {
		for(motor=0; motor<MAX_MOTORS; motor++) {
			cmdMotorNamePrintStatus(motor);
		}
		cmdOK();
	}
	else {
		motor=atoi(arg)-1;
		if ( motor<0 || motor>=MAX_MOTORS ) {
			cmdError(F("Motor number out of range"));
		}
		else {
			arg = SCmd.next();
			if (arg != NULL) {
				char sName[MAX_NAMELEN];

				#ifdef DEBUG_OUTPUT
				SerialTimePrintfln(F("cmdMotorName arg=%s"), arg);
				SerialTimePrintfln(F("cmdMotorName sizeof(eeprom.MotorName[motor])=%d"), sizeof(eeprom.MotorName[motor]) );
				SerialTimePrintfln(F("cmdMotorName &eeprom.MotorName[motor]=%p"), &eeprom.MotorName[motor] );
				#endif

				strncpy(sName, arg, sizeof(sName)-1);
				strReplaceChar(sName, '_', ' ');
				strcpy(eeprom.MotorName[motor], sName);
				eeprom.MotorName[motor][sizeof(eeprom.MotorName[motor])-1]='\0';
				eepromWriteVars();
				cmdOK();
			}
			else {
				cmdMotorNamePrintStatus(motor);
				cmdOK();
			}
		}
	}
}
void cmdMotorNamePrintStatus(int motor)
{
	sendStatus(true,MOTOR,F("%02d NAME %s"), motor+1, (char *)eeprom.MotorName[motor]);
}


void cmdMotorType()
{
//~ "    motortype [<m> [<cmd>]\r\n"
//~ "         Set/Get motor type\r\n"
//~ "         <m>      Motor number [1..m]\r\n"
//~ "         <cmd>    can be WINDOW or JALOUSIE\r\n"
	int motor;
	char *arg;

	arg = SCmd.next();
	if (arg == NULL) {
		for(motor=0; motor<MAX_MOTORS; motor++) {
			cmdMotorTypePrintStatus(motor);
		}
		cmdOK();
	}
	else {
		motor=atoi(arg)-1;
		if ( motor<0 || motor>=MAX_MOTORS ) {
			cmdError(F("Motor number out of range"));
		}
		else {
			arg = SCmd.next();
			if (arg != NULL) {
				if      ( strnicmp(arg, F("WIN"),3)==0 ) {
					setMotorType(motor, WINDOW);
					eepromWriteVars();
					cmdOK();
				}
				else if ( strnicmp(arg, F("JAL"),3)==0 ) {
					setMotorType(motor, JALOUSIE);
					eepromWriteVars();
					cmdOK();
				}
				else {
					cmdErrorParameter(F("'WINDOW' or 'JALOUSIE'"));
				}
			}
			else {
				cmdMotorTypePrintStatus(motor);
				cmdOK();
			}
		}
	}
}
void cmdMotorTypePrintStatus(int motor)
{
	sendStatus(true,MOTOR,F("%02d TYPE %-8S (%s)")
				,motor+1
				,getMotorType(motor)==WINDOW ? F("WINDOW") : F("JALOUSIE")
				,(char *)eeprom.MotorName[motor]);
}


void cmdRainSensor()
{
//~ "RAIN, RAINSENSOR [<cmd>]\r\n"
//~ "  Set/Get rain sensor\r\n"
//~ "     <cmd>    can be\r\n"
//~ "       AUTO        Rain detection and detection enabled from input signals\r\n"
//~ "       ENABLE      Enable rain detection, disables AUTO\r\n"
//~ "       DISABLE     Disable rain detection, disables AUTO\r\n"
//~ "       WET|ON      Raining, disables AUTO\r\n"
//~ "       DRY|OFF     No raining, disables AUTO\r\n"
//~ "       RESUME <s>  Resume window position after rain was gone\r\n"
//~ "                   <s> is the delay in sec after rain was gone before resume starr\r\n"
//~ "                   and before resume starts.\r\n"
//~ "       FORGET      Do not remember window position, keep it close\r\n"
	char *arg;
	bool cmd = false;

	arg = SCmd.next();
	if (arg == NULL) {
		cmdRainSensorPrintStatus();
		cmdOK();
	}
	else {
		if ( strnicmp(arg, fstrON,2)==0 || strnicmp(arg, fstrWET,3)==0 ) {
			softRainInput = true;
			bitClear(eeprom.Rain, RAIN_BIT_AUTO);
			cmd = true;
		}
		if ( strnicmp(arg, fstrOFF,3)==0 || strnicmp(arg, fstrDRY,3)==0 ) {
			softRainInput = false;
			bitClear(eeprom.Rain, RAIN_BIT_AUTO);
			cmd = true;
		}
		else if ( strnicmp(arg, F("EN"),2)==0 ) {
			bitSet(eeprom.Rain, RAIN_BIT_ENABLE);
			bitClear(eeprom.Rain, RAIN_BIT_AUTO);
			cmd = true;
		}
		else if ( strnicmp(arg, F("DI"),2)==0 ) {
			bitClear(eeprom.Rain, RAIN_BIT_ENABLE);
			bitClear(eeprom.Rain, RAIN_BIT_AUTO);
			cmd = true;
		}
		else if ( strnicmp(arg, F("AU"),2)==0 ) {
			bitSet(eeprom.Rain, RAIN_BIT_AUTO);
			cmd = true;
		}
		else if ( strnicmp(arg, F("RE"),2)==0 ) {
			int delay = eeprom.RainResumeTime;
			arg = SCmd.next();
			if( arg!=NULL ) {
				delay=atoi(arg);
			}
			if ( delay<0 || delay>6000 ) {
				cmdError(F("Parameter out of range (0-6000)"));
			}
			else {
				eeprom.RainResumeTime = delay;
				bitSet(eeprom.Rain, RAIN_BIT_RESUME);
				cmd = true;
			}
		}
		else if ( strnicmp(arg, F("FO"),2)==0 ) {
			// Clear resumeMotorPosition[]
			for(size_t i=0; i<(sizeof(resumeMotorPosition)/sizeof(resumeMotorPosition[0])); i++) {
				resumeMotorPosition[i] = NO_POSITION;
			}
			bitClear(eeprom.Rain, RAIN_BIT_RESUME);
			cmd = true;
		}
		if ( cmd ) {
			eepromWriteVars();
			cmdOK();
		}
		else {
			cmdErrorParameter(F("'ENABLE', 'DISABLE', 'AUTO', 'ON' or 'OFF'"));
		}
	}
}
void cmdRainSensorPrintStatus()
{
	bool sensorEnabled;

	debEnable.update();
	debInput.update();

	if( bitRead(eeprom.Rain, RAIN_BIT_AUTO) ) {
		sensorEnabled = (debEnable.read() == RAIN_ENABLE_AKTIV);
	}
	else {
		sensorEnabled = bitRead(eeprom.Rain, RAIN_BIT_ENABLE);
	}

	sendStatus(true,RAIN,F("%S MODE:%S IN:%S SET:%S RAIN:%S FROM:%S POS:%S DELAY:%d")
				,sensorEnabled && ((debInput.read()==RAIN_INPUT_AKTIV) || softRainInput) ? fstrWET : fstrDRY
				,bitRead(eeprom.Rain, RAIN_BIT_AUTO) ? F("AUTO") : F("MANUAL")
				,(debInput.read()==RAIN_INPUT_AKTIV) ? fstrWET : fstrDRY
				,softRainInput ? fstrWET : fstrDRY
				,sensorEnabled ? F("ENABLED") : F("DISABLED")
				,bitRead(eeprom.Rain, RAIN_BIT_AUTO) ? F("INPUT") : F("SET")
				,bitRead(eeprom.Rain, RAIN_BIT_RESUME) ? F("RESUME") : F("FORGET")
				,eeprom.RainResumeTime
				);
}

void cmdStatus()
{
//~ "    status [on|off]\r\n"
//~ "         Set/Get FS20Uno status message\r\n"
//~ "         on        Status messages enabled\r\n"
//~ "         off       Status messages disabled\r\n"
	char *arg;

	arg = SCmd.next();
	if (arg == NULL) {
		sendStatus(true,SYSTEM,F("STATUS %S"), eeprom.SendStatus?fstrON:fstrOFF);
		cmdOK();
	}
	else {
		if ( strnicmp(arg, fstrON,2)==0 ) {
			eeprom.SendStatus = true;
			eepromWriteVars();
			cmdOK();
		}
		else if ( strnicmp(arg, F("OF"),2)==0 ) {
			eeprom.SendStatus = false;
			eepromWriteVars();
			cmdOK();
		}
		else {
			cmdErrorParameter(F("'ON' or 'OFF'"));
		}
	}
}

void cmdLed()
{
/*
 * LED [period flash]
 *       Set/Get LED blink period
 *         period  blink period in ms
 *         flash   flash time in ms
 *                 if ommitted returns current values
 */
	WORD Interval;
	WORD Flash;
	char *argInterval;
	char *argFlash;

	argInterval = SCmd.next();
	argFlash = SCmd.next();
	if (argInterval == NULL) {
		sendStatus(true,SYSTEM,F("LED %d %d"), eeprom.BlinkInterval, eeprom.BlinkLen);
		cmdOK();
	}
	else if ( argInterval != NULL && argFlash != NULL ) {
		Interval=(WORD)atoi(argInterval);
		Flash=(WORD)atoi(argFlash);
		if ( Interval<Flash ) {
			cmdError(F("flash time must be >= interval time"));
		}
		else {
			eeprom.BlinkInterval = Interval;
			eeprom.BlinkLen = Flash;
			eepromWriteVars();
			cmdOK();
		}
	}
	else {
		cmdError(F("Need two arguments"));
	}
}

void cmdFactoryReset()
{
	unsigned long eepromCRC = CalcCRC(EEPROMCRC, (byte *)(EEPROM_ADDR_CRC32+4), EEPROM.length()-4) - 1;
	EEPROM.put(EEPROM_ADDR_CRC32, eepromCRC);
	eepromInitVars();
	cmdOK();
}

void cmdReset(void)
{
	cmdOK();

	sendStatus(true,SYSTEM,F("RESTART (in %d s)"), (WATCHDOG_TIME/1000));
	for(byte d=0; d<(FS20_SM8_IN_PROGRAMMODE/1000); d++) {
		Serial.print(F("."));
		delay(1000);
	}
	while(true) {};
}

void cmdEcho(void)
{
//~ "    ECHO [ON|OFF]\r\n"
//~ "        Set/Get echo on or off\r\n"
	char *arg;

	arg = SCmd.next();
	if (arg == NULL) {
		sendStatus(true,SYSTEM,F("ECHO %S"), eeprom.Echo?fstrON:fstrOFF);
		cmdOK();
	}
	else if ( strnicmp(arg, fstrON,2)==0 ) {
		eeprom.Echo = true;
		SCmd.setEcho(eeprom.Echo);
		eepromWriteVars();
		cmdOK();
	}
	else if ( strnicmp(arg, fstrOFF,3)==0 ) {
		eeprom.Echo = false;
		SCmd.setEcho(eeprom.Echo);
		eepromWriteVars();
		cmdOK();
	}
	else {
		cmdErrorParameter(F("'ON' or 'OFF'"));
	}
}

void cmdTerm(void)
{
//~ "    TERM [CR|LF]\r\n"
//~ "        Set/Get command terminator\r\n"
	char *arg;

	arg = SCmd.next();
	if (arg == NULL) {
		sendStatus(true,SYSTEM,F("TERM %S"),eeprom.Term=='\r'?fstrCR:fstrLF);
		cmdOK();
	}
	else if ( strnicmp(arg, fstrCR,2)==0 ) {
		eeprom.Term = '\r';
		SCmd.setTerm(eeprom.Term);
		eepromWriteVars();
		cmdOK();
	}
	else if ( strnicmp(arg, fstrLF,2)==0 ) {
		eeprom.Term = '\n';
		SCmd.setTerm(eeprom.Term);
		eepromWriteVars();
		cmdOK();
	}
	else {
		cmdErrorParameter(F("'CR' or 'LF'"));
	}
}



void cmdOK(void)
{
	SerialPrintfln(F("OK"));
}

void cmdError(String err)
{
	SerialPrintf(F("ERROR: "));
	Serial.println(err);
}

void cmdErrorParameter(String err)
{
	SerialPrintf(F("ERROR: Parameter error (use "));
	Serial.print(err);
	Serial.println(")");
}

// This gets set as the default handler, and gets called when no other command matches.
void unrecognized()
{
	SerialPrintfln(F("Unknown command, try HELP"));
}
