/* ===================================================================
 * Command Interface Functions
 * ===================================================================*/
SerialCommand SCmd;   		// SerialCommand object

void setupSerialCommand(void)
{
	// Setup callbacks for SerialCommand commands
	SCmd.addCommand("?",cmdHelp);
	SCmd.addCommand("HELP",cmdHelp);
	SCmd.addCommand("INFO",cmdInfo);
	SCmd.addCommand("UPTIME",cmdUptime);
	SCmd.addCommand("FS20",cmdFS20);
	SCmd.addCommand("WALL",cmdWallButton);
	SCmd.addCommand("BUTTON",cmdWallButton);
	SCmd.addCommand("WALLBUTTON",cmdWallButton);
	SCmd.addCommand("MOTOR",cmdMotor);
	SCmd.addCommand("MOTORTIME",cmdRuntime);
	SCmd.addCommand("MOTORTYPE",cmdType);
	SCmd.addCommand("RAINSENSOR",cmdRainSensor);
	SCmd.addCommand("RAIN",cmdRainSensor);
	SCmd.addCommand("STATUS",cmdStatus);
	SCmd.addCommand("LED",cmdLed);
	SCmd.addCommand("FACTORYRESET",cmdFactoryReset);
	SCmd.addDefaultHandler(unrecognized);   // Handler for command that isn't matched  (says "What?")
}

void processSerialCommand(void)
{
	SCmd.readSerial();     // We don't do much, just process serial commands
}


void cmdHelp()
{	
	Serial.print(F(
"FS20Uno Command Help\r\n"
"----------------------------------------------------------------------\r\n"
"    ?\r\n"
"    help\r\n"
"        List all commands\r\n"
"\r\n"
"    info\r\n"
"        Get info about system\r\n"
"\r\n"
"    uptime\r\n"
"        Tell how long the system has been running\r\n"
"\r\n"
"    fs20 [<ch> [<cmd> [ON|OFF]]]\r\n"
"         Set/Get FS20 control\r\n"
"         <ch>     FS20 channel number [1..c]\r\n"
"         ON|OFF   set key <ch> to value\r\n"
"\r\n"
"    button | wall | wallbutton [<b> [ON|OFF]]\r\n"
"         Set/Get FS20 control\r\n"
"         <b>      Wall button number [1..m]\r\n"
"         ON|OFF   set new value\r\n"
"\r\n"
"    motor [<m> [<cmd>]]\r\n"
"         Set/Get motor control\r\n"
"         <m>      Motor number [1..m]\r\n"
"         <cmd>    can be OPEN, CLOSE, OFF, STATUS\r\n"
"\r\n"
"    motortime [<m> [<sec>]\r\n"
"         Set/Get motor runtime\r\n"
"         <m>      Motor number [1..m]\r\n"
"         <sec>    Maximum runtime for this motor (in sec)\r\n"
"\r\n"
"    motortype [<m> [<cmd>]\r\n"
"         Set/Get motor type\r\n"
"         <m>      Motor number [1..m]\r\n"
"         <cmd>    can be WINDOW or JALOUSIE\r\n"
"\r\n"
"    rain | rainsensor [<cmd>]\r\n"
"         Set/Get Rain sensor function\r\n"
"         <cmd>    can be\r\n"
"                  ENABLE   Enable rain detection (disables AUTO)\r\n"
"                  DISABLE  disable rain detection (disables AUTO)\r\n"
"                  AUTO     Rain detection from hardware-input\r\n"
"                  ON       Raining\r\n"
"                  OFF      No raining\r\n"
"\r\n"
"    status [on|off]\r\n"
"         Set/Get FS20Uno status message\r\n"
"         on        Status messages enabled\r\n"
"         off       Status messages disabled\r\n"
"\r\n"
"    led [<int> <flash>]\r\n"
"         Set/Get LED alive blinking parameter\r\n"
"         int       LED blinkinterval in ms\r\n"
"         flash     LED flash duration in ms\r\n"
"\r\n"
"    factoryreset\r\n"
"         Reset all values to factory defaults\r\n"
"\r\n"
"Set values: Use the command with parameters to set a value\r\n"
"Get values: Use the command without the given parameter to return the\r\n"
"            current value\r\n"
));
}

void cmdInfo()
{
	printProgramInfo(false);
}

void cmdUptime()
{
	SerialTimePrintf(F("\r\n"));
}

void cmdFS20()
{
//~ "    fs20 [<ch> [<cmd> [ON|OFF|PRG]]]\r\n"
//~ "         Set/Get FS20 control\r\n"
//~ "         <ch>         FS20 channel number [1..c]\r\n"
//~ "         ON|OFF|PRG   set FS20 channel <ch> mode\r\n"
// valSM8Button -> Tastensteuerung
// curSM8Status -> FS20 Ausgänge
	int channel;
	char *arg;

	arg = SCmd.next();
	if (arg == NULL) {
		for(channel=0; channel<IOBITS_CNT; channel++) {
			SerialPrintf(F("FS20 CH%02d "), channel+1);
			SerialPrintf(bitRead(curSM8Status, channel)?F("ON"):F("OFF"));
			SerialPrintf(F("\r\n"));
		}
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
				if      ( strnicmp(arg, "ON",2)==0 ) {
					if( !bitRead(curSM8Status, channel) ) {
						bitClear(valSM8Button, channel);
					}
					cmdOK();
				}
				else if ( strnicmp(arg, "OF",2)==0 ) {
					if( bitRead(curSM8Status, channel) ) {
						bitClear(valSM8Button, channel);
					}
					cmdOK();
				}
				else if ( strnicmp(arg, "PRG",3)==0 ) {
					bitSet(SM8StatusIgnore, channel);
					bitClear(valSM8Button, channel);
					expanderWriteWord(MPC_SM8BUTTON,   GPIO, valSM8Button);						
					SerialPrintf(F("Setting channel %d into program mode, please wait:  "), channel);
					for(byte d=0; d<(FS20_SM8_IN_PROGRAMMODE/1000); d++) {
						SerialPrintf(F("\b%1d"), (FS20_SM8_IN_PROGRAMMODE/1000)-d);
						delay(1000);
					}
					bitSet(SM8StatusIgnore, channel);
					bitClear(valSM8Button, channel);
					expanderWriteWord(MPC_SM8BUTTON,   GPIO, valSM8Button);						
					SerialPrintf(F("\r\nChannel %d now in programming mode\r\n"), channel);
				}
				else {
					cmdError(F("Wrong parameter (use 'on' or 'off')"));
				}
			}
			else {
				SerialPrintf(F("FS20-%02d "), channel+1);
				SerialPrintf(bitRead(curSM8Status, channel)?F("ON"):F("OFF"));
				SerialPrintf(F("\r\n"));
			}
		}
	}
}

void cmdWallButton()
{
//~ "    button | wall | wallbutton [<b> [ON|OFF]]\r\n"
//~ "         Set/Get FS20 control\r\n"
//~ "         <b>      Wall button number [1..m]\r\n"
//~ "         ON|OFF   set new value\r\n"
}

void cmdMotor()
{
//~ "    motor [<m> [<cmd>]]\r\n"
//~ "         Set/Get motor control\r\n"
//~ "         <m>      Motor number [1..m]\r\n"
//~ "         <cmd>    can be OPEN, CLOSE, OFF, STATUS\r\n"
	int motor;
	char *arg;

	arg = SCmd.next();
	if (arg == NULL) {
		for(motor=0; motor<MAX_MOTORS; motor++) {
			SerialPrintf(F("M%02d "), motor+1);
			SerialPrintf(getMotorDirection(motor)==MOTOR_OFF?F("OFF"):(getMotorDirection(motor)>=MOTOR_OPEN)?F("OPENING"):F("CLOSING"));
			SerialPrintf(F("\r\n"));
		}
	}
	else {
		motor=atoi(arg)-1;
		if ( motor<0 || motor>=MAX_MOTORS ) {
			cmdError(F("Motor number out of range"));
		}
		else {
			arg = SCmd.next();
			if (arg != NULL) {
				if      ( strnicmp(arg, "OP",2)==0 ) {
					if( getMotorDirection(motor)<MOTOR_OPEN ) {
						setMotorDirection(motor, MOTOR_OPEN);
					}
					cmdOK();
				}
				else if ( strnicmp(arg, "CL",2)==0 ) {
					if( getMotorDirection(motor)>MOTOR_CLOSE ) {
						setMotorDirection(motor, MOTOR_CLOSE);
					}
					cmdOK();
				}
				else if ( strnicmp(arg, "OF",2)==0 ) {
					if( getMotorDirection(motor)!=MOTOR_OFF ) {
						setMotorDirection(motor, MOTOR_OFF);
					}
					cmdOK();
				}
				else {
					cmdError(F("Wrong parameter (use 'open', 'close' or 'off')"));
				}
			}
			if (arg == NULL || strnicmp(arg, "STAT",4)==0 ) {
				SerialPrintf(F("M%02d "), motor+1);
				SerialPrintf(getMotorDirection(motor)==MOTOR_OFF?F("OFF"):(getMotorDirection(motor)>=MOTOR_OPEN)?F("OPENING"):F("CLOSING"));
				SerialPrintf(F("\r\n"));
			}
		}
	}
}

void cmdRuntime()
{
//~ "    motortime [<m> [<sec>]\r\n"
//~ "         Set/Get motor runtime\r\n"
//~ "         <m>      Motor number [1..m]\r\n"
//~ "         <sec>    Maximum runtime for this motor (in sec)\r\n"
	int motor;
	double runtime;
	char *arg;

	arg = SCmd.next();
	if (arg == NULL) {
		for(motor=0; motor<MAX_MOTORS; motor++) {
			SerialPrintf(F("M%02d TIME %4d.%-3d\r\n"), motor+1, eepromMaxRuntime[motor] / 1000, eepromMaxRuntime[motor] % 1000);
		}
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
				SerialPrintf(F("M%02d TIME %4d.%-3d\r\n"), motor+1, eepromMaxRuntime[motor] / 1000, eepromMaxRuntime[motor] % 1000);
			}
			else {
				// Set new runtime value
				runtime = atof(arg);
				if ( runtime<5.0 || runtime>300.0) {
					cmdError(F("Runtime out of range (min=5, max=300)"));
				}
				else {
					eepromMaxRuntime[motor] = (DWORD)(runtime*1000.0);
					// Write new value into EEPROM
					eepromWriteLong(EEPROM_ADDR_MOTOR_MAXRUNTIME+(4*motor), eepromMaxRuntime[motor]);
					// Write new EEPROM checksum
					eepromWriteLong(EEPROM_ADDR_CRC32, eepromCalcCRC());
					SerialPrintf(F("%d.%-d s"), eepromMaxRuntime[motor] / 1000, eepromMaxRuntime[motor] % 1000);
					cmdOK();
				}
			}
		}
	}
}

void cmdType()
{
//~ "    motortype [<m> [<cmd>]\r\n"
//~ "         Set/Get motor type\r\n"
//~ "         <m>      Motor number [1..m]\r\n"
//~ "         <cmd>    can be WINDOW or JALOUSIE\r\n"
	int motor;
	char *arg;
	bool cmd = false;

	arg = SCmd.next();
	if (arg == NULL) {
		for(motor=0; motor<MAX_MOTORS; motor++) {
			SerialPrintf(F("M%02d TYPE "), motor+1);
			SerialPrintf(bitRead(eepromMTypeBitmask, motor)?F("WINDOW"):F("JALOUSIE"));
			SerialPrintf(F("\r\n"));
		}
	}
	else {
		motor=atoi(arg)-1;
		if ( motor<0 || motor>=MAX_MOTORS ) {
			cmdError(F("Motor number out of range"));
		}
		else {
			arg = SCmd.next();
			if (arg != NULL) {
				if      ( strnicmp(arg, "WIN",3)==0 ) {
					bitSet(eepromMTypeBitmask, motor);
					cmd = true;
				}
				else if ( strnicmp(arg, "JAL",3)==0 ) {
					bitClear(eepromMTypeBitmask, motor);
					cmd = true;
				}
				if ( cmd ) {
					// Write new value into EEPROM
					eepromWriteLong(EEPROM_ADDR_MTYPE_BITMASK, (DWORD)eepromMTypeBitmask);

					// Write new EEPROM checksum
					eepromWriteLong(EEPROM_ADDR_CRC32, eepromCalcCRC());
					cmdOK();
				}
				else {
					cmdError(F("Wrong parameter (use 'window' or 'jalousie'"));
				}
			}
			if (arg == NULL || stricmp(arg, "STATUS")==0 ) {
				SerialPrintf(F("M%02d TYPE "), motor+1);
				SerialPrintf(bitRead(eepromMTypeBitmask, motor)?F("WINDOW"):F("JALOUSIE"));
				SerialPrintf(F("\r\n"));
			}
		}
	}
}

void cmdRainSensor()
{
//~ "    rain | rainsensor [<cmd>]\r\n"
//~ "         Set/Get Rain sensor function\r\n"
//~ "         <cmd>    can be\r\n"
//~ "                  ENABLE   Enable rain detection (disables AUTO)\r\n"
//~ "                  DISABLE  disable rain detection (disables AUTO)\r\n"
//~ "                  AUTO     Rain detection from hardware-input\r\n"
//~ "                  ON       Raining\r\n"
//~ "                  OFF      No raining\r\n"
	char *arg;
	bool cmd = false;

	arg = SCmd.next();
	if (arg == NULL) {
		debEnable.update();
		debInput.update();

		SerialPrintf(F("Rain sensor mode:         "));
		SerialPrintf(bitRead(eepromRain, RAIN_BIT_AUTO)!=0?F("AUTO"):F("Manual"));
		SerialPrintf(F("\r\n"));

		SerialPrintf(F("Rainsensor enable status: "));
		SerialPrintf(bitRead(eepromRain, RAIN_BIT_ENABLE)!=0?F("Enabled"):F("Disabled"));
		if( bitRead(eepromRain, RAIN_BIT_AUTO)!=0 ) {
			SerialPrintf(F(" - ignored"));
		}
		SerialPrintf(F("\r\n"));
		SerialPrintf(F("Rainsensor status:        "));
		SerialPrintf(softRainInput?F("Raining"):F("Dry"));
		if( bitRead(eepromRain, RAIN_BIT_AUTO)!=0 ) {
			SerialPrintf(F(" - ignored"));
		}
		SerialPrintf(F("\r\n"));

		SerialPrintf(F("Rainsensor enable input:  "));
		SerialPrintf((debEnable.read() == RAIN_ENABLE_AKTIV)?F("Enabled"):F("Disabled"));
		if( bitRead(eepromRain, RAIN_BIT_AUTO)==0 && 
		   (debEnable.read() == RAIN_ENABLE_AKTIV)!=bitRead(eepromRain, RAIN_BIT_ENABLE) ) {
			SerialPrintf(F(" - ignored"));
		}
		SerialPrintf(F("\r\n"));
		SerialPrintf(F("Rainsensor input:         "));
		SerialPrintf((debInput.read()  == RAIN_INPUT_AKTIV)?F("Raining"):F("Dry"));
		if( bitRead(eepromRain, RAIN_BIT_AUTO)==0 ) {
			SerialPrintf(F(" - ignored"));
		}
		SerialPrintf(F("\r\n"));
	}
	else {
		if ( strnicmp(arg, "ON",2)==0 ) {
			softRainInput = true;
			bitClear(eepromRain, RAIN_BIT_AUTO);
			cmd = true;
		}
		else if ( strnicmp(arg, "OF",2)==0 ) {
			softRainInput = false;
			bitClear(eepromRain, RAIN_BIT_AUTO);
			cmd = true;
		}
		else if ( strnicmp(arg, "EN",2)==0 ) {
			bitSet(eepromRain, RAIN_BIT_ENABLE);
			bitClear(eepromRain, RAIN_BIT_AUTO);
			cmd = true;
		}
		else if ( strnicmp(arg, "DI",2)==0 ) {
			bitClear(eepromRain, RAIN_BIT_ENABLE);
			bitClear(eepromRain, RAIN_BIT_AUTO);
			cmd = true;
		}
		else if ( strnicmp(arg, "AU",2)==0 ) {
			bitSet(eepromRain, RAIN_BIT_AUTO);
			cmd = true;
		}
		if ( cmd ) {
			// Write new value into EEPROM
			eepromWriteByte(EEPROM_ADDR_RAIN, eepromRain);
			// Write new EEPROM checksum
			eepromWriteLong(EEPROM_ADDR_CRC32, eepromCalcCRC());
			cmdOK();
		}
		else {
			cmdError(F("Wrong parameter (use 'enable', 'disable', 'auto', 'on' or 'off'"));
		}
	}
}


void cmdStatus()
{
//~ "    status [on|off]\r\n"
//~ "         Set/Get FS20Uno status message\r\n"
//~ "         on        Status messages enabled\r\n"
//~ "         off       Status messages disabled\r\n"
	char *arg;
	bool cmd = false;

	arg = SCmd.next();
	if (arg == NULL) {
		SerialPrintf(eepromSendStatus?F("ON\r\n"):F("OFF\r\n"));
	}
	else {
		if ( strnicmp(arg, "ON",2)==0 ) {
			eepromSendStatus = true;
			cmd = true;
		}
		else if ( strnicmp(arg, "OF",2)==0 ) {
			eepromSendStatus = false;
			cmd = true;
		}
		if ( cmd ) {
			// Write new value into EEPROM
			eepromWriteByte(EEPROM_ADDR_SENDSTATUS, eepromSendStatus);
			// Write new EEPROM checksum
			eepromWriteLong(EEPROM_ADDR_CRC32, eepromCalcCRC());
			cmdOK();
		}
		else {
			cmdError(F("Wrong parameter (use 'on' or 'off'"));
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
		SerialPrintf(F("Interval %d ms\r\n"), eepromBlinkInterval);
		SerialPrintf(F("Flash %d ms\r\n"), eepromBlinkLen);
	}
	else if ( argInterval != NULL && argFlash != NULL ) {
		Interval=(WORD)atoi(argInterval);
		Flash=(WORD)atoi(argFlash);
		if ( Interval<=Flash ) {
			cmdError(F("Wrong arguments, flash time must be greater than interval time"));
		}
		else {
			eepromBlinkInterval = Interval;
			eepromBlinkLen = Flash;

			// Write new value into EEPROM
			eepromWriteLong(EEPROM_ADDR_LED_BLINK_INTERVAL, (DWORD)eepromBlinkInterval);
			eepromWriteLong(EEPROM_ADDR_LED_BLINK_LEN, (DWORD)eepromBlinkLen);

			// Write new EEPROM checksum
			eepromWriteLong(EEPROM_ADDR_CRC32, eepromCalcCRC());
			cmdOK();
		}
	}
	else {
		cmdError(F("Need two arguments"));
	}
}

void cmdFactoryReset()
{
	eepromWriteLong(EEPROM_ADDR_CRC32, eepromCalcCRC()-1);
	setupEEPROMVars();
	cmdOK();
}

void cmdOK(void)
{
	SerialPrintf(F("OK\r\n"));
}

void cmdError(String err)
{
	SerialPrintf(F("ERROR: "));
	Serial.println(err);
}

// This gets set as the default handler, and gets called when no other command matches.
void unrecognized()
{
	SerialPrintf(F("Unknown command, try HELP\r\n"));
}
