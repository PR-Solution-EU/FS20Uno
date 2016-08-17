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
	SCmd.addCommand("ECHO",cmdEcho);
	SCmd.addCommand("TERM",cmdTerm);
	SCmd.addCommand("UPTIME",cmdUptime);
	SCmd.addCommand("FS20",cmdFS20);
	SCmd.addCommand("BUTTON",cmdWallButton);
	SCmd.addCommand("PUSHBUTTON",cmdWallButton);
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
	SCmd.setEcho(eepromCmdEcho);
	SCmd.setTerm(eepromCmdTerm);
}

void processSerialCommand(void)
{
	SCmd.readSerial();     // We don't do much, just process serial commands
}


void cmdHelp()
{
	char *arg;

	arg = SCmd.next();
	if (arg == NULL) {
		Serial.print(F(
			"Command List\r\n"
			"----------------------------------------------------------------------\r\n"
			"ECHO          Set/Get echo on or off\r\n"
			"FACTORYRESET  Set system values to factory defaults\r\n"
			"FS20          Set/Get FS20 control\r\n"
			"HELP          Get command help\r\n"
			"INFO          Get program version\r\n"
			"LED           Set/Get LED alive blinking parameter\r\n"
			"MOTOR         Set/Get motor control\r\n"
			"MOTORTIME     Set/Get motor runtime\r\n"
			"MOTORTYPE     Set/Get motor type\r\n"
			"RAINSENSOR    Set/Get Rain sensor function\r\n"
			"PUSHBUTTON    Set/Get wall pushbutton status\r\n"
			"STATUS        Set/Get FS20Uno status message\r\n"
			"TERM          Set/Get command terminator\r\n"
			"UPTIME        Get system uptime\r\n"
			"\r\n"
			"To get current values use the command without any parameter.\r\n"
			"To set values use command with parameters.\r\n"
			"For command parameter details use 'HELP command', e.g. HELP INFO\r\n"
			));
	}
	else if ( strnicmp(arg, F("EC"),2)==0 ) {
		Serial.print(F(
			"ECHO [ON|OFF]\r\n"
			"  Set/Get command interface echo on or off\r\n"
			));
	}
	else if ( strnicmp(arg, F("FA"),2)==0 ) {
		Serial.print(F(
			"FACTORYRESET\r\n"
			"  Reset all values to factory defaults\r\n"
			));
	}
	else if ( strnicmp(arg, F("FS"),2)==0 ) {
		Serial.print(F(
			"FS20 [<ch> [ON|OFF|PRG]]\r\n"
			"  Set/Get FS20 receiver channel\r\n"
			"     <ch>     FS20 channel number [1..c]\r\n"
			"     ON       switch channel <ch> ON\r\n"
			"     OFF      switch channel <ch> OFF\r\n"
			"     PRG      switch channel <ch> into programming mode\r\n"
			));
	}
	else if ( strnicmp(arg, F("HE"),2)==0 ) {
		Serial.print(F(
			"HELP, ?\r\n"
			"  List all commands\r\n"
			));
	}
	else if ( strnicmp(arg, F("IN"),2)==0 ) {
		Serial.print(F(
			"INFO\r\n"
			"  Get info about system\r\n"
			));
	}
	else if ( strnicmp(arg, F("LE"),2)==0 ) {
		Serial.print(F(
			"LED [<int> <flash>]\r\n"
			"  Set/Get LED alive blinking parameter\r\n"
			"     int       LED blinkinterval in ms\r\n"
			"     flash     LED flash duration in ms\r\n"
			));
	}
	else if ( strnicmp(arg, F("MOTOR"),5)==0 ) {
		Serial.print(F(
			"MOTOR [<m> [<cmd>]]\r\n"
			"  Set/Get motor status\r\n"
			"     <m>      Motor number [1..m]\r\n"
			"     <cmd>    can be\r\n"
			"              OPEN   - motor in OPEN direction\r\n"
			"              CLOSE  - motor in CLOSE direction\r\n"
			"              TOPEN  - toogle OPEN direction\r\n"
			"              TCLOSE - toogle CLOSE direction\r\n"
			"              TOOGLE - toogle direction\r\n"
			"              OFF    - stop motor\r\n"
			"              STATUS - return the current status\r\n"
			));
	}
	else if ( strnicmp(arg, F("MOTORTI"),7)==0 ) {
		Serial.print(F(
			"MOTORTIME [<m> [<sec>]\r\n"
			"  Set/Get motor maximum runtime\r\n"
			"     <m>      Motor number [1..m]\r\n"
			"     <sec>    Maximum runtime for this motor (in sec)\r\n"
			));
	}
	else if ( strnicmp(arg, F("MOTORTY"),7)==0 ) {
		Serial.print(F(
			"MOTORTYPE [<m> [<cmd>]\r\n"
			"  Set/Get motor type\r\n"
			"     <m>      Motor number [1..m]\r\n"
			"     <cmd>    can be WINDOW or JALOUSIE\r\n"
			));
	}
	else if ( strnicmp(arg, F("RA"),2)==0 ) {
		Serial.print(F(
			"RAIN, RAINSENSOR [<cmd>]\r\n"
			"  Set/Get rain sensor\r\n"
			"     <cmd>    can be\r\n"
			"       AUTO     Rain detection and detection enabled from input signals\r\n"
			"       ENABLE   Enable rain detection, disables AUTO\r\n"
			"       DISABLE  disable rain detection, disables AUTO\r\n"
			"       ON       Raining, disables AUTO\r\n"
			"       OFF      No raining, disables AUTO\r\n"
			));
	}
	else if ( strnicmp(arg, F("PU"),2)==0 ) {
		Serial.print(F(
			"BUTTON, PUSHBUTTON, WALLBUTTON [<b> [ON|OFF]]\r\n"
			"  Set/Get wall pushbutton status\r\n"
			"     <b>      Wall button number [1..m]\r\n"
			"     ON|OFF   set new value\r\n"
			));
	}
	else if ( strnicmp(arg, F("ST"),2)==0 ) {
		Serial.print(F(
			"STATUS [ON|OFF]\r\n"
			"  Set/Get system status message\r\n"
			"     ON        Status messages enabled\r\n"
			"               Automatically send status messages when system status changes\r\n"
			"     OFF       Status messages disabled\r\n"
			"               No messages are send on system status changes\r\n"
			));
	}
	else if ( strnicmp(arg, F("TE"),2)==0 ) {
		Serial.print(F(
			"TERM [CR|LF]\r\n"
			"  Set/Get command terminator\r\n"
			));
	}
	else if ( strnicmp(arg, F("UP"),2)==0 ) {
		Serial.print(F(
			"UPTIME\r\n"
			"  Tell how long the system has been running\r\n"
			));
	}
	
	Serial.print(F("\r\n"));
}

void cmdEcho(void)
{
//~ "    ECHO [ON|OFF]\r\n"
//~ "        Set/Get echo on or off\r\n"
	char *arg;

	arg = SCmd.next();
	if (arg == NULL) {
		SerialPrintf(F("ECHO "));
		SerialPrintf(eepromCmdEcho?F("ON"):F("OFF"));
		SerialPrintf(F("\r\n"));
	}
	else if ( strnicmp(arg, F("ON"),2)==0 ) {
		eepromCmdEcho = true;
		SCmd.setEcho(eepromCmdEcho);
		eepromWriteVars(EEPROM_ECHO);
		cmdOK();
	}
	else if ( strnicmp(arg, F("OF"),2)==0 ) {
		eepromCmdEcho = false;
		SCmd.setEcho(eepromCmdEcho);
		eepromWriteVars(EEPROM_ECHO);
		cmdOK();
	}
	else {
		cmdError(F("Unknown parameter (use 'ON' or 'OFF')"));
	}
}

void cmdTerm(void)
{
//~ "    TERM [CR|LF]\r\n"
//~ "        Set/Get command terminator\r\n"
	char *arg;

	arg = SCmd.next();
	if (arg == NULL) {
		SerialPrintf(F("TERM "));
		SerialPrintf(eepromCmdTerm=='\r'?F("CR"):F("LF"));
		SerialPrintf(F("\r\n"));
	}
	else if ( strnicmp(arg, F("CR"),2)==0 ) {
		eepromCmdTerm = '\r';
		SCmd.setTerm(eepromCmdTerm);
		eepromWriteVars(EEPROM_TERM);
		cmdOK();
	}
	else if ( strnicmp(arg, F("OF"),2)==0 ) {
		eepromCmdTerm = '\n';
		SCmd.setTerm(eepromCmdTerm);
		eepromWriteVars(EEPROM_TERM);
		cmdOK();
	}
	else {
		cmdError(F("Unknown parameter (use 'CR' or 'LF')"));
	}
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
				if      ( strnicmp(arg, F("ON"),2)==0 ) {
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
						#ifdef WATCHDOG_ENABLED
						Watchdog.reset();
						#endif
						delay(1000);
						#ifdef WATCHDOG_ENABLED
						Watchdog.reset();
						#endif
					}
					bitSet(SM8StatusIgnore, channel);
					bitClear(valSM8Button, channel);
					expanderWriteWord(MPC_SM8BUTTON,   GPIO, valSM8Button);
					SerialPrintf(F("\r\nChannel %d now in programming mode\r\n"), channel+1);
				}
				else {
					cmdError(F("Unknown parameter (use 'ON', 'OFF' or 'PRG')"));
				}
			}
			else {
				SerialPrintf(F("FS20 CH%02d "), channel+1);
				SerialPrintf(bitRead(curSM8Status, channel)?F("ON"):F("OFF"));
				SerialPrintf(F("\r\n"));
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
			SerialPrintf(F("PB%02d "), button+1);
			SerialPrintf(bitRead(curWallButton, button)?F("ON"):F("OFF"));
			SerialPrintf(F("\r\n"));
		}
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
				if      ( strnicmp(arg, F("ON"),2)==0 ) {
					bitSet(curWallButton, button);
					cmdOK();
				}
				else if ( strnicmp(arg, F("OF"),2)==0 ) {
					bitClear(curWallButton, button);
					cmdOK();
				}
				else {
					cmdError(F("Unknown parameter (use 'ON' or 'OFF')"));
				}
			}
			else {
				SerialPrintf(F("PB%02d "), button+1);
				SerialPrintf(bitRead(curWallButton, button)?F("ON"):F("OFF"));
				SerialPrintf(F("\r\n"));
			}
		}
	}
}

void cmdMotor()
{
//~ "MOTOR [<m> [<cmd>]]\r\n"
//~ "  Set/Get motor status\r\n"
//~ "     <m>      Motor number [1..m]\r\n"
//~ "     <cmd>    can be\r\n"
//~ "              OPEN   - motor in OPEN direction\r\n"
//~ "              CLOSE  - motor in CLOSE direction\r\n"
//~ "              TOPEN  - toogle OPEN direction\r\n"
//~ "              TCLOSE - toogle CLOSE direction\r\n"
//~ "              TOOGLE - toogle direction\r\n"
//~ "              OFF    - stop motor\r\n"
//~ "              STATUS - return the current status\r\n"
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
				else {
					cmdError(F("Unknown 2. parameter (use 'HELP MOTOR' for more info)"));
				}
			}
			if (arg == NULL || strnicmp(arg, F("STAT"),4)==0 ) {
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
					eepromWriteVars(EEPROM_MOTOR_MAXRUNTIME);
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
				if      ( strnicmp(arg, F("WIN"),3)==0 ) {
					bitSet(eepromMTypeBitmask, motor);
					eepromWriteVars(EEPROM_MTYPE_BITMASK);
					cmdOK();
				}
				else if ( strnicmp(arg, F("JAL"),3)==0 ) {
					bitClear(eepromMTypeBitmask, motor);
					eepromWriteVars(EEPROM_MTYPE_BITMASK);
					cmdOK();
				}
				else {
					cmdError(F("Unknown parameter (use 'WINDOW' or 'JALOUSIE'"));
				}
			}
			if (arg == NULL || strnicmp(arg, F("STAT"),4)==0 ) {
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
		bool sensorEnabled;

		/*
		Rainsensor input:   Dry|Raining
		Rainsensor setting: Dry|Raining
		Sensor monitoring:  Enabled|Disabled readin from input|setting
		Rainsensor status:  Dry|Raining
		*/

		SerialPrintf(F("Sensor monitoring:  "));
		if( bitRead(eepromRain, RAIN_BIT_AUTO) ) {
			sensorEnabled = (debEnable.read() == RAIN_ENABLE_AKTIV);
		}
		else {
			sensorEnabled = bitRead(eepromRain, RAIN_BIT_ENABLE);
		}
		SerialPrintf(sensorEnabled?F("Enabled"):F("Disabled"));
		SerialPrintf(F(" reading from "));
		SerialPrintf(bitRead(eepromRain, RAIN_BIT_AUTO)?F("input"):F("setting"));
		SerialPrintf(F("\r\n"));

		SerialPrintf(F("Rainsensor input:   "));
		SerialPrintf((debInput.read()==RAIN_INPUT_AKTIV)?F("Raining"):F("Dry"));
		SerialPrintf(F("\r\n"));
		
		SerialPrintf(F("Rainsensor setting: "));
		SerialPrintf(softRainInput?F("Raining"):F("Dry"));
		SerialPrintf(F("\r\n"));

		SerialPrintf(F("Rainsensor status:  "));
		SerialPrintf( sensorEnabled && ((debInput.read()==RAIN_INPUT_AKTIV) || softRainInput) ? F("Raining") : F("Dry") );
		SerialPrintf(F("\r\n"));
	}
	else {
		if ( strnicmp(arg, F("ON"),2)==0 ) {
			softRainInput = true;
			bitClear(eepromRain, RAIN_BIT_AUTO);
			cmd = true;
		}
		else if ( strnicmp(arg, F("OF"),2)==0 ) {
			softRainInput = false;
			bitClear(eepromRain, RAIN_BIT_AUTO);
			cmd = true;
		}
		else if ( strnicmp(arg, F("EN"),2)==0 ) {
			bitSet(eepromRain, RAIN_BIT_ENABLE);
			bitClear(eepromRain, RAIN_BIT_AUTO);
			cmd = true;
		}
		else if ( strnicmp(arg, F("DI"),2)==0 ) {
			bitClear(eepromRain, RAIN_BIT_ENABLE);
			bitClear(eepromRain, RAIN_BIT_AUTO);
			cmd = true;
		}
		else if ( strnicmp(arg, F("AU"),2)==0 ) {
			bitSet(eepromRain, RAIN_BIT_AUTO);
			cmd = true;
		}
		if ( cmd ) {
			eepromWriteVars(EEPROM_RAIN);
			cmdOK();
		}
		else {
			cmdError(F("Unknown parameter (use 'ENABLE', 'DISABLE', 'AUTO', 'ON' or 'OFF'"));
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

	arg = SCmd.next();
	if (arg == NULL) {
		SerialPrintf(eepromCmdSendStatus?F("ON\r\n"):F("OFF\r\n"));
	}
	else {
		if ( strnicmp(arg, F("ON"),2)==0 ) {
			eepromCmdSendStatus = true;
			eepromWriteVars(EEPROM_SENDSTATUS);
			cmdOK();
		}
		else if ( strnicmp(arg, F("OF"),2)==0 ) {
			eepromCmdSendStatus = false;
			eepromWriteVars(EEPROM_SENDSTATUS);
			cmdOK();
		}
		else {
			cmdError(F("Unknown parameter (use 'ON' or 'OFF'"));
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

			eepromWriteVars(EEPROM_LED_BLINK_INTERVAL | EEPROM_LED_BLINK_LEN);
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
	eepromInitVars();
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
