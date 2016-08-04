/* Kommandos der seriellen Schnittstelle
 * HELP
 * 	     List all commands
 * INFO
 *       Get info about system
 * UPTIME
 *       Tell how long the system has been running
 * MOTOR m [cmd]
 *       Set/Get motor control
 *         m    motor number 1..8
 *         cmd  OPEN, CLOSE, OFF, STATUS
 *              if ommitted returns current value
 * MOTORTIME x [sec]
 *       Set/Get motor runtime
 *         m    motor number 1..8
 *         sec  max run-time in sec
 *              if ommitted returns current value
 * MOTORTYPE x [cmd]
 *       Set/Get motor type control
 *         m    motor number 1..8
 *         cmd  WINDOW, JALOUSIE
 *              if ommitted returns current value
 * RAINSENSOR [cmd]
 *       Set/Get Rain sensor
 * 		   cmd
 *           AUTO     Enable rain hardware inputs
 *           ON       Raining
 *           OFF      No raining
 *           ENABLE   Enable rain detection (disables AUTO)
 *           DISABLE  disable rain detection (disables AUTO)
 *                    if cmd is ommitted, return current values
 * STATUS [ON|OFF]
 *       Set/Get Status sending
 *           ON       Send status messages
 *           OFF      Do not send status messages
 * LED [period flash]
 *       Set/Get LED blink interval/flash time
 *         interval blink interval in ms
 *         flash    flash time in ms
 *                  if ommitted returns current values
 */

SerialCommand SCmd;   		// SerialCommand object


void setupSerialCommand(void)
{
	// Setup callbacks for SerialCommand commands
	SCmd.addCommand("?",cmdHelp);
	SCmd.addCommand("HELP",cmdHelp);
	SCmd.addCommand("INFO",cmdInfo);
	SCmd.addCommand("UPTIME",cmdUptime);
	SCmd.addCommand("MOTOR",cmdMotor);
	SCmd.addCommand("MOTORTIME",cmdRuntime);
	SCmd.addCommand("MOTORTYPE",cmdType);
	SCmd.addCommand("RAINSENSOR",cmdRainSensor);
	SCmd.addCommand("RAIN",cmdRainSensor);
	SCmd.addCommand("STATUS",cmdStatus);
	SCmd.addCommand("LED",cmdLed);
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
"    motor [<m> [<cmd>]]\r\n"
"         Set/Get motor control\r\n"
"         <m>    Motor number [1..x]\r\n"
"         <cmd>  One of the following values:\r\n"
"                OPEN, CLOSE, OFF, STATUS\r\n"
"\r\n"
"    motortime [<m> [<sec>]\r\n"
"         Set/Get motor runtime\r\n"
"         <m>    Motor number [1..x]\r\n"
"         <sec>  Maximum runtime for this motor (in sec)\r\n"
"\r\n"
"    motortype [<m> [<cmd>]\r\n"
"         Set/Get motor type\r\n"
"         <m>    Motor number [1..x]\r\n"
"         <cmd>  One of the following values:\r\n"
"                WINDOW, JALOUSIE\r\n"
"         If a parameter is ommitted, command returns the current value\r\n"
"\r\n"
"    rain [<cmd>] or\r\n"
"    rainsensor [<cmd>]\r\n"
"         Set/Get Rain sensor function\r\n"
"         <cmd>  One of the following values:\r\n"
"                ENABLE   Enable rain detection (disables AUTO)\r\n"
"                DISABLE  disable rain detection (disables AUTO)\r\n"
"                AUTO     Rain detection from hardware-input\r\n"
"                ON       Raining\r\n"
"                OFF      No raining\r\n"
"\r\n"
"    status [on|off]\r\n"
"         Set/Get FS20Uno status message\r\n"
"         on      Status messages enabled\r\n"
"         off     Status messages disabled\r\n"
"\r\n"
"    led [<int> <flash>]\r\n"
"         Set/Get LED alive blinking parameter\r\n"
"         int     LED blinkinterval in ms\r\n"
"         flash   LED flash duration in ms\r\n"
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

void cmdMotor()
{
/*
 * MOTOR m [cmd]
 *       Motor control
 *         m    motor number 1..8
 *         cmd  OPEN, CLOSE, OFF, STATUS
 *              if ommitted returns current value
 */
	int motor;
	char *arg;

	arg = SCmd.next();
	if (arg == NULL) {
		for(motor=0; motor<MAX_MOTORS; motor++) {
			SerialPrintf(F("%-2d "), motor+1);
		}
		SerialPrintf(F("\r\n"));
		for(motor=0; motor<MAX_MOTORS; motor++) {
			SerialPrintf(getMotorDirection(motor)==MOTOR_OFF?F("--"):(getMotorDirection(motor)==MOTOR_OPEN)?F("OP"):F("CL"));
			SerialPrintf(F(" "));
		}
		SerialPrintf(F("\r\n"));
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
					setMotorDirection(motor, MOTOR_OPEN);
					cmdOK();
				}
				else if ( strnicmp(arg, "CL",2)==0 ) {
					setMotorDirection(motor, MOTOR_CLOSE);
					cmdOK();
				}
				else if ( strnicmp(arg, "OF",2)==0 ) {
					setMotorDirection(motor, MOTOR_OFF);
					cmdOK();
				}
				else {
					cmdError(F("Wrong parameter"));
				}
			}
			if (arg == NULL || strnicmp(arg, "STAT",4)==0 ) {
				if ( bitRead(valMotorRelais, motor)!=0 ) {
					if ( bitRead(valMotorRelais, motor+8)!=0 ) {
						SerialPrintf(F("OPENING"));
					}
					else {
						SerialPrintf(F("CLOSING"));
					}
				}
				else {
					SerialPrintf(F("OFF"));
				}
				SerialPrintf(F("\r\n"));
			}
		}
	}
}

void cmdRuntime()
{
/*
 * MOTORTIME x [sec]
 *       Set motor runtime
 *         m    motor number 1..8
 *         sec  max run-time in sec
 *              if ommitted returns current value
 */
	int motor;
	double runtime;
	char *arg;

	arg = SCmd.next();
	if (arg == NULL) {
		for(motor=0; motor<MAX_MOTORS; motor++) {
			SerialPrintf(F("%-7d"), motor+1);
		}
		SerialPrintf(F("\r\n"));
		for(motor=0; motor<MAX_MOTORS; motor++) {
			SerialPrintf(F("%-6.1f "), (double)eepromMaxRuntime[motor]/1000.0);
		}
		SerialPrintf(F("\r\n"));
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
				SerialPrintf(F("%.3f s"), (double)eepromMaxRuntime[motor]/1000.0);
			}
			else {
				// Set new runtime value
				runtime = atof(arg);
				if ( runtime<5.0 || runtime>300.0) {
					cmdError(F("Runtime out of range"));
				}
				else {
					eepromMaxRuntime[motor] = (DWORD)(runtime*1000.0);
					// Write new value into EEPROM
					eepromWriteLong(EEPROM_ADDR_MOTOR_MAXRUNTIME+(4*motor), eepromMaxRuntime[motor]);
					// Write new EEPROM checksum
					eepromWriteLong(EEPROM_ADDR_CRC32, eepromCalcCRC());
					SerialPrintf(F("%.3f s"), (double)eepromMaxRuntime[motor]/1000.0);
					cmdOK();
				}
			}
		}
	}
}

void cmdType()
{
/*
 * MOTORTYPE x [cmd]
 *       Motor type control
 *         m    motor number 1..8
 *         cmd  WINDOW, JALOUSIE
 *              if ommitted returns current value
 */
	int motor;
	char *arg;
	bool cmd = false;

	arg = SCmd.next();
	if (arg == NULL) {
		for(motor=0; motor<MAX_MOTORS; motor++) {
			SerialPrintf(F("%-2d "), motor+1);
		}
		SerialPrintf(F("\r\n"));
		for(motor=0; motor<MAX_MOTORS; motor++) {
			SerialPrintf(bitRead(eepromMTypeBitmask, motor)?F("WI"):F("JA"));
			SerialPrintf(F(" "));
		}
		SerialPrintf(F("\r\n"));
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
					cmdError(F("Wrong parameter"));
				}
			}
			if (arg == NULL || stricmp(arg, "STATUS")==0 ) {
				if ( bitRead(eepromMTypeBitmask, motor)!=0 ) {
					SerialPrintf(F("WINDOW"));
				}
				else {
					SerialPrintf(F("JALOUSIE"));
				}
				SerialPrintf(F("\r\n"));
			}
		}
	}
}

void cmdRainSensor()
{
/*
 * RAINSENSOR [cmd]
 *       Set/Get Rain sensor
 * 		   cmd
 *           AUTO     Enable rain hardware inputs
 *           ON       Raining
 *           OFF      No raining
 *           ENABLE   Enable rain detection (disables AUTO)
 *           DISABLE  disable rain detection (disables AUTO)
 *                    if cmd is ommitted, return current values
 */
	char *arg;
	bool cmd = false;

	arg = SCmd.next();
	if (arg == NULL) {
		debEnable.update();
		debInput.update();

		SerialPrintf(F("Rain sensor mode: "));
		SerialPrintf(bitRead(eepromRain, RAIN_BIT_AUTO)!=0?F("AUTO"):F("Software"));
		SerialPrintf(F("\r\n"));

		SerialPrintf(F("Sensor software mode:  "));
		SerialPrintf(bitRead(eepromRain, RAIN_BIT_ENABLE)!=0?F("Enabled"):F("Disabled"));
		SerialPrintf(F("\r\n"));
		SerialPrintf(F("Sensor software value: "));
		SerialPrintf(softRainInput?F("Raining"):F("Dry"));
		SerialPrintf(F("\r\n"));

		SerialPrintf(F("Sensor hardware mode:  "));
		SerialPrintf((debEnable.read() == RAIN_ENABLE_AKTIV)?F("Enabled"):F("Disabled"));
		SerialPrintf(F("\r\n"));
		SerialPrintf(F("Sensor hardware value: "));
		SerialPrintf((debInput.read()  == RAIN_INPUT_AKTIV)?F("Raining"):F("Dry"));
		SerialPrintf(F("\r\n"));

	}
	else {
		if ( strnicmp(arg, "ON",2)==0 ) {
			softRainInput = true;
			cmd = true;
		}
		else if ( strnicmp(arg, "OF",2)==0 ) {
			softRainInput = false;
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
			cmdError(F("Wrong parameter"));
		}
	}
}


void cmdStatus()
{
/*
 * STATUS [ON|OFF]
 *       Set/Get Status sending
 *           ON       Send status messages
 *           OFF      Do not send status messages
 */
	char *arg;
	bool cmd = false;

	arg = SCmd.next();
	if (arg == NULL) {
		SerialPrintf(eepromSendStatus?F("ON"):F("OFF"));
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
			eepromWriteByte(EEPROM_ADDR_SENDSTATUS, eepromRain);
			// Write new EEPROM checksum
			eepromWriteLong(EEPROM_ADDR_CRC32, eepromCalcCRC());
			cmdOK();
		}
		else {
			cmdError(F("Wrong parameter"));
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
		SerialPrintf(F("Interval %d\r\n"), eepromBlinkInterval);
		SerialPrintf(F("Flash %d\r\n"), eepromBlinkLen);
	}
	else if ( argInterval != NULL && argFlash != NULL ) {
		Interval=(WORD)atoi(argInterval);
		Flash=(WORD)atoi(argFlash);
		if ( Interval<Flash ) {
			cmdError(F("Wrong arguments, flash must be greater than interval"));
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
		cmdError(F("To less arguments"));
	}
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
	SerialPrintf(F("Command not found, try HELP"));
}
