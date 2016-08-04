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
	Serial.println(F(
"FS20Uno command help:\r\n"
"  ?\r\n"
"  HELP      List all commands\r\n"
"  INFO      Get info about system\r\n\r\n"
"  UPTIME    Tell how long the system has been running\r\n"
"  MOTOR m [cmd]\r\n"
"            Set/Get motor control\r\n"
"              m    motor number 1..8\r\n"
"              cmd  OPEN, CLOSE, OFF, STATUS\r\n"
"                   if ommitted returns current value\r\n"
"  MOTORTIME x [sec]\r\n"
"            Set/Get motor runtime\r\n"
"              m    motor number 1..8\r\n"
"              sec  max run-time in sec\r\n"
"                   if ommitted returns current value\r\n"
"  MOTORTYPE x [cmd]\r\n"
"            Set/Get motor type control\r\n"
"              m    motor number 1..8\r\n"
"              cmd  WINDOW, JALOUSIE\r\n"
"                   if ommitted returns current value\r\n"
"  RAIN [cmd]\r\n"
"  RAINSENSOR [cmd]\r\n"
"            Set/Get Rain sensor\r\n"
"              cmd\r\n"
"                AUTO     Enable rain hardware inputs\r\n"
"                ON       Raining\r\n"
"                OFF      No raining\r\n"
"                ENABLE   Enable rain detection (disables AUTO)\r\n"
"                DISABLE  disable rain detection (disables AUTO)\r\n"
"                If cmd is ommitted, return current values\r\n"
" STATUS [ON|OFF]\r\n"
"            Set/Get Status sending\r\n"
"                ON       Send status messages\r\n"
"                OFF      Do not send status messages\r\n"
"                If cmd is ommitted, return current values\r\n"
"  LED [period flash]\r\n"
"            Set/Get LED blink interval/flash time\r\n"
"              interval blink interval in ms\r\n"
"              flash    flash time in ms\r\n"
"              If ommitted returns current values\r\n"
));
}

void cmdInfo()
{
	printProgramInfo(false);
}

void cmdUptime()
{
	printUptime();
	Serial.println();
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
			mySerial.printf("%-2d ", motor+1);
		}
		Serial.println();
		for(motor=0; motor<MAX_MOTORS; motor++) {
			Serial.print(getMotorDirection(motor)==MOTOR_OFF?"--":(getMotorDirection(motor)==MOTOR_OPEN)?"OP":"CL");
			Serial.print(" ");
		}
		Serial.println();
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
						Serial.println(F("OPENING"));
					}
					else {
						Serial.println(F("CLOSING"));
					}
				}
				else {
					Serial.println(F("OFF"));
				}
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
			mySerial.printf("%-7d", motor+1);
		}
		Serial.println();
		for(motor=0; motor<MAX_MOTORS; motor++) {
			mySerial.printf("%-6.1f ", (double)eepromMaxRuntime[motor]/1000.0);
		}
		Serial.println();
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
				Serial.println(eepromMaxRuntime[motor]/1000.0, 3);
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
					Serial.println(eepromMaxRuntime[motor]/1000.0, 3);
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
			mySerial.printf("%-2d ", motor+1);
		}
		Serial.println();
		for(motor=0; motor<MAX_MOTORS; motor++) {
			Serial.print(bitRead(eepromMTypeBitmask, motor)?"WI":"JA");
			Serial.print(" ");
		}
		Serial.println();
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
					Serial.println(F("WINDOW"));
				}
				else {
					Serial.println(F("JALOUSIE"));
				}
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

		Serial.print(F("Rain sensor mode: "));
		Serial.println(bitRead(eepromRain, RAIN_BIT_AUTO)!=0?F("AUTO"):F("Software"));

		Serial.print(F("Sensor software mode:  "));
		Serial.println(bitRead(eepromRain, RAIN_BIT_ENABLE)!=0?F("Enabled"):F("Disabled"));
		Serial.print(F("Sensor software value: "));
		Serial.println(softRainInput?F("Raining"):F("Dry"));

		Serial.print(F("Sensor hardware mode:  "));
		Serial.println((debEnable.read() == RAIN_ENABLE_AKTIV)?F("Enabled"):F("Disabled"));
		Serial.print(F("Sensor hardware value: "));
		Serial.println((debInput.read()  == RAIN_INPUT_AKTIV)?F("Raining"):F("Dry"));

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
		Serial.println(eepromSendStatus?F("ON"):F("OFF"));
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
		Serial.print(F("Interval ")); Serial.println(eepromBlinkInterval);
		Serial.print(F("Flash ")); Serial.println(eepromBlinkLen);
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
	Serial.println(F("OK"));
}

void cmdError(String err)
{
	Serial.print(F("ERROR: "));
	Serial.println(err);
}

// This gets set as the default handler, and gets called when no other command matches.
void unrecognized()
{
	Serial.println(F("Command not found, try HELP"));
}
