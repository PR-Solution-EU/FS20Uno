/* Kommandos der seriellen Schnittstelle
 * HELP
 * 	     List all commands
 * INFO
 *       Get info about system
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
 * RAINSENSOR [ENABLE|DISABLE|ON/OFF]
 *       Set/Get Rain sensor
 *         ON/OFF   Switch rain detection
 *         ENABLE/  Enable or
 *         DISABLE  disable rain detection
 *                  if ommitted returns current values
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
	SCmd.addCommand("MOTOR",cmdMotor);
	SCmd.addCommand("MOTORTIME",cmdRuntime);
	SCmd.addCommand("MOTORTYPE",cmdType);
	SCmd.addCommand("RAINSENSOR",cmdRainSensor);
	SCmd.addCommand("LED",cmdLed);
	SCmd.addDefaultHandler(unrecognized);   // Handler for command that isn't matched  (says "What?")
}

void processSerialCommand(void)
{
	SCmd.readSerial();     // We don't do much, just process serial commands
}


void cmdHelp()
{
	Serial.println(F("= Command Help ="));
	Serial.println();
	Serial.println(F("  ?"));
	Serial.println(F("  HELP"));
	Serial.println(F("  	     List all commands"));
	Serial.println(F("  INFO"));
	Serial.println(F("        Get info about system"));
	Serial.println(F("  MOTOR m [cmd]"));
	Serial.println(F("        Set/Get motor control"));
	Serial.println(F("          m    motor number 1..8"));
	Serial.println(F("          cmd  OPEN, CLOSE, OFF, STATUS"));
	Serial.println(F("               if ommitted returns current value"));
	Serial.println(F("  MOTORTIME x [sec]"));
	Serial.println(F("        Set/Get motor runtime"));
	Serial.println(F("          m    motor number 1..8"));
	Serial.println(F("          sec  max run-time in sec"));
	Serial.println(F("               if ommitted returns current value"));
	Serial.println(F("  MOTORTYPE x [cmd]"));
	Serial.println(F("        Set/Get motor type control"));
	Serial.println(F("          m    motor number 1..8"));
	Serial.println(F("          cmd  WINDOW, JALOUSIE"));
	Serial.println(F("               if ommitted returns current value"));
	Serial.println(F("  LED [period flash]"));
	Serial.println(F("        Set/Get LED blink interval/flash time"));
	Serial.println(F("          interval blink interval in ms"));
	Serial.println(F("          flash    flash time in ms"));
	Serial.println(F("                   if ommitted returns current values"));
	Serial.println();
}

void cmdInfo()
{
	Serial.println();
	printProgramInfo();
	Serial.print(F("Uptime: "));
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
		cmdError(F("To less arguments"));
	}
	else {
		motor=atoi(arg)-1;
		if( motor<0 || motor>=MAX_MOTORS ) {
			cmdError(F("Motor number out of range"));
		}
		else {
			arg = SCmd.next();
			if (arg != NULL) {
				if      ( stricmp(arg, "OPEN")==0 ) {
					setMotorDirection(motor, MOTOR_OPEN);
					cmdOK();
				}
				else if ( stricmp(arg, "CLOSE")==0 ) {
					setMotorDirection(motor, MOTOR_CLOSE);
					cmdOK();
				}
				else if ( stricmp(arg, "OFF")==0 ) {
					setMotorDirection(motor, MOTOR_OFF);
					cmdOK();
				}
			}
			if (arg == NULL || stricmp(arg, "STATUS")==0 ) {
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
		cmdError(F("To less arguments"));
	}
	else {
		motor=atoi(arg)-1;
		if( motor<0 || motor>=MAX_MOTORS ) {
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
				if( runtime<5.0 || runtime>300.0) {
					cmdError(F("Runtime value out of range"));
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
		cmdError(F("To less arguments"));
	}
	else {
		motor=atoi(arg)-1;
		if( motor<0 || motor>=MAX_MOTORS ) {
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
 * RAINSENSOR [ENABLE|DISABLE|ON/OFF]
 *       Set/Get Rain sensor
 *         ON/OFF   Switch rain detection
 *         ENABLE/  Enable or
 *         DISABLE  disable rain detection
 *                  if ommitted returns current values
 */
	char *arg;
	bool cmd = false;

	arg = SCmd.next();
	if (arg == NULL) {
	}
	else {
		if ( strnicmp(arg, "ON",2)==0 ) {
			cmd = true;
		}
		else if ( strnicmp(arg, "OF",3)==0 ) {
			cmd = true;
		}
		else if ( strnicmp(arg, "EN",2)==0 ) {
			cmd = true;
		}
		else if ( strnicmp(arg, "DI",2)==0 ) {
			cmd = true;
		}
		if( cmd ) {
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
		if( Interval<Flash ) {
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
	Serial.println(F("What?"));
}
