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
	SCmd.addCommand(PSTR("ECHO"),Echo);
	SCmd.addCommand(PSTR("TERM"),Term);
	SCmd.addCommand(PSTR("UPTIME"),cmdUptime);
	SCmd.addCommand(PSTR("FS20"),cmdFS20);
	SCmd.addCommand(PSTR("BUTTON"),cmdWallButton);
	SCmd.addCommand(PSTR("PUSHBUTTON"),cmdWallButton);
	SCmd.addCommand(PSTR("WALLBUTTON"),cmdWallButton);
	SCmd.addCommand(PSTR("MOTOR"),cmdMotor);
	SCmd.addCommand(PSTR("MOTORNAME"),cmdName);
	SCmd.addCommand(PSTR("MOTORTIME"),cmdRuntime);
	SCmd.addCommand(PSTR("MOTORTYPE"),cmdType);
	SCmd.addCommand(PSTR("RAINSENSOR"),cmdRainSensor);
	SCmd.addCommand(PSTR("RAIN"),cmdRainSensor);
	SCmd.addCommand(PSTR("STATUS"),cmdStatus);
	SCmd.addCommand(PSTR("LED"),cmdLed);
	SCmd.addCommand(PSTR("FACTORYRESET"),cmdFactoryReset);
	SCmd.addDefaultHandler(unrecognized);   // Handler for command that isn't matched  (says "What?")
	SCmd.setEcho(eeprom.Echo);
	SCmd.setTerm(eeprom.Term);
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
			"MOTORNAME     Set/Get motor names\r\n"
			"MOTORTIME     Set/Get motor runtime\r\n"
			"MOTORTYPE     Set/Get motor type\r\n"
			"RAINSENSOR    Set/Get Rain sensor function\r\n"
			"PUSHBUTTON    Set/Get wall pushbutton status\r\n"
			"STATUS        Set/Get autosend status messages\r\n"
			"TERM          Set/Get command terminator\r\n"
			"UPTIME        Get system uptime\r\n"
			"\r\n"
			"To get current values use the command without any parameter.\r\n"
			"To set values use command with parameters.\r\n"
			"\r\n"
			"For command parameter details use 'HELP <command>', e.g. HELP INFO\r\n"
			"\r\n"
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
			"MOTOR [<m> [<cmd> [<param>]]\r\n"
			"  Set/Get motor status\r\n"
			"     <m>      Motor number [1..m]\r\n"
			"     <cmd>    can be\r\n"
			"         OPEN      Motor in OPEN direction\r\n"
			"         CLOSE     Motor in CLOSE direction\r\n"
			"         TOPEN     Toogle OPEN direction\r\n"
			"         TCLOSE    Toogle CLOSE direction\r\n"
			"         TOOGLE    Toogle direction\r\n"
			"         GOTO <p>  Goto position <p> (in %, 0-100)\r\n"
			"         <p>       Goto position <p> (in %, 0-100)\r\n"
			"         OFF       Stop motor\r\n"
			"         SYNC      Set motor in a default defined state\r\n"
			"         STATUS    Return the current status\r\n"
			));
	}
	else if ( strnicmp(arg, F("MOTORNA"),7)==0 ) {
		Serial.print(F(
			"MOTORNAME [<m> [<name>]\r\n"
			"  Set/Get motor name\r\n"
			"     <m>      Motor number [1..m]\r\n"
			"     <name    Motor name\r\n"
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
			"       AUTO        Rain detection and detection enabled from input signals\r\n"
			"       ENABLE      Enable rain detection, disables AUTO\r\n"
			"       DISABLE     Disable rain detection, disables AUTO\r\n"
			"       ON          Raining, disables AUTO\r\n"
			"       OFF         No raining, disables AUTO\r\n"
			"       RESUME <s>  Resume window position after rain was gone\r\n"
			"                   <s> is the delay in sec after rain was gone before resume starr\r\n"
			"                   and before resume starts.\r\n"
			"       FORGET      Do not remember window position, keep it close\r\n"
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
			"  Set/Get autosend system status message\r\n"
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
			"UPTIME [h]\r\n"
			"  Tell how long the system has been running\r\n"
			"  If optional argument is given, operation hours will be set\r\n"
			));
	}
	watchdogReset();
	
	Serial.print(F("\r\n"));
}

void Echo(void)
{
//~ "    ECHO [ON|OFF]\r\n"
//~ "        Set/Get echo on or off\r\n"
	char *arg;

	arg = SCmd.next();
	if (arg == NULL) {
		SerialPrintf(F("ECHO %S\r\n"),eeprom.Echo?fstrON:fstrOFF);
		cmdOK();
	}
	else if ( strnicmp(arg, fstrON,2)==0 ) {
		eeprom.Echo = true;
		SCmd.setEcho(eeprom.Echo);
		eepromWriteVars();
		cmdOK();
	}
	else if ( strnicmp(arg, F("OF"),2)==0 ) {
		eeprom.Echo = false;
		SCmd.setEcho(eeprom.Echo);
		eepromWriteVars();
		cmdOK();
	}
	else {
		cmdErrorParameter(F("'ON' or 'OFF'"));
	}
}

void Term(void)
{
//~ "    TERM [CR|LF]\r\n"
//~ "        Set/Get command terminator\r\n"
	char *arg;

	arg = SCmd.next();
	if (arg == NULL) {
		SerialPrintf(F("TERM %S\r\n"),eeprom.Term=='\r'?F("CR"):F("LF"));
		cmdOK();
	}
	else if ( strnicmp(arg, F("CR"),2)==0 ) {
		eeprom.Term = '\r';
		SCmd.setTerm(eeprom.Term);
		eepromWriteVars();
		cmdOK();
	}
	else if ( strnicmp(arg, F("OF"),2)==0 ) {
		eeprom.Term = '\n';
		SCmd.setTerm(eeprom.Term);
		eepromWriteVars();
		cmdOK();
	}
	else {
		cmdErrorParameter(F("'CR' or 'LF'"));
	}
}

void cmdInfo()
{
	printProgramInfo(false);
	cmdUptime();
}

void cmdUptime()
{
	char *arg;
	
	arg = SCmd.next();
	if (arg != NULL) {
		eeprom.OperatingHours=atol(arg);
		eepromWriteVars();
	}
	SerialPrintf(F("Uptime:    "));
	SerialTimePrintf(F("\r\n"));
	SerialPrintf(F("Operating: %ld h\r\n"), eeprom.OperatingHours);
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
			SerialPrintf(F("FS20 CH%02d %S\r\n"), channel+1, bitRead(curSM8Status, channel)?fstrON:fstrOFF);
			watchdogReset();
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
						watchdogReset();
						delay(1000);
						watchdogReset();
					}
					bitSet(SM8StatusIgnore, channel);
					bitClear(valSM8Button, channel);
					expanderWriteWord(MPC_SM8BUTTON,   GPIO, valSM8Button);
					SerialPrintf(F("\r\nChannel %d now in programming mode\r\n"), channel+1);
					cmdOK();
				}
				else {
					cmdErrorParameter(F("'ON', 'OFF' or 'PRG'"));
				}
			}
			else {
				SerialPrintf(F("FS20 CH%02d %S\r\n"), channel+1,bitRead(curSM8Status, channel)?fstrON:fstrOFF);
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
			SerialPrintf(F("PB%02d %S\r\n"), button+1, bitRead(curWallButton, button)?fstrON:fstrOFF);
			watchdogReset();
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
				SerialPrintf(F("PB%02d %S\r\n"), button+1, bitRead(curWallButton, button)?fstrON:fstrOFF);
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
			cmdMotorPrintStatus(motor);
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
						setMotorPosition(motor, (MOTOR_TIMER)((long)(eeprom.MaxRuntime[motor] / TIMER_MS) * (long)percent / 100L));
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
					if ( getMotorType(motor)==WINDOW ) {
						setMotorDirection(motor, MOTOR_CLOSE);
					}
					else {
						setMotorDirection(motor, MOTOR_OPEN);
					}
					cmdOK();
				}
				else if ( (atoi(arg)>=0 && atoi(arg)<=100) ) {
					setMotorPosition(motor, (MOTOR_TIMER)((long)(eeprom.MaxRuntime[motor] / TIMER_MS) * (long)atoi(arg) / 100L));
					cmdOK();
				}
				else {
					cmdError(F("Unknown 2. parameter (use 'HELP MOTOR' for more info)"));
				}
			}
			if (arg == NULL || strnicmp(arg, F("ST"),2)==0 ) {
				cmdMotorPrintStatus(motor);
				cmdOK();
			}
		}
	}
}
void cmdMotorPrintStatus(int motor)
{
	byte runTimePercent = (byte)((long)MotorPosition[motor]*100L / (long)(eeprom.MaxRuntime[motor] / TIMER_MS));
	
	if( runTimePercent<1 && MotorPosition[motor]>0 ) {
		runTimePercent=1;
	}
	if( runTimePercent>100 ) {
		runTimePercent=100;
	}
	SerialPrintf(F("M%02d %-7S %3d%% %-7S (%s)\r\n")
				,motor+1
				,runTimePercent==0?F("CLOSE"):(runTimePercent==100?F("OPEN"):F("BETWEEN"))
				,runTimePercent
				,getMotorDirection(motor)==MOTOR_OFF?F("OFF"):(getMotorDirection(motor)>=MOTOR_OPEN)?F("OPENING"):F("CLOSING")
				,(char *)eeprom.MotorName[motor]);
	watchdogReset();
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
			cmdRuntimePrintStatus(motor);
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
				cmdRuntimePrintStatus(motor);
				cmdOK();
			}
			else {
				// Set new runtime value
				runtime = atof(arg);
				if ( runtime<1.0 || runtime>(double)(MOTOR_MAXRUNTIME/TIMER_MS) ) {
					cmdError(F("Motor runtime out of range"));
				}
				else {
					eeprom.MaxRuntime[motor] = (DWORD)(runtime*1000.0);
					eepromWriteVars();
					cmdRuntimePrintStatus(motor);
					cmdOK();
				}
			}
		}
	}
}
void cmdRuntimePrintStatus(int motor)
{
	SerialPrintf(F("M%02d %d.%03d (%s)\r\n")
				,(int)(motor+1)
				,(int)(eeprom.MaxRuntime[motor] / 1000)
				,(int)(eeprom.MaxRuntime[motor] % 1000)
				,(char *)eeprom.MotorName[motor]);
	watchdogReset();
}


void cmdName()
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
			cmdNamePrintStatus(motor);
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
				#ifdef DEBUG_OUTPUT
				SerialTimePrintf(F("cmdName arg=%s\r\n"), arg);
				SerialTimePrintf(F("cmdName sizeof(eeprom.MotorName[motor])=%d\r\n"), sizeof(eeprom.MotorName[motor]) );
				SerialTimePrintf(F("cmdName &eeprom.MotorName[motor]=%p\r\n"), &eeprom.MotorName[motor] );
				#endif
				strncpy((char *)&eeprom.MotorName[motor], arg, sizeof(eeprom.MotorName[motor])-1);
				eeprom.MotorName[motor][sizeof(eeprom.MotorName[motor])-1]='\0';
				eepromWriteVars();
				cmdOK();
			}
			else {
				cmdNamePrintStatus(motor);
				cmdOK();
			}
		}
	}
}
void cmdNamePrintStatus(int motor)
{
	SerialPrintf(F("M%02d %s\r\n"), motor+1, (char *)eeprom.MotorName[motor]);
	watchdogReset();
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
			cmdTypePrintStatus(motor);
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
				cmdTypePrintStatus(motor);
				cmdOK();
			}
		}
	}
}
void cmdTypePrintStatus(int motor)
{
	SerialPrintf(F("M%02d %-8s (%S)\r\n"), motor+1, getMotorType(motor)==WINDOW ? F("WINDOW") : F("JALOUSIE"), (char *)eeprom.MotorName[motor]);
	watchdogReset();
}


void cmdRainSensor()
{
//~ "RAIN, RAINSENSOR [<cmd>]\r\n"
//~ "  Set/Get rain sensor\r\n"
//~ "     <cmd>    can be\r\n"
//~ "       AUTO        Rain detection and detection enabled from input signals\r\n"
//~ "       ENABLE      Enable rain detection, disables AUTO\r\n"
//~ "       DISABLE     Disable rain detection, disables AUTO\r\n"
//~ "       ON          Raining, disables AUTO\r\n"
//~ "       OFF         No raining, disables AUTO\r\n"
//~ "       RESUME <s>  Resume window position after rain was gone\r\n"
//~ "                   <s> is the delay in sec after rain was gone before resume starr\r\n"
//~ "                   and before resume starts.\r\n"
//~ "       FORGET      Do not remember window position, keep it close\r\n"
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

		if( bitRead(eeprom.Rain, RAIN_BIT_AUTO) ) {
			sensorEnabled = (debEnable.read() == RAIN_ENABLE_AKTIV);
		}
		else {
			sensorEnabled = bitRead(eeprom.Rain, RAIN_BIT_ENABLE);
		}
		SerialPrintf(F("Sensor monitoring:  %S  reading from %S\r\n")
				,sensorEnabled?F("Enabled"):F("Disabled")
				,bitRead(eeprom.Rain, RAIN_BIT_AUTO)?F("input"):F("setting"));
		SerialPrintf(F("Rainsensor input:   %S\r\n"), (debInput.read()==RAIN_INPUT_AKTIV)?F("Raining"):F("Dry"));
		SerialPrintf(F("Rainsensor setting: %S\r\n"), softRainInput?F("Raining"):F("Dry"));
		SerialPrintf(F("Window position:    %S, delay: %d sec\r\n"), bitRead(eeprom.Rain, RAIN_BIT_RESUME)?F("Resume"):F("Forget"), eeprom.RainResumeTime);
		SerialPrintf(F("Rainsensor status:  %S\r\n"), sensorEnabled && ((debInput.read()==RAIN_INPUT_AKTIV) || softRainInput) ? F("Raining") : F("Dry"));
		cmdOK();
	}
	else {
		if ( strnicmp(arg, fstrON,2)==0 ) {
			softRainInput = true;
			bitClear(eeprom.Rain, RAIN_BIT_AUTO);
			cmd = true;
		}
		else if ( strnicmp(arg, F("OF"),2)==0 ) {
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

void cmdStatus()
{
//~ "    status [on|off]\r\n"
//~ "         Set/Get FS20Uno status message\r\n"
//~ "         on        Status messages enabled\r\n"
//~ "         off       Status messages disabled\r\n"
	char *arg;

	arg = SCmd.next();
	if (arg == NULL) {
		SerialPrintf(eeprom.SendStatus?F("ON\r\n"):F("OFF\r\n"));
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
		SerialPrintf(F("Interval %d ms\r\n"), eeprom.BlinkInterval);
		SerialPrintf(F("Flash %d ms\r\n"), eeprom.BlinkLen);
		cmdOK();
	}
	else if ( argInterval != NULL && argFlash != NULL ) {
		Interval=(WORD)atoi(argInterval);
		Flash=(WORD)atoi(argFlash);
		if ( Interval<Flash ) {
			cmdError(F("Wrong option, flash must be >= interval time"));
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
	unsigned long eepromCRC = eepromCalcCRC() - 1;
	EEPROM.put(EEPROM_ADDR_CRC32, eepromCRC);
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

void cmdErrorParameter(String err)
{
	SerialPrintf(F("ERROR: Unknown option (use "));
	Serial.print(err);
	Serial.println(")");
}

// This gets set as the default handler, and gets called when no other command matches.
void unrecognized()
{
	SerialPrintf(F("Unknown command, try HELP\r\n"));
}
