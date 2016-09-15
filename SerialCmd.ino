/* ===================================================================
 * Command Interface Functions
 * ===================================================================*/
SerialCommand SCmd;   		// SerialCommand object


enum commands
{
	 CMD_HELP=0
	,CMD_INFO
	,CMD_ECHO
	,CMD_TERM
	,CMD_STATUS
	,CMD_UPTIME
	,CMD_FS20
	,CMD_LED
	,CMD_MOTOR
	,CMD_MOTORNAME
	,CMD_MOTORTIME
	,CMD_MOTORTYPE
	,CMD_PUSHBUTTON
	,CMD_RAIN
	,CMD_BACKUP
	,CMD_RESTORE
	,CMD_FACTORY
	,CMD_REBOOT
} COMMANDS;

/* ===================================================================
 * PROGMEM Strings & Arrays
 * ===================================================================*/
const char fstrCR[]				PROGMEM = "CR";
const char fstrLF[]				PROGMEM = "LF";
const char fstrENABLE[] 		PROGMEM = "ENSABLE";
const char fstrDISABLE[]		PROGMEM = "DISABLE";
const char fstrAUTO[] 			PROGMEM = "AUTO";
const char fstrMANUAL[]			PROGMEM = "MANUAL";
const char fstrRESUME[]			PROGMEM = "RESUME";
const char fstrFORGET[]			PROGMEM = "FORGET";
const char fstrWET[]			PROGMEM = "WET";
const char fstrDRY[]			PROGMEM = "DRY";

const char sCmdHELP[]			PROGMEM = "HELP";
const char sCmdINFO[]			PROGMEM = "INFO";
const char sCmdECHO[]			PROGMEM = "ECHO";
const char sCmdTERM[]			PROGMEM = "TERM";
const char sCmdSTATUS[]			PROGMEM = "STATUS";
const char sCmdUPTIME[]			PROGMEM = "UPTIME";
const char sCmdLED[]			PROGMEM = "LED";
const char sCmdMOTOR[]			PROGMEM = "MOTOR";
const char sCmdMOTORNAME[]		PROGMEM = "MOTORNAME";
const char sCmdMOTORTIME[]		PROGMEM = "MOTORTIME";
const char sCmdMOTORTYPE[]		PROGMEM = "MOTORTYPE";
const char sCmdFS20[]			PROGMEM = "FS20";
const char sCmdPUSHBUTTON[]		PROGMEM = "PB";
const char sCmdRAIN[]			PROGMEM = "RAIN";
const char sCmdBACKUP[]			PROGMEM = "BACKUP";
const char sCmdRESTORE[]		PROGMEM = "RESTORE";
const char sCmdFACTORY[]		PROGMEM = "FACTORY";
const char sCmdREBOOT[]			PROGMEM = "REBOOT";

#ifndef DEBUG_OUTPUT
const char sParmHELP[]			PROGMEM = "[<cmd>|ALL]";
const char sParmINFO[]			PROGMEM = "";
const char sParmECHO[]			PROGMEM = "[ON|OFF]";
const char sParmTERM[]			PROGMEM = "[CR|LF]";
const char sParmSTATUS[]		PROGMEM = "[ON|OFF]";
const char sParmUPTIME[]		PROGMEM = "[<uptime> [<h>]]";
const char sParmLED[]			PROGMEM = "[<len> [<count> [<normal> [<rain>]]]]";
const char sParmMOTOR[]			PROGMEM = "[<m> [[T]OPEN|[T]CLOSE|TOOGLE|[GOTO ]<pos>|OFF|SYNC|STATUS]";
const char sParmMOTORNAME[]		PROGMEM = "[<m> [<name>]";
const char sParmMOTORTIME[]		PROGMEM = "[<m> [<sec> [<overtravel>]]";
const char sParmMOTORTYPE[]		PROGMEM = "[<m> [<type>]";
const char sParmFS20[]			PROGMEM = "[<ch> [ON|OFF|PRG]]";
const char sParmPUSHBUTTON[]	PROGMEM = "[<b> [ON|OFF]]";
const char sParmRAIN[]			PROGMEM = "[AUTO|ENABLE|DISABLE|WET|ON|DRY|OFF|RESUME [<delay>]|FORGET]";
const char sParmBACKUP[]		PROGMEM = "";
const char sParmRESTORE[]		PROGMEM = "<addr> <data>";
const char sParmFACTORY[]		PROGMEM = "";
const char sParmREBOOT[]		PROGMEM = "";

const char sDescHELP[]			PROGMEM = "Print command help";
const char sDescINFO[]			PROGMEM = "Print version";
const char sDescECHO[]			PROGMEM = "Get/Set local echo";
const char sDescTERM[]			PROGMEM = "Get/Set command terminator";
const char sDescSTATUS[]		PROGMEM = "Get/Set status messages";
const char sDescUPTIME[]		PROGMEM = "Get/Set system uptime";
const char sDescLED[]			PROGMEM = "Get/Set LED control";
const char sDescMOTOR[]			PROGMEM = "Get/Set motor control";
const char sDescMOTORNAME[]		PROGMEM = "Get/Set motor name";
const char sDescMOTORTIME[]		PROGMEM = "Get/Set motor runtime";
const char sDescMOTORTYPE[]		PROGMEM = "Get/Set motor type";
const char sDescFS20[]			PROGMEM = "Get/Set FS20 status";
const char sDescPUSHBUTTON[]	PROGMEM = "Get/Set pushbutton status";
const char sDescRAIN[]			PROGMEM = "Rain sensor function";
const char sDescBACKUP[]		PROGMEM = "Create backup from EEPROM";
const char sDescRESTORE[]		PROGMEM = "Restore data into EEPROM";
const char sDescFACTORY[]		PROGMEM = "Reset to factory defaults";
const char sDescREBOOT[]		PROGMEM = "Restart controller";

const char sPDescHELP[]			PROGMEM = "    <cmd>  command for getting help\r\n"
								          "    ALL    print full help\r\n";
const char sPDescINFO[]			PROGMEM = "";
const char sPDescECHO[]			PROGMEM = "    ON|OFF  set local echo ON or OFF";
const char sPDescTERM[]			PROGMEM = "    CR|LF   set terminator to CR or LF";
const char sPDescSTATUS[]		PROGMEM = "    ON|OFF  set status messages ON or OFF";
const char sPDescUPTIME[]		PROGMEM = "    <uptime>  set uptime (in ms since start)\r\n";
const char sPDescLED[]			PROGMEM = "    <len>     pattern bit time in ms\r\n"
								          "    <count>   pattern bit count [1.." TOSTRING(MAX_LEDPATTERN_BITS) "]\r\n"
								          "    <normal>  normal blink pattern\r\n"
								          "    <rain>    rain blink pattern";
const char sPDescMOTOR[]		PROGMEM = "    <m>    motor number [1.." TOSTRING(MAX_MOTORS) "]\r\n"
								          "    <cmd>  can be\r\n"
								          "      OPEN       start motor opening\r\n"
								          "      CLOSE      start motor closing\r\n"
								          "      TOPEN      toogle opening\r\n"
								          "      TCLOSE     toogle closing\r\n"
								          "      TOOGLE     Toogle anyway\r\n"
								          "      [GOTO] <p> goto position <p> (in %, 0-100)\r\n"
								          "      OFF        stop motor\r\n"
								          "      SYNC       set motor in a defined state\r\n"
								          "      STATUS     get the current status";
const char sPDescMOTORNAME[]	PROGMEM = "    <m>   motor number [1.." TOSTRING(MAX_MOTORS) "]\r\n"
								          "    <name motor name";
const char sPDescMOTORTIME[]	PROGMEM = "    <m>          motor number [1.." TOSTRING(MAX_MOTORS) "]\r\n"
								          "    <sec>        maximum runtime [s]\r\n"
								          "    <overtravel> overtravel time [s]";
const char sPDescMOTORTYPE[]	PROGMEM = "    <m>     motor number [1.." TOSTRING(MAX_MOTORS) "]\r\n"
								          "    <type>  WINDOW or JALOUSIE";
const char sPDescFS20[]			PROGMEM = "    <ch>    FS20 channel number [1.." TOSTRING(IOBITS_CNT) "]\r\n"
								          "    ON|OFF  set <ch> ON or OFF\r\n"
								          "    PRG     set <ch> into programming mode";
const char sPDescPUSHBUTTON[]	PROGMEM = "    <b>     pushbutton number [1.." TOSTRING(IOBITS_CNT) "]\r\n"
								          "    ON|OFF  set <b> ON or OFF";
const char sPDescRAIN[]			PROGMEM = "    <cmd>  can be\r\n"
								          "      AUTO       rain detection enabled from input signals\r\n"
								          "      ENABLE     enable rain detection\r\n"
								          "      DISABLE    disable rain detection\r\n"
								          "      WET|ON     start raining\r\n"
								          "      DRY|OFF    stop raining\r\n"
								          "      RESUME <s> auto-resume window position after rain was gone.\r\n"
								          "                 <s> is the delay (in sec) before resume starts.\r\n"
								          "      FORGET     do not resume window position\r\n"
								          "  Note: ON|WET, OFF|DRY, ENABLE or DISABLE disables AUTO mode";
const char sPDescBACKUP[]		PROGMEM = "";
const char sPDescRESTORE[]		PROGMEM = "    <addr>  4-digit hex destination address\r\n"
								          "    <data>  hex data to restore";
const char sPDescFACTORY[]		PROGMEM = "";
const char sPDescREBOOT[]		PROGMEM = "";


const char* const sCmdTable[][4] PROGMEM = {
	 {sCmdHELP		,sParmHELP		,sDescHELP		,sPDescHELP}
	,{sCmdINFO		,sParmINFO		,sDescINFO		,sPDescINFO}
	,{sCmdECHO		,sParmECHO		,sDescECHO		,sPDescECHO}
	,{sCmdTERM		,sParmTERM		,sDescTERM		,sPDescTERM}
	,{sCmdSTATUS	,sParmSTATUS	,sDescSTATUS	,sPDescSTATUS}
	,{sCmdUPTIME	,sParmUPTIME	,sDescUPTIME	,sPDescUPTIME}
	,{sCmdLED		,sParmLED		,sDescLED		,sPDescLED}
	,{sCmdMOTOR		,sParmMOTOR		,sDescMOTOR		,sPDescMOTOR}
	,{sCmdMOTORNAME	,sParmMOTORNAME	,sDescMOTORNAME	,sPDescMOTORNAME}
	,{sCmdMOTORTIME	,sParmMOTORTIME	,sDescMOTORTIME	,sPDescMOTORTIME}
	,{sCmdMOTORTYPE	,sParmMOTORTYPE	,sDescMOTORTYPE	,sPDescMOTORTYPE}
	,{sCmdFS20		,sParmFS20		,sDescFS20		,sPDescFS20}
	,{sCmdPUSHBUTTON,sParmPUSHBUTTON,sDescPUSHBUTTON,sPDescPUSHBUTTON}
	,{sCmdRAIN		,sParmRAIN		,sDescRAIN		,sPDescRAIN}
	,{sCmdBACKUP	,sParmBACKUP	,sDescBACKUP	,sPDescBACKUP}
	,{sCmdRESTORE	,sParmRESTORE	,sDescRESTORE	,sPDescRESTORE}
	,{sCmdFACTORY	,sParmFACTORY	,sDescFACTORY	,sPDescFACTORY}
	,{sCmdREBOOT	,sParmREBOOT	,sDescREBOOT	,sPDescREBOOT}
};
const void (*cmdTable[])() = {
	 (const void (*)())cmdHelp
	,(const void (*)())cmdInfo
	,(const void (*)())cmdEcho
	,(const void (*)())cmdTerm
	,(const void (*)())cmdStatus
	,(const void (*)())cmdUptime
	,(const void (*)())cmdLed
	,(const void (*)())cmdMotor
	,(const void (*)())cmdMotorName
	,(const void (*)())cmdMotorTime
	,(const void (*)())cmdMotorType
	,(const void (*)())cmdFS20
	,(const void (*)())cmdPushButton
	,(const void (*)())cmdRain
	,(const void (*)())cmdBackup
	,(const void (*)())cmdRestore
	,(const void (*)())cmdFactory
	,(const void (*)())cmdReboot
};
#endif

/* ===================================================================
 * Helper Function
 * ===================================================================*/
void setupSerialCommand(void)
{
	#ifndef DEBUG_OUTPUT
	// Setup callbacks for SerialCommand commands
	for(byte i=0; i<sizeof(sCmdTable)/sizeof(sCmdTable[0]); i++) {
		SCmd.addCommand( (char*)pgm_read_word(&(sCmdTable[i][PRINT_CMD])), (void (*)())cmdTable[i] );
	}
	#else
	// do it step by step without help texts to save all the space for program memory
	SCmd.addCommand( sCmdHELP,		cmdHelp );
	SCmd.addCommand( sCmdINFO,		cmdInfo );
	SCmd.addCommand( sCmdECHO,		cmdEcho );
	SCmd.addCommand( sCmdTERM,		cmdTerm );
	SCmd.addCommand( sCmdSTATUS,	cmdStatus );
	SCmd.addCommand( sCmdUPTIME,	cmdUptime );
	SCmd.addCommand( sCmdLED,		cmdLed );
	SCmd.addCommand( sCmdMOTOR,		cmdMotor );
	SCmd.addCommand( sCmdMOTORNAME,	cmdMotorName );
	SCmd.addCommand( sCmdMOTORTIME,	cmdMotorTime );
	SCmd.addCommand( sCmdMOTORTYPE,	cmdMotorType );
	SCmd.addCommand( sCmdFS20,		cmdFS20 );
	SCmd.addCommand( sCmdPUSHBUTTON,cmdPushButton );
	SCmd.addCommand( sCmdRAIN,		cmdRain );
	SCmd.addCommand( sCmdBACKUP,	cmdBackup );
	SCmd.addCommand( sCmdRESTORE,	cmdRestore );
	SCmd.addCommand( sCmdFACTORY,	cmdFactory );
	SCmd.addCommand( sCmdREBOOT,	cmdReboot );
	#endif

	SCmd.addDefaultHandler(unrecognized);   // Handler for command that isn't matched  (says "What?")
	SCmd.setEcho(eeprom.Echo);
	SCmd.setTerm(eeprom.Term);
}

void processSerialCommand(void)
{
	SCmd.readSerial();     // We don't do much, just process serial commands
	watchdogReset();
}

void cmdOK(void)
{
	Serial.println(F("OK"));
}

void cmdError(const __FlashStringHelper *str)
{
	Serial.print(F("ERROR: "));
	Serial.println(str);
}

void cmdErrorParameter(const __FlashStringHelper *err)
{
	Serial.print(F("ERROR: Parameter error (use "));
	Serial.print(err);
	Serial.println(")");
}

void cmdErrorOutOfRange(const __FlashStringHelper *str)
{
	Serial.print(F("ERROR: "));
	Serial.print(str);
	Serial.println(F(" out of range"));
}

void unrecognized()
{
	SerialPrintfln(F("Unknown command, try HELP"));
}

/* ===================================================================
 * Command interface function implementation
 * ===================================================================*/

/* HELP [cmd]
 *   Command help
 */
void cmdHelp()
{
	#ifdef DEBUG_OUTPUT
	Serial.print(F("No HELP when debug is enabled!"));
	#else
	char *arg;

	arg = SCmd.next();
	if ( (arg == NULL) || (strcasecmp_P(arg, PSTR("ALL"))==0) ) {
		helpHeader();
		for(byte i=0; i<sizeof(sCmdTable)/sizeof(sCmdTable[0]); i++) {
			printHelp(i, PRINT_CMD, " ");
			printHelp(i, PRINT_PARM, "\r\n  ");
			printHelp(i, PRINT_DESC, "\r\n");
			if ( arg != NULL ) {
				printHelp(i, PRINT_PDESC, "\r\n\r\n");
			}
		}
		if ( arg == NULL ) {
			Serial.println(F("\r\nFor more info, use 'HELP cmd'"));
		}
	}
	else {

		bool cmdFound=false;
		for(byte i=0; !cmdFound && i<sizeof(sCmdTable)/sizeof(sCmdTable[0]); i++) {
			if( strcasecmp_P(arg, (char *)pgm_read_word(&(sCmdTable[i][PRINT_CMD])))==0 ) {
				printHelp(i, PRINT_CMD, " ");
				printHelp(i, PRINT_PARM, "\r\n  ");
				printHelp(i, PRINT_DESC, "\r\n");
				printHelp(i, PRINT_PDESC, "\r\n");
				cmdFound=true;
			}
		}
		if ( !cmdFound ) {
			SerialPrintfln(F("ERROR: Unknown command '%s', use HELP for command list"), arg);
			return;
		}
	}
	#endif

	watchdogReset();
	cmdOK();
}
#ifndef DEBUG_OUTPUT
int printHelp(byte cmd, printCmdType type, const char *postfix)
{
	char *p = NULL;

	switch ( type ) {
		case PRINT_CMD:
		case PRINT_PARM:
		case PRINT_DESC:
		case PRINT_PDESC:
			p = pgm_read_word(&(sCmdTable[cmd][type]));
			break;
	}
	if ( p!=NULL ) {
		unsigned char c;
		
		while( (c = pgm_read_byte(p++)) != 0 ) {
			Serial.write(c);
		}
		if ( postfix!=NULL ) {
			Serial.print(postfix);
		}
		return strlen_P(p);
	}
	
	return 0;
}
void helpHeader()
{
	SerialPrintfln(F("\r\n"
					 "%S COMMAND LIST\r\n"
					 "--------------------"), PROGRAM);
}
#endif

/* INFO 
 *   Print version
 */
void cmdInfo()
{
	printProgramInfo(false);
	cmdUptime();
	SerialPrintfln(F("EEPROM CRC32: %08lx"), CalcCRC(RAMCRC, (byte *)&eeprom, sizeof(eeprom)));
	cmdOK();
}

/* ECHO [ON|OFF]
 *   Local echo
 */
void cmdEcho(void)
{
	char *arg;

	arg = SCmd.next();
	if (arg == NULL) {
		sendStatus(true,SYSTEM,F("ECHO %S"), eeprom.Echo?fstrON:fstrOFF);
		cmdOK();
	}
	else if ( strnicmp_P(arg, fstrON,2)==0 ) {
		eeprom.Echo = true;
		SCmd.setEcho(eeprom.Echo);
		eepromWriteVars();
		cmdOK();
	}
	else if ( strnicmp_P(arg, fstrOFF,2)==0 ) {
		eeprom.Echo = false;
		SCmd.setEcho(eeprom.Echo);
		eepromWriteVars();
		cmdOK();
	}
	else {
		cmdErrorParameter(F("'ON' or 'OFF'"));
	}
}

/* TERM [CR|LF]
 *   Command terminator
 */
void cmdTerm(void)
{
	char *arg;

	arg = SCmd.next();
	if (arg == NULL) {
		sendStatus(true,SYSTEM,F("TERM %S"),eeprom.Term=='\r'?fstrCR:fstrLF);
		cmdOK();
	}
	else if ( strnicmp_P(arg, fstrCR,2)==0 ) {
		eeprom.Term = '\r';
		SCmd.setTerm(eeprom.Term);
		eepromWriteVars();
		cmdOK();
	}
	else if ( strnicmp_P(arg, fstrLF,2)==0 ) {
		eeprom.Term = '\n';
		SCmd.setTerm(eeprom.Term);
		eepromWriteVars();
		cmdOK();
	}
	else {
		cmdErrorParameter(F("'CR' or 'LF'"));
	}
}

/* STATUS [ON|OFF]
 *   Send status messages
 */
void cmdStatus()
{
	char *arg;

	arg = SCmd.next();
	if (arg == NULL) {
		sendStatus(true,SYSTEM,F("STATUS %S"), eeprom.SendStatus?fstrON:fstrOFF);
		cmdOK();
	}
	else {
		if ( strnicmp_P(arg, fstrON,2)==0 ) {
			eeprom.SendStatus = true;
			eepromWriteVars();
			cmdOK();
		}
		else if ( strnicmp_P(arg, fstrOFF, 2)==0 ) {
			eeprom.SendStatus = false;
			eepromWriteVars();
			cmdOK();
		}
		else {
			cmdErrorParameter(F("'ON' or 'OFF'"));
		}
	}
}

/* UPTIME [<uptime> [<h>]]
 *   System uptime
 *     <uptime> Set uptime (in ms since start)
 *     <h>      Set operation hours (in h)
 */
void cmdUptime()
{
	char *arg;

	arg = SCmd.next();
	if (arg != NULL) {

		cli(); //halt the interrupts
		timer0_millis =  (unsigned long)atol(arg);
		sei(); //re-enable the interrupts
		operationHours(true);

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

/* LED [<len> [<count> [<normal> [<rain>]]]]
 *   LED control
 *     <len>     pattern bit time in ms
 *     <count>   pattern bit count (max 32)
 *     <normal>  normal blink pattern
 *     <rain>    rain blink pattern
 */
void cmdLed()
{
	LEDPATTERN 	LEDPatternNormal;
	LEDPATTERN 	LEDPatternRain;
	byte 		LEDBitCount;
	WORD 		LEDBitLenght;					

	char *argLEDPatternNormal;
	char *argLEDPatternRain;
	char *argLEDBitCount;
	char *argLEDBitLenght;

	LEDBitLenght		= eeprom.LEDBitLenght;
	LEDBitCount			= eeprom.LEDBitCount;
	LEDPatternNormal	= eeprom.LEDPatternNormal;
	LEDPatternRain		= eeprom.LEDPatternRain;

	argLEDBitLenght		= SCmd.next();
	argLEDBitCount		= SCmd.next();
	argLEDPatternNormal	= SCmd.next();
	argLEDPatternRain	= SCmd.next();
	if ( argLEDBitLenght == NULL ) {
		sendStatus(true,SYSTEM,F("LED %d %d 0x%08lx 0x%08lx")
					,eeprom.LEDBitLenght
					,eeprom.LEDBitCount
					,eeprom.LEDPatternNormal
					,eeprom.LEDPatternRain
					);
		cmdOK();
	}
	else {
		LEDBitLenght	= (WORD)atoi(argLEDBitLenght);
		if ( argLEDBitCount!=NULL ) {
			LEDBitCount		= (byte)atoi(argLEDBitCount);
			if ( LEDBitCount<1 || LEDBitCount>MAX_LEDPATTERN_BITS ) {
				cmdErrorOutOfRange(F("<bits>"));
				return;
			}
		}
		if ( argLEDPatternNormal!=NULL ) {
			LEDPatternNormal = (LEDPATTERN)strtoul(argLEDPatternNormal, NULL, 0);
			if ( errno == ERANGE ) {
				cmdErrorOutOfRange(F("<normal>"));
				return;
			}
		}
		if ( argLEDBitCount!=NULL ) {
			LEDPatternRain = (LEDPATTERN)strtoul(argLEDPatternRain, NULL, 0);
			if ( errno == ERANGE ) {
				cmdErrorOutOfRange(F("<rain>"));
				return;
			}
		}
		eeprom.LEDPatternNormal	= LEDPatternNormal;
		eeprom.LEDPatternRain	= LEDPatternRain;
		eeprom.LEDBitCount		= LEDBitCount;
		eeprom.LEDBitLenght		= LEDBitLenght;
		eepromWriteVars();
		cmdOK();
	}
}

/* MOTOR [<m> [[T]OPEN|[T]CLOSE|TOOGLE|[GOTO ]<pos>|OFF|SYNC|STATUS]
 *   Motor control
 *     <m>      Motor number [1..MAX_MOTORS]
 *     <cmd>    can be
 *       OPEN       Motor in OPEN direction
 *       CLOSE      Motor in CLOSE direction
 *       TOPEN      Toogle OPEN direction
 *       TCLOSE     Toogle CLOSE direction
 *       TOOGLE     Toogle direction
 *       [GOTO] <p> Goto position <p> (in %, 0-100)
 *       OFF        Stop motor
 *       SYNC       Set motor in a default defined state
 *       STATUS     Return the current status
 */
void cmdMotor()
{
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
			cmdErrorOutOfRange(F("<m>"));
		}
		else {
			arg = SCmd.next();
			if (arg != NULL) {
				if      ( strnicmp_P(arg, PSTR("OP"),2)==0 ) {
					if( getMotorDirection(motor)<MOTOR_OPEN ) {
						setMotorDirection(motor, MOTOR_OPEN);
					}
					cmdOK();
				}
				else if ( strnicmp_P(arg, PSTR("CL"),2)==0 ) {
					if( getMotorDirection(motor)>MOTOR_CLOSE ) {
						setMotorDirection(motor, MOTOR_CLOSE);
					}
					cmdOK();
				}
				else if ( strnicmp_P(arg, fstrOFF, 2)==0 ) {
					if( getMotorDirection(motor)!=MOTOR_OFF ) {
						setMotorDirection(motor, MOTOR_OFF);
					}
					cmdOK();
				}
				else if ( strnicmp_P(arg, PSTR("GO"),2)==0 ) {
					arg = SCmd.next();
					int percent=atoi(arg);

					if ( percent<0 || percent>100 ) {
						cmdError(F("<p> of range"));
					}
					else {
						setMotorPosition(motor, percent);
						cmdOK();
					}
				}
				else if ( strnicmp_P(arg, PSTR("TOP"),3)==0 ) {
					if( getMotorDirection(motor)==MOTOR_OFF || getMotorDirection(motor)<=MOTOR_CLOSE ) {
						setMotorDirection(motor, MOTOR_OPEN);
					}
					else {
						setMotorDirection(motor, MOTOR_OFF);
					}
					cmdOK();
				}
				else if ( strnicmp_P(arg, PSTR("TCL"),3)==0 ) {
					if( getMotorDirection(motor)==MOTOR_OFF || getMotorDirection(motor)>=MOTOR_OPEN  ) {
						setMotorDirection(motor, MOTOR_CLOSE);
					}
					else {
						setMotorDirection(motor, MOTOR_OFF);
					}
					cmdOK();
				}
				else if ( strnicmp_P(arg, PSTR("TOG"),3)==0 ) {
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
				else if ( strnicmp_P(arg, PSTR("SY"),2)==0 ) {
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
			if (arg == NULL || strnicmp_P(arg, PSTR("ST"),2)==0 ) {
				sendMotorStatus(true,motor);
				cmdOK();
			}
		}
	}
}

/* MOTORTIME [<m> [<sec> [<overtravel>]]
 *   Motor runtime
 *     <m>          Motor number [1..8]
 *     <sec>        Maximum runtime [s]
 *     <overtravel> Overtravel time [s]
 */
void cmdMotorTime()
{
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
			cmdErrorOutOfRange(F("<m>"));
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
					cmdErrorOutOfRange(F("<sec>"));
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
							cmdErrorOutOfRange(F("<overtravel>"));
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

/* MOTORNAME [<m> [<name>]
 *   Motor names
 *     <m>   Motor number [1..MAX_MOTORS]
 *     <name Motor name
 */
void cmdMotorName()
{
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
			cmdErrorOutOfRange(F("<m>"));
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

/* MOTORTYPE [<m> [WINDOW|JALOUSIE]
 *   Motor type
 *     <m>   Motor number [1..MAX_MOTORS]
 *     <cmd> 'WINDOW' or 'JALOUSIE'
 */
void cmdMotorType()
{
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
			cmdErrorOutOfRange(F("<m>"));
		}
		else {
			arg = SCmd.next();
			if (arg != NULL) {
				if      ( strnicmp_P(arg, F("WIN"),3)==0 ) {
					setMotorType(motor, WINDOW);
					eepromWriteVars();
					cmdOK();
				}
				else if ( strnicmp_P(arg, F("JAL"),3)==0 ) {
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

/* FS20 [<ch> [ON|OFF|PRG]]
 *   FS20 control
 *     <ch> FS20 channel number [1..IOBITS_CNT]
 *     ON   set <ch> ON
 *     OFF  set <ch> OFF
 *     PRG  set <ch> to programming mode
 */
void cmdFS20()
{
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
			cmdErrorOutOfRange(F("<ch>"));
		}
		else {
			// Channel number ok
			arg = SCmd.next();
			if (arg != NULL) {
				// ON|OFF entered?
				if      ( strnicmp_P(arg, fstrON, 2)==0 ) {
					if( !bitRead(curSM8Status, channel) ) {
						bitClear(valSM8Button, channel);
					}
					cmdOK();
				}
				else if ( strnicmp_P(arg, fstrOFF, 2)==0 ) {
					if( bitRead(curSM8Status, channel) ) {
						bitClear(valSM8Button, channel);
					}
					cmdOK();
				}
				else if ( strnicmp_P(arg, PSTR("PRG"),3)==0 ) {
					bitSet(SM8StatusIgnore, channel);
					bitClear(valSM8Button, channel);
					expanderWriteWord(MPC_SM8BUTTON,   GPIO, valSM8Button);
					SerialPrintf(F("Setting FS20 %d into program mode, please wait:  "), channel+1);
					for(byte d=0; d<(FS20_SM8_IN_PROGRAMMODE/1000); d++) {
						SerialPrintf(F("\b%1d"), (FS20_SM8_IN_PROGRAMMODE/1000)-d);
						delay(1000);
						watchdogReset();
					}
					bitSet(SM8StatusIgnore, channel);
					bitClear(valSM8Button, channel);
					expanderWriteWord(MPC_SM8BUTTON,   GPIO, valSM8Button);
					SerialPrintfln(F("\r\nFS20 %d now in programming mode"), channel+1);
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

/* PUSHBUTTON [<b> [ON|OFF]]
 *   Pushbutton status
 *     <b>    Pushbutton number [1..IOBITS_CNT]
 *     ON|OFF set <b> ON or OFF
 */
void cmdPushButton()
{
	int button;
	char *arg;

	arg = SCmd.next();
	if (arg == NULL) {
		for(button=0; button<IOBITS_CNT; button++) {
			sendStatus(true,PUSHBUTTON, F("%02d %S"), button+1, bitRead(curPushButton, button)?fstrON:fstrOFF);
		}
		cmdOK();
	}
	else {
		// Button number entered
		button=atoi(arg)-1;
		if ( button<0 || button>=IOBITS_CNT ) {
			cmdErrorOutOfRange(F("<b>"));
		}
		else {
			// Button number ok
			arg = SCmd.next();
			if (arg != NULL) {
				// ON|OFF entered?
				if      ( strnicmp_P(arg, fstrON,2)==0 ) {
					bitSet(curPushButton, button);
					cmdOK();
				}
				else if ( strnicmp_P(arg, fstrOFF, 2)==0 ) {
					bitClear(curPushButton, button);
					cmdOK();
				}
				else {
					cmdErrorParameter(F("'ON' or 'OFF'"));
				}
			}
			else {
				sendStatus(true,PUSHBUTTON, F("%02d %S"), button+1, bitRead(curPushButton, button)?fstrON:fstrOFF);
				cmdOK();
			}
		}
	}
}

/* RAIN [AUTO|ENABLE|DISABLE|WET|ON|DRY|OFF|RESUME [<delay>]|FORGET]
 *   Rain sensor function
 *     <cmd>    can be
 *       AUTO       Rain detection enabled from input signals
 *       ENABLE     Enable rain detection
 *       DISABLE    Disable rain detection
 *       WET|ON     Raining
 *       DRY|OFF    Dry
 *       RESUME <s> Resume window position after rain was gone
 *                  <s> is the delay (sec) before resume starts
 *       FORGET     Don not resume, keep windows closed
 *   ON|WET, OFF|DRY, ENABLE or DISABLE disables AUTO mode
 */
void cmdRain()
{
	char *arg;
	bool cmd = false;

	arg = SCmd.next();
	if (arg == NULL) {
		cmdRainSensorPrintStatus();
		cmdOK();
	}
	else {
		if ( strnicmp_P(arg, fstrON,2)==0 || strnicmp_P(arg, fstrWET,3)==0 ) {
			softRainInput = true;
			bitClear(eeprom.Rain, RAIN_BIT_AUTO);
			cmd = true;
		}
		if ( strnicmp_P(arg, fstrOFF,2)==0 || strnicmp_P(arg, fstrDRY,3)==0 ) {
			softRainInput = false;
			bitClear(eeprom.Rain, RAIN_BIT_AUTO);
			cmd = true;
		}
		else if ( strnicmp_P(arg, fstrENABLE,2)==0 ) {
			bitSet(eeprom.Rain, RAIN_BIT_ENABLE);
			bitClear(eeprom.Rain, RAIN_BIT_AUTO);
			cmd = true;
		}
		else if ( strnicmp_P(arg, fstrDISABLE,2)==0 ) {
			bitClear(eeprom.Rain, RAIN_BIT_ENABLE);
			bitClear(eeprom.Rain, RAIN_BIT_AUTO);
			cmd = true;
		}
		else if ( strnicmp_P(arg, fstrAUTO,2)==0 ) {
			bitSet(eeprom.Rain, RAIN_BIT_AUTO);
			cmd = true;
		}
		else if ( strnicmp_P(arg, fstrRESUME,2)==0 ) {
			int delay = eeprom.RainResumeTime;
			arg = SCmd.next();
			if( arg!=NULL ) {
				delay=atoi(arg);
			}
			if ( delay<0 || delay>6000 ) {
				cmdError(F("<s> (max 6000) of range"));
			}
			else {
				eeprom.RainResumeTime = delay;
				bitSet(eeprom.Rain, RAIN_BIT_RESUME);
				cmd = true;
			}
		}
		else if ( strnicmp_P(arg, fstrFORGET,2)==0 ) {
			// Clear resumeMotorPosition[]
			for(size_t i=0; i<(sizeof(resumeMotorPosition)/sizeof(resumeMotorPosition[0])); i++) {
				resumeMotorPosition[i] = NO_RESUME_POSITION;
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
		sensorEnabled = (debEnable.read() == RAIN_ENABLE_ACTIVE);
	}
	else {
		sensorEnabled = bitRead(eeprom.Rain, RAIN_BIT_ENABLE);
	}

	sendStatus(true,RAIN,F("%S MODE:%S IN:%S SET:%S RAIN:%SD FROM:%S POS:%S DELAY:%d")
				,sensorEnabled && ((debInput.read()==RAIN_INPUT_ACTIVE) || softRainInput) ? fstrWET : fstrDRY
				,bitRead(eeprom.Rain, RAIN_BIT_AUTO) ? fstrAUTO : fstrMANUAL
				,(debInput.read()==RAIN_INPUT_ACTIVE) ? fstrWET : fstrDRY
				,softRainInput ? fstrWET : fstrDRY
				,sensorEnabled ? fstrENABLE : fstrDISABLE
				,bitRead(eeprom.Rain, RAIN_BIT_AUTO) ? F("INPUT") : F("SET")
				,bitRead(eeprom.Rain, RAIN_BIT_RESUME) ? fstrRESUME : fstrFORGET
				,eeprom.RainResumeTime
				);
}

/* BACKUP 
 *   Create backup from EEPROM
 */
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
	SerialPrintf(F("TERM %S\r\n"), eeprom.Term=='\r'?fstrCR:fstrLF);
	// STATUS
	SerialPrintf(F("STATUS %S\r\n"), eeprom.SendStatus?fstrON:fstrOFF);
	// UPTIME
	SerialPrintf(F("UPTIME %ld %ld\r\n"), millis(), eeprom.OperatingHours);
	// LED
	SerialPrintf(F("LED %d %d 0x%08lx 0x%08lx")
					,eeprom.LEDBitLenght
					,eeprom.LEDBitCount
					,eeprom.LEDPatternNormal
					,eeprom.LEDPatternRain
					);

	// RAIN
	SerialPrintf(F("RAIN RESUME %d\r\n"), eeprom.RainResumeTime);
	if( !bitRead(eeprom.Rain, RAIN_BIT_RESUME) ) {
		SerialPrintf(F("RAIN FORGET\r\n"));
	}
	SerialPrintf(F("RAIN %S\r\n"), bitRead(eeprom.Rain, RAIN_BIT_ENABLE) ? fstrENABLE : fstrDISABLE);
	if( bitRead(eeprom.Rain, RAIN_BIT_AUTO) ) {
		SerialPrintf(F("RAIN %S\r\n"), fstrAUTO);
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

/* RESTORE <addr> <data>
 *   Restore data into EEPROM
 *     <addr>  4-digit hex destination address
 *     <data>  hex data to restore
 */
void cmdRestore(void)
{
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

/* FACTORY 
 *   Factory defaults
 */
void cmdFactory()
{
	unsigned long eepromCRC = CalcCRC(EEPROMCRC, (byte *)(EEPROM_ADDR_CRC32+4), EEPROM.length()-4) - 1;
	EEPROM.put(EEPROM_ADDR_CRC32, eepromCRC);
	eepromInitVars();
	cmdOK();
}

/* REBOOT 
 *   Restart controller
 */
void cmdReboot(void)
{
	cmdOK();

	#if WATCHDOG_REBOOT_DELAY >= 1000
		sendStatus(true,SYSTEM,F("REBOOT (in %d s)"), (WATCHDOG_REBOOT_DELAY/1000));
	#else
		sendStatus(true,SYSTEM,F("REBOOT now"));
	#endif
	Watchdog.enable(WATCHDOG_REBOOT_DELAY);
	for(byte d=0; d<(WATCHDOG_REBOOT_DELAY/1000); d++) {
		Serial.print(F("."));
		delay(1000);
	}
	while(true) {};
}
