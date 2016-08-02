/* ==========================================================================
	 File:    FS20Uno.ino
	 Author:  Norbert Richter <mail@norbert-richter.info>
	 Project: Dachflächenfenster/-Jalousie Steuerung
			mit 2x FS20 SM8
	 Desc:    Die Steuerung besteht aus
				- einem Arduino Uno
				- drei Porterweterungen MCP23017
				- zwei ELV FS20 SM8
			Der Uno über 16 Relais ingesamt acht Motoren
			- Ein Relais für Motor Ein/Aus
				- Ein Relais für die Drehrichtung.

	 Funktionsweise der Steuerung:
	 Zwei FS20 SM8 dienen als FS20 Empfänger für Dachflächenfenstersteuerung (DFF)
	 Mit Hilfer zweier FS20-SM8 stehen stehen 16 FS20 Kanäle zur Verfügung.
	 Jeweils zwei steuern einen Motor:
	 - ein Kanal steuert Linkslauf
	 - ein Kanal steuert Rechtslauf

	 Damit lassen sich insgesamt 4 DF-Fenster mit jeweils zwei 24V Motoren ansteuern:
	 - ein Motor zum Öffnen/Schliessen des Fensters
	 - ein Motor für die Jalousie
	 Der Uno übernimmt dabei die 'Übersetzung' von Öffnen/Schliessen
	 auf die Motorzustände
	 - Aus
	 - Öffnen
	 - Schließen

	 Da wir in den Ablauf des FS20 SM8 nur bedingt eingreifen können, werden die
	 FS20 SM8 nur als 'Geber' eingesetzt. Die Motorsteuerung erfolgt komplett
	 über dieses Steuerungsprogramm.

	 Verdrahtung:
	 MPC1
	 Port A (Output): Relais Motor 1-8 EIN/AUS
	 Port B (Output): Relais Motor 1-8 Drehrichtung
	 MPC2
	 Port A (Output): FS20-SM8 #1 Taster 1-8 ("Auf" Funktion: 1=DFF1, 2=DFF2, 3=DFF3, 4=DFF4, 5=Jal1...)
	 Port B (Output): FS20-SM8 #2 Taster 1-8 ("Zu" Funktion)
	 MPC3
	 Port A (Input) : FS20-SM8 #1 Status 1-8 ("Auf" Funktion)
	 Port B (Input) : FS20-SM8 #2 Status 1-8 ("Zu" Funktion)
	 Die FS20-SM8 Ausgänge schalten das Signal gegen Masse (0=Aktiv)
	 MPC4
	 Port A (Input) : Wandtaster             ("Auf" Funktion)
	 Port B (Input) : Wandtaster             ("Zu" Funktion)
	 Die Taster schalten das Signal gegen Masse (0=Aktiv).
	 Die Wandtaster haben folgende Funktion:
	 - Taste Auf: "Auf" einschalten, nochmaliger Druck schaltet Motor ab.
	 - Taste Zu : "Zu" einschalten, nochmaliger Druck schaltet Motor ab.
	 - Beide Tasten: Schaltet Motor ab.

	 Zwei Digitaleingänge des Uno werden für Zustandsvorgänge benutzt:
	 D7: Regensensor (Aktiv Low)
	 D8: Regensensor aktiv (Aktiv Low)

	 Vom SM8 werden jeweils die Ausgänge als Steuereingang für die Motorsteuerung
	 herangezogen, dabei werden folgende Bedingungen besonders berücksichtigt:
	 - gleichzeitig aktivierte Ausgänge (Auf & Zu aktiv) werden entkoppelt
	 Nur die jweils steigende Flanke eines Ausgangs steuert die Richtung.
	 War bereits die entgegengesetzte Richtung aktiv, wird die Richtung
	 umgeschaltet.
	 - Motorschutz:
	 Das Einschalten der Motorspannung erfolgt erst, nachdem das Richtungsrelais
	 auch wirklich umgeschalten hat (OPERATE_TIME).
	 Ebenso wird bei laufendem Motor und Richtungsumkehr der Motor zuerst abge-
	 schaltet, die Laufrichtung geändert und danach der Motor wieder einge-
	 schaltet (MOTOR_SWITCHOVER).
	 - Die fallende Flanke des SM8 Ausgangs schaltet die jeweilige Drehrichtung ab


	 ========================================================================== */
/* TODO
	 - Kommandos über RS232
*/

#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <PrintEx.h>			// https://github.com/Chris--A/PrintEx#printex-library-for-arduino-
#include <MsTimer2.h>			// http://playground.arduino.cc/Main/MsTimer2
#include <Bounce2.h>			// https://github.com/thomasfredericks/Bounce2
#include <Adafruit_SleepyDog.h> // 
#include <SerialCommand.h>		// https://github.com/scogswell/ArduinoSerialCommand

#include "FS20Uno.h"
#include "I2C.h"

#define PROGRAM "FS20Uno"
#define VERSION "2.07"
#include "REVISION.h"
#define DATAVERSION 105


// define next macro to enable debug output pins
#undef DEBUG_PINS
// define next macros to output debug prints
#define DEBUG_OUTPUT
#undef DEBUG_OUTPUT_SETUP
#undef DEBUG_OUTPUT_WATCHDOG
#define DEBUG_OUTPUT_EEPROM
#undef DEBUG_OUTPUT_SM8STATUS
#undef DEBUG_OUTPUT_WALLBUTTON
#undef DEBUG_OUTPUT_SM8OUTPUT
#undef DEBUG_OUTPUT_MOTOR
#define DEBUG_OUTPUT_RAIN
#undef DEBUG_OUTPUT_LIVE

#ifndef DEBUG_OUTPUT
	// enable next line to enable watchdog timer
	#define WATCHDOG_ENABLED
	#undef DEBUG_OUTPUT_SETUP
	#undef DEBUG_OUTPUT_WATCHDOG
	#undef DEBUG_OUTPUT_EEPROM
	#undef DEBUG_OUTPUT_SM8STATUS
	#undef DEBUG_OUTPUT_WALLBUTTON
	#undef DEBUG_OUTPUT_SM8OUTPUT
	#undef DEBUG_OUTPUT_MOTOR
	#undef DEBUG_OUTPUT_RAIN
	#undef DEBUG_OUTPUT_LIVE
#endif

#ifdef DEBUG_PINS
	#define DBG_INT 			12			// Debug PIN = D12
	#define DBG_MPC 			11			// Debug PIN = D11
	#define DBG_TIMER	 		10			// Debug PIN = D10
	#define DBG_TIMERLEN 		9			// Debug PIN = D9
#endif




// loop() timer vars
TIMER ledTimer = 0;
char  ledCounter = 0;
TIMER runTimer = 0;


// MPC output data

/* Motor
	 Unteren Bits: Motor Ein/Aus
	 Oberen Bits:  Motor Drehrichtung
*/
volatile IOBITS valMotorRelais = IOBITS_ZERO;

/* SM8 Tasten
	 Unteren Bits: Motor Öffnen
	 Oberen Bits:  Motor Schliessen
*/
volatile IOBITS valSM8Button   = ~IOBITS_ZERO;

// values read from MPC port during MPC interrupt
/*
	 Unteren Bits: Motor Öffnen
	 Oberen Bits:  Motor Schliessen
*/
volatile IOBITS irqSM8Status   = IOBITS_ZERO;
volatile IOBITS irqWallButton  = IOBITS_ZERO;

// values currently used within program
/*
	 Unteren Bits: Motor Öffnen
	 Oberen Bits:  Motor Schliessen
*/
volatile IOBITS curSM8Status   = IOBITS_ZERO;
volatile IOBITS SM8StatusIgnore = IOBITS_ZERO;
volatile IOBITS curWallButton  = IOBITS_ZERO;

// Entprellzähler für SM8-Ausgänge und Wandtaster
volatile char debSM8Status[IOBITS_CNT]  = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
volatile char debWallButton[IOBITS_CNT] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


// Motor Steuerungskommandos:
//   0: Motor AUS
//  >0: Motor Öffnen
//  <0: Motor Schliessen
// abs(Werte) <> 0 haben folgende Bedeutung:
//   1: Motor sofort schalten
//  >1: Motor Delay in (abs(Wert) - 1) * 10 ms
#define MOTOR_OPEN			1
#define MOTOR_DELAYED_OPEN	(MOTOR_SWITCHOVER/TIMER_MS)
#define MOTOR_CLOSE			-1
#define MOTOR_DELAYED_CLOSE	(-MOTOR_SWITCHOVER/TIMER_MS)
#define MOTOR_OFF			0
volatile MOTOR_CTRL    MotorCtrl[MAX_MOTORS]    = {MOTOR_OFF, MOTOR_OFF, MOTOR_OFF, MOTOR_OFF, MOTOR_OFF, MOTOR_OFF, MOTOR_OFF, MOTOR_OFF};
volatile MOTOR_TIMEOUT MotorTimeout[MAX_MOTORS] = {0, 0, 0, 0, 0, 0, 0, 0};

// SM8 Tastensteuerung "gedrückt"-Zeit
volatile SM8_TIMEOUT   SM8Timeout[IOBITS_CNT] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

volatile bool isrTrigger = false;

// EEPROM Variablen
MOTORBITS	eepromMTypeBitmask;
DWORD 		eepromMaxRuntime[MAX_MOTORS];
WORD 		eepromBlinkInterval;
WORD 		eepromBlinkLen;


StreamEx mySerial = Serial; // StreamEx object


// Rain Sensor inputs and vars
Bounce debEnable = Bounce();
Bounce debInput = Bounce();
bool RainDetect = false;


/*	====================================================================
	Function:	 setup
	Return:
	Arguments:
	Description: setup function runs once 
	             when you press reset or power the board
	====================================================================
*/
void setup()
{
	Serial.begin(SERIAL_BAUDRATE);

	printProgramInfo();

#ifdef DEBUG_PINS
	pinMode(DBG_INT, OUTPUT); 		// debugging
	pinMode(DBG_MPC, OUTPUT); 		// debugging
	pinMode(DBG_TIMER, OUTPUT);		// debugging
	pinMode(DBG_TIMERLEN, OUTPUT);	// debugging
#endif

	pinMode(ONBOARD_LED, OUTPUT);   // for onboard LED
	// indicate setup started
	digitalWrite(ONBOARD_LED, HIGH);

	// Input pins pulled-up
	pinMode(RAIN_ENABLE, INPUT_PULLUP);
	// After setting up the button, setup the Bounce instance :
	debEnable.attach(RAIN_ENABLE);
	debEnable.interval(DEBOUNCE_TIME); // interval in ms

	pinMode(RAIN_INPUT, INPUT_PULLUP);
	debInput.attach(RAIN_INPUT);
	debInput.interval(DEBOUNCE_TIME);



#ifdef DEBUG_PINS
	digitalWrite(DBG_INT, LOW);  	// debugging
	digitalWrite(DBG_MPC, LOW);		// debugging
	digitalWrite(DBG_TIMER, LOW);	// debugging
	digitalWrite(DBG_TIMERLEN, LOW);	// debugging
#endif

	Wire.begin();
	// expander configuration register
	expanderWriteBoth(MPC_MOTORRELAIS, IOCON, 0b00100100);	//                    sequential mode, INT Open-drain output
	expanderWriteBoth(MPC_SM8BUTTON,   IOCON, 0b00100100);	//                    sequential mode, INT Open-drain output
	expanderWriteBoth(MPC_SM8STATUS,   IOCON, 0b01100100);	// mirror interrupts, sequential mode, INT Open-drain output
	expanderWriteBoth(MPC_WALLBUTTON,  IOCON, 0b01100100);	// mirror interrupts, sequential mode, INT Open-drain output

	// enable pull-up on switches
	expanderWriteBoth(MPC_MOTORRELAIS, GPPUA, 0xFF);  		// pull-up resistor A/B
	expanderWriteBoth(MPC_SM8BUTTON,   GPPUA, 0xFF);  		// pull-up resistor A/B
	expanderWriteBoth(MPC_SM8STATUS,   GPPUA, 0xFF);  		// pull-up resistor A/B
	expanderWriteBoth(MPC_WALLBUTTON,  GPPUA, 0xFF);  		// pull-up resistor A/B

	// port data
	expanderWriteWord(MPC_MOTORRELAIS, GPIOA, valMotorRelais);
	expanderWriteWord(MPC_SM8BUTTON,   GPIOA, valSM8Button);
	expanderWriteWord(MPC_SM8STATUS,   GPIOA, ~IOBITS_ZERO);
	expanderWriteWord(MPC_WALLBUTTON,  GPIOA, ~IOBITS_ZERO);

	// port direction
	expanderWriteBoth(MPC_MOTORRELAIS, IODIRA, 0x00);		// OUTPUT
	expanderWriteBoth(MPC_SM8BUTTON,   IODIRA, 0x00);		// OUTPUT
	expanderWriteBoth(MPC_SM8STATUS,   IODIRA, 0xFF);		// INPUT
	expanderWriteBoth(MPC_WALLBUTTON,  IODIRA, 0xFF);		// INPUT

	// invert polarity
	expanderWriteBoth(MPC_SM8STATUS,   IOPOLA, 0xFF);		// invert polarity of signal
	expanderWriteBoth(MPC_WALLBUTTON,  IOPOLA, 0xFF);		// invert polarity of signal

	// enable interrupts on input MPC
	expanderWriteBoth(MPC_SM8STATUS,   GPINTENA, 0xFF); 	// enable interrupts
	expanderWriteBoth(MPC_WALLBUTTON,  GPINTENA, 0xFF); 	// enable interrupts

	// read from interrupt capture ports to clear them
	expanderRead(MPC_SM8STATUS,  INTCAPA);
	expanderRead(MPC_SM8STATUS,  INTCAPB);
	expanderRead(MPC_WALLBUTTON, INTCAPA);
	expanderRead(MPC_WALLBUTTON, INTCAPB);

	// pin 19 of MCP23017 is plugged into D2 of the Arduino which is interrupt 0
	pinMode(ISR_INPUT, INPUT);					// make sure input
	digitalWrite(ISR_INPUT, HIGH);				// enable pull-up as we have made the interrupt pins open drain

	// Read EEPROM program variables
	setupEEPROMVars();

	ledCounter = eepromBlinkInterval / eepromBlinkLen;
 	

#ifdef WATCHDOG_ENABLED
	int countdownMS = Watchdog.enable(4000);
#ifdef DEBUG_OUTPUT_WATCHDOG
	printUptime();
	Serial.print(F("Enabled the watchdog with max countdown of "));
	Serial.print(countdownMS, DEC);
	Serial.println(F(" milliseconds!"));
	Serial.println();
#endif
#endif

	setupSerialCommand();

#ifdef DEBUG_OUTPUT_SETUP
	printUptime();
	Serial.println(F("Setup done, starting main loop()"));
#endif

	// indicate setup was done
	digitalWrite(ONBOARD_LED, LOW);

	// External interrupt
	attachInterrupt(digitalPinToInterrupt(ISR_INPUT), extISR, FALLING);

	// Timer2 interrupt
	MsTimer2::set(TIMER_MS, timerISR);
	MsTimer2::start();

}



/*	====================================================================
	Function:	 setupEEPROMVars
	Return:
	Arguments:
	Description: Initalisiere Standard Werte einiger Programmvariablen
			     aus EEPROM-Daten
	====================================================================
*/
void setupEEPROMVars()
{
	int i;

#ifdef DEBUG_OUTPUT_EEPROM
	//Print length of data to run CRC on.
	printUptime();
	Serial.print(F("EEPROM length: "));
	Serial.println(EEPROM.length());
#endif
	// Write data version into EEPROM before checking CRC32
	eepromWriteLong(EEPROM_ADDR_DATAVERSION, DATAVERSION);
	
	unsigned long dataCRC = eepromCalcCRC();
	unsigned long eepromCRC = eepromReadLong(EEPROM_ADDR_CRC32);

#ifdef DEBUG_OUTPUT_EEPROM
	//Print length of data to run CRC on.
	printUptime();
	Serial.print(F("EEPROM length: "));
	Serial.println(EEPROM.length());

	//Print the result of calling eepromCRC()
	printUptime();
	Serial.print(F("CRC32 of EEPROM data: 0x"));
	Serial.println(dataCRC, HEX);

	printUptime();
	Serial.print(F("Stored CRC32: 0x"));
	Serial.println(eepromCRC, HEX);
#endif

	if ( dataCRC != eepromCRC ) {
#ifdef DEBUG_OUTPUT_EEPROM
		printUptime();
		Serial.println(F("EEPROM CRC32 not matching, write defaults..."));
#endif
		eepromWriteLong(EEPROM_ADDR_LED_BLINK_INTERVAL, LED_BLINK_INTERVAL);
		eepromWriteLong(EEPROM_ADDR_LED_BLINK_LEN, 		LED_BLINK_LEN);
		eepromWriteLong(EEPROM_ADDR_MTYPE_BITMASK, 		MTYPE_BITMASK);
		for(i=0; i<MAX_MOTORS; i++) {
			eepromWriteLong(EEPROM_ADDR_MOTOR_MAXRUNTIME+(4*i), 
				bitRead(MTYPE_BITMASK,i)!=0?MOTOR_WINDOW_MAXRUNTIME:MOTOR_JALOUSIE_MAXRUNTIME);
		}
		eepromWriteLong(EEPROM_ADDR_CRC32, eepromCalcCRC());
	}
#ifdef DEBUG_OUTPUT_EEPROM
	else {
		printUptime();
		Serial.println(F("EEPROM CRC232 is valid"));
	}
#endif
#ifdef DEBUG_OUTPUT_EEPROM
	printUptime();
	Serial.println(F("EEPROM read defaults..."));
#endif
	eepromBlinkInterval	= eepromReadLong(EEPROM_ADDR_LED_BLINK_INTERVAL);
	eepromBlinkLen		= eepromReadLong(EEPROM_ADDR_LED_BLINK_LEN);
	eepromMTypeBitmask 	= eepromReadLong(EEPROM_ADDR_MTYPE_BITMASK);
	for(i=0; i<MAX_MOTORS; i++) {
		eepromMaxRuntime[i]	= eepromReadLong(EEPROM_ADDR_MOTOR_MAXRUNTIME+(4*i));
	}
#ifdef DEBUG_OUTPUT_EEPROM
	printUptime();
	Serial.println(F("EEPROM values:"));
	mySerial.printf("  eepromBlinkInterval: %d\n",     eepromBlinkInterval);
	mySerial.printf("  eepromBlinkLen:      %d\n",     eepromBlinkLen);
	mySerial.printf("  eepromMTypeBitmask:  0x%02x\n", eepromMTypeBitmask);
	mySerial.printf("  eepromMaxRuntime:    " );
	for(i=0; i<MAX_MOTORS; i++) {
		mySerial.printf("%s%ld", i?",":"", eepromMaxRuntime[i]);
	}
	mySerial.printf("\n");
#endif
}



/*	====================================================================
	Function:	 extISR
	Return:
	Arguments:
	Description: Interrupt service routine
				 called when external pin D2 goes from 1 to 0
	====================================================================
*/
void extISR()
{
#ifdef DEBUG_PINS
	digitalWrite(DBG_INT, !digitalRead(DBG_INT));  			// debugging
#endif
	isrTrigger = true;
}

/*	====================================================================
	Function:	 timerISR
	Return:
	Arguments:
	Description: Timer Interrupt service routine
				 Wird alle 10 ms aufgerufen
	====================================================================
*/
void timerISR()
{
	byte i;

#ifdef DEBUG_PINS
	digitalWrite(DBG_TIMER, !digitalRead(DBG_TIMER));	// debugging
#endif

#ifdef DEBUG_PINS
	digitalWrite(DBG_TIMERLEN, HIGH);	// debugging
#endif

	for (i = 0; i < IOBITS_CNT; i++) {
		// Tastenentprellung für SM8 Ausgänge
		if ( debSM8Status[i] > 0 ) {
			debSM8Status[i]--;
		}
		else if ( bitRead(curSM8Status, i) != bitRead(irqSM8Status, i) ) {
			bitWrite(curSM8Status, i, bitRead(irqSM8Status, i));
		}

		// Tastenentprellung für Wandtaster
		if ( debWallButton[i] > 0 ) {
			debWallButton[i]--;
		}
		else if ( bitRead(curWallButton, i) != bitRead(irqWallButton, i) ) {
			bitWrite(curWallButton, i, bitRead(irqWallButton, i));
		}

		// SM8 Tastersteuerung Timeout
		if ( SM8Timeout[i] > 0 ) {
			if ( --SM8Timeout[i] == 0 ) {
				// SM8 Taster Timeout abgelaufen, Tasterausgang abschalten
				bitSet(valSM8Button, i);
			}
		}

	}

	// MPC Motor bits steuern (das MPC Register wird außerhalb der ISR geschrieben)
	for (i = 0; i < MAX_MOTORS; i++) {
		// Motor Zeitablauf
		if ( MotorTimeout[i] > 0 ) {
			--MotorTimeout[i];
			if ( MotorTimeout[i] == 0 ) {
				MotorCtrl[i] = MOTOR_OFF;
			}
		}

		// Motor Control
		if ( MotorCtrl[i] > MOTOR_OPEN ) {
			--MotorCtrl[i];
			// Motor auf Öffnen, Motor AUS
			bitSet(valMotorRelais, i + 8);
			bitClear(valMotorRelais, i);
		}
		else if ( MotorCtrl[i] < MOTOR_CLOSE ) {
			++MotorCtrl[i];
			// Motor auf Schliessen, Motor AUS
			bitClear(valMotorRelais, i + 8);
			bitClear(valMotorRelais, i);
		}
		else {
			if ( MotorCtrl[i] == MOTOR_OPEN ) {
				// Motor auf Öffnen, Motor EIN
				bitSet(valMotorRelais, i + 8);
				bitSet(valMotorRelais, i);
			}
			else if ( MotorCtrl[i] == MOTOR_CLOSE ) {
				// Motor auf Schliessen, Motor EIN
				bitClear(valMotorRelais, i + 8);
				bitSet(valMotorRelais, i);
			}
			else if ( MotorCtrl[i] == MOTOR_OFF ) {
				// Motor AUS, Motor auf Schliessen
				bitClear(valMotorRelais, i);
				bitClear(valMotorRelais, i + 8);
			}
		}
	}

#ifdef DEBUG_PINS
	digitalWrite(DBG_TIMERLEN, LOW);	// debugging
#endif

}

/*	====================================================================
	Function:	 handleMPCInt
	Return:
	Arguments:
	Description: MPC Interrupt-Behandlung außerhalb extISR
				 Liest die MPC Register in globale Variablen
				 Aufruf aus loop(), nicht von der ISR
	====================================================================
*/
void handleMPCInt()
{
	byte portValue;
	byte i;

#ifdef DEBUG_PINS
	//digitalWrite(DBG_INT, !digitalRead(DBG_INT));  		// debugging
#endif
	isrTrigger = false;

	if ( expanderReadWord(MPC_SM8STATUS, INFTFA) )
	{
#ifdef DEBUG_PINS
		digitalWrite(DBG_MPC, HIGH);	// debugging
#endif
		irqSM8Status = expanderReadWord(MPC_SM8STATUS, INTCAPA);
		for (i = 0; i < IOBITS_CNT; i++) {
			if ( (curSM8Status & (1 << i)) != (irqSM8Status & (1 << i)) ) {
				debSM8Status[i] = DEBOUNCE_TIME / TIMER_MS;
			}
		}
#ifdef DEBUG_PINS
		digitalWrite(DBG_MPC, LOW);		// debugging
#endif
	}

	if ( expanderReadWord(MPC_WALLBUTTON, INFTFA) )
	{
#ifdef DEBUG_PINS
		digitalWrite(DBG_MPC, HIGH);	// debugging
#endif
		irqWallButton = expanderReadWord(MPC_WALLBUTTON, INTCAPA);
		for (i = 0; i < IOBITS_CNT; i++) {
			if ( (curWallButton & (1 << i)) != (irqWallButton & (1 << i)) ) {
				debWallButton[i] = DEBOUNCE_TIME / TIMER_MS;
			}
		}
#ifdef DEBUG_PINS
		digitalWrite(DBG_MPC, LOW);		// debugging
#endif
	}

#ifdef DEBUG_PINS
	//digitalWrite(DBG_INT, !digitalRead(DBG_INT));  		// debugging
#endif
}



/*	====================================================================
	Function:	 printProgramInfo
	Return:
	Arguments:
	Description: Print program info
	====================================================================
*/
void printProgramInfo(void)
{
	mySerial.printf("%s v%s (build %s)\n", PROGRAM, VERSION, REVISION);
	mySerial.printf("compiled on %s %s (GnuC%s %s)\n", __DATE__, __TIME__, __GNUG__?"++ ":" ", __VERSION__);
	mySerial.printf("(c) 2016 by PR-Solution (http://p-r-solution.de)\n");
	mySerial.printf("Norbert Richter <norbert-richter@p-r-solution.de>\n\n");
}

//#ifdef DEBUG_OUTPUT
/*	====================================================================
	Function:	 printUptime
	Return:
	Arguments:
	Description: Print uptime
	====================================================================
*/
void printUptime(void)
{
	uint32_t t;
	uint16_t days;
	uint8_t hours, minutes, seconds;

	t = millis();
	/* div_t div(int number, int denom) */
	days = (uint16_t)(t / (24L * 60L * 60L * 1000L));
	t -= (uint32_t)days * (24L * 60L * 60L * 1000L);
	hours = (uint8_t)(t / (60L * 60L * 1000L));
	t -= (uint32_t)hours * (60L * 60L * 1000L);
	minutes = (uint8_t)(t / (60L * 1000L));
	t -= (uint32_t)minutes * (60L * 1000L);
	seconds = (uint8_t)(t / 1000L);
	t -= (uint32_t)seconds * 1000L;

	if( days ) {
		mySerial.printf("%d day%s, %02d:%02d:%02d.%03ld ", days, days==1?"":"s", hours, minutes, seconds, t);
	}
	else {
		mySerial.printf("%02d:%02d:%02d.%03ld ", hours, minutes, seconds, t);
	}
}
//#endif



/*	====================================================================
	Function:	 setMotorDirection
	Return:
	Arguments:
	Description: Motor in neue Laufrichtung (oder AUS) schalten
	====================================================================
*/
bool setMotorDirection(byte motorNum, MOTOR_CTRL newDirection)
{
	MOTOR_CTRL currentMotorCtrl = MotorCtrl[motorNum];

	if( motorNum < MAX_MOTORS ) {
		// Neue Richtung: Öffnen
		if ( newDirection >= MOTOR_OPEN ) {
			// Motor läuft auf Schliessen
			if (MotorCtrl[motorNum] <= MOTOR_CLOSE) {
				// Motor auf Öffnen mit Umschaltdelay
				MotorCtrl[motorNum] = MOTOR_DELAYED_OPEN;
			}
			// Motor läuft auf Öffnen
			else if (MotorCtrl[motorNum] >= MOTOR_OPEN) {
				// Motor aus
				MotorCtrl[motorNum] = MOTOR_OFF;
			}
			// Motor ist aus
			else {
				// Motor auf öffnen ohne Umschaltdelay
				MotorCtrl[motorNum] = MOTOR_OPEN;
			}
		}
		// Neue Richtung: Schliessen
		else if ( newDirection <= MOTOR_CLOSE ) {
			// Motor läuft auf Öffnen
			if (MotorCtrl[motorNum] >= MOTOR_OPEN) {
				// Motor auf Schliessen mit Umschaltdelay
				MotorCtrl[motorNum] = MOTOR_DELAYED_CLOSE;
			}
			// Motor läuft auf Schliessen
			else if (MotorCtrl[motorNum] <= MOTOR_CLOSE) {
				// Motor aus
				MotorCtrl[motorNum] = MOTOR_OFF;
			}
			// Motor ist aus
			else {
				// Motor auf Schliessen ohne Umschaltdelay
				MotorCtrl[motorNum] = MOTOR_CLOSE;
			}
		}
		// Neue Richtung: AUS
		else {
			// Motor AUS
			MotorCtrl[motorNum] = MOTOR_OFF;
		}
		return (currentMotorCtrl != MotorCtrl[motorNum]);
	}

	return false;
}



/*	====================================================================
	Function:	 ctrlSM8Status
	Return:
	Arguments:
	Description: Kontrolle der Eingangssignale der SM8
	====================================================================
*/
void ctrlSM8Status(void)
{
	static IOBITS tmpSM8Status   = IOBITS_ZERO;

	/* FS20 SM8 Output */
	static IOBITS SM8Status;
	static IOBITS prevSM8Status = IOBITS_ZERO;
	bool changeMotor;

#ifdef DEBUG_OUTPUT_MOTOR
	changeMotor = false;
#endif

	if ( (tmpSM8Status != curSM8Status) ) {
#ifdef DEBUG_OUTPUT_SM8STATUS
		printUptime();
		Serial.println(F("SM8 Input Status changed"));
#endif
		/* Lese SM8 Ausgänge
			 die unteren 8-bits sind "Öffnen" Befehle
			 die oberen 8-bits sind "Schliessen" Befehle
			 Flankengesteuert
				 Flanke von 0 nach 1 bedeuted: Motor Ein
				 Flanke von 1 nach 0 bedeuted: Motor Aus
				 Gleiche Flanken für Öffnen und Schliessen
					sind ungültig

				Fo Fc  Motor
				-- --  ----------
				0  0   Aus
				1  0   Öffnen
				0  1   Schliessen
				1  1   Ignorieren
		*/
		if ( prevSM8Status != curSM8Status ) {
			byte i;

			IOBITS SM8StatusSlope;
			IOBITS SM8StatusChange;

			SM8Status = curSM8Status;
			SM8StatusSlope   = ~prevSM8Status & SM8Status;
			SM8StatusChange  =  prevSM8Status ^ SM8Status;

#ifdef DEBUG_OUTPUT_SM8STATUS
			Serial.println();
			printUptime();
			Serial.print(F("---------------------------------------- ")); Serial.println((float)millis() / 1000.0, 3);
			mySerial.printf("prevSM8Status:   0x%04x\n", prevSM8Status);
			mySerial.printf("SM8Status:       0x%04x\n", SM8Status);
			mySerial.printf("SM8StatusSlope:  0x%04x\n", SM8StatusSlope);
			mySerial.printf("SM8StatusChange: 0x%04x\n", SM8StatusChange);
			mySerial.printf("SM8StatusIgnore: 0x%04x\n", SM8StatusIgnore);
#endif
			// Eventuell Änderungen einzelner Bits ignorieren
			SM8StatusChange &= ~SM8StatusIgnore;
			SM8StatusIgnore = 0;
#ifdef DEBUG_OUTPUT_SM8STATUS
			mySerial.printf("SM8StatusChange: 0x%04x\n", SM8StatusChange);
#endif

			for (i = 0; i < 8; i++) {
				// Motor Öffnen
				if ( bitRead(SM8StatusChange, i) != 0 && bitRead(SM8StatusChange, i + 8) == 0 ) {
					if ( bitRead(SM8StatusSlope, i) != 0 ) {
						// Flanke von 0 nach 1: Motor Ein
						changeMotor = setMotorDirection(i, MOTOR_OPEN);
						// Taste für Schliessen aktiv?
						if ( bitRead(SM8Status, i + 8) != 0 ) {
							// Taste für Schliessen zurücksetzen
							bitClear(valSM8Button, i + 8);
							bitSet(SM8StatusIgnore, i + 8);
						}
					}
					else {
						// Flanke von 0 nach 1: Motor Aus
						changeMotor = setMotorDirection(i, MOTOR_OFF);
					}
				}
				// Motor Schliessen
				else if ( bitRead(SM8StatusChange, i) == 0 && bitRead(SM8StatusChange, i + 8) != 0 ) {
					if ( bitRead(SM8StatusSlope, i + 8) != 0 ) {
						// Flanke von 0 nach 1: Motor Ein
						changeMotor = setMotorDirection(i, MOTOR_CLOSE);
						// Taste für Öffnen aktiv?
						if ( bitRead(SM8Status, i) != 0 ) {
							// Taste für Öffnen zurücksetzen
							bitClear(valSM8Button, i);
							bitSet(SM8StatusIgnore, i);
						}
					}
					else {
						// Flanke von 0 nach 1: Motor Aus
						changeMotor = setMotorDirection(i, MOTOR_OFF);;
					}
				}
				// Ungültig: Änderungen beider Eingänge zur gleichen Zeit
				else if ( bitRead(SM8StatusChange, i) != 0 && bitRead(SM8StatusChange, i + 8) != 0 ) {
					// Beide Tasten zurücksetzen
					bitClear(valSM8Button, i);
					bitClear(valSM8Button, i + 8);
					bitSet(SM8StatusIgnore, i);
					bitSet(SM8StatusIgnore, i + 8);
				}
			}

			prevSM8Status = curSM8Status;
		}

#ifdef DEBUG_OUTPUT_MOTOR
		if ( changeMotor ) {
			byte i;

			Serial.println();
			printUptime();
			Serial.print(F("---------------------------------------- ")); Serial.println((float)millis() / 1000.0, 3);
			Serial.println(F("   M1   M2   M3   M4   M5   M6   M7   M8"));
			for (i = 0; i < 8; i++) {
				if (MotorCtrl[i] == 0) {
					Serial.print(F("  off"));
				}
				else {
					mySerial.printf("%5d", MotorCtrl[i]);
				}
			}
			Serial.println();
		}
#endif

		tmpSM8Status  = curSM8Status;
	}
}

/*	====================================================================
	Function:	 ctrlWallButton
	Return:
	Arguments:
	Description: Kontrolle der Eingangssignale der Wandtaster
	====================================================================
*/
void ctrlWallButton(void)
{
	static IOBITS tmpWallButton  = IOBITS_ZERO;

	/* Wall Button Output */
	static IOBITS WallButton;
	static IOBITS prevWallButton = IOBITS_ZERO;
	static IOBITS WallButtonLocked = IOBITS_ZERO;
	bool changeMotor;

#ifdef DEBUG_OUTPUT_MOTOR
	changeMotor = false;
#endif

	if ( (tmpWallButton != curWallButton) ) {
		/* Lese Wandtaster
			 die unteren 8-bits sind "Öffnen" Befehle
			 die oberen 8-bits sind "Schliessen" Befehle
			 Flankenänderung von 0 auf 1 schaltet Motor ein/aus
			 Flankenänderung von 1 auf 0 bewirkt nichts
			 Einzelpegel bewirkt nichts
			 Pegel Öffnen und Schliessen = 1:
				- Motor Aus
				- Taster verriegeln
			 Pegel Öffnen und Schliessen = 0:
				- Taster entriegeln
		*/
		if ( prevWallButton != curWallButton ) {
			byte i;

			IOBITS WallButtonSlope;
			IOBITS WallButtonChange;

			WallButton = curWallButton;
			WallButtonSlope = ~prevWallButton & WallButton;
			WallButtonChange = prevWallButton ^ WallButton;

			for (i = 0; i < 8; i++) {
				// Flankenänderung von 0 auf 1 schaltet Motor ein/aus
				if ( bitRead(WallButtonChange, i) != 0 && bitRead(WallButtonSlope, i) != 0 ) {
					changeMotor = setMotorDirection(i, MOTOR_OPEN);
				}
				else if ( bitRead(WallButtonChange, i + 8) != 0 && bitRead(WallButtonSlope, i + 8) != 0 ) {
					changeMotor = setMotorDirection(i, MOTOR_CLOSE);
				}
				// Pegel Öffnen und Schliessen = 1:
				if ( bitRead(WallButton, i) != 0 && bitRead(WallButton, i + 8) != 0 ) {
					changeMotor = setMotorDirection(i, MOTOR_OFF);
					bitSet(WallButtonLocked, i);
					bitSet(WallButtonLocked, i + 8);
				}
				// Pegel Öffnen und Schliessen = 0:
				if ( bitRead(WallButton, i) == 0 && bitRead(WallButton, i + 8) == 0 ) {
					bitClear(WallButtonLocked, i);
					bitClear(WallButtonLocked, i + 8);
				}
			}

#ifdef DEBUG_OUTPUT_WALLBUTTON
			Serial.println();
			printUptime();
			Serial.print(F("---------------------------------------- ")); Serial.println((float)millis() / 1000.0, 3);
			mySerial.printf("prevWallButton:   0x%04x\n", prevWallButton);
			mySerial.printf("WallButton:       0x%04x\n", WallButton);
			mySerial.printf("WallButtonSlope:  0x%04x\n", WallButtonSlope);
			mySerial.printf("WallButtonChange: 0x%04x\n", WallButtonChange);
			mySerial.printf("WallButtonLocked: 0x%04x\n", WallButtonLocked);

#endif

			prevWallButton = curWallButton;
		}

#ifdef DEBUG_OUTPUT_MOTOR
		if ( changeMotor ) {
			byte i;

			Serial.println();
			printUptime();
			Serial.print(F("---------------------------------------- ")); Serial.println((float)millis() / 1000.0, 3);
			Serial.println(F("   M1   M2   M3   M4   M5   M6   M7   M8"));
			for (i = 0; i < 8; i++) {
				if (MotorCtrl[i] == 0) {
					Serial.print(F("  off"));
				}
				else {
					mySerial.printf("%5d", MotorCtrl[i]);
				}
			}
			Serial.println();
		}
#endif

		tmpWallButton = curWallButton;
	}
}

/*	====================================================================
	Function:	 ctrlSM8Button
	Return:
	Arguments:
	Description: Kontrolle der SM8 Tastensteuerung
	====================================================================
*/
void ctrlSM8Button(void)
{
	static IOBITS tmpSM8Button   = IOBITS_ZERO;
	byte i;

	if ( tmpSM8Button != valSM8Button ) {
#ifdef DEBUG_OUTPUT_SM8OUTPUT
		printUptime();
		Serial.println(F("SM8 output changed"));
#endif
		expanderWriteWord(MPC_SM8BUTTON,   GPIOA, valSM8Button);

		for (i = 0; i < IOBITS_CNT; i++) {
			// SM8 Taste Timeout setzen, falls Tastenausgang gerade aktiviert wurde
			if ( (bitRead(tmpSM8Button, i) != bitRead(valSM8Button, i)) && (bitRead(valSM8Button, i) == 0) ) {
#ifdef DEBUG_OUTPUT_SM8OUTPUT
				printUptime();
				Serial.print(F("Set SM8 key "));
				Serial.print(i + 1);
				Serial.print(F(" timeout to "));
				Serial.print(FS20_SM8_IN_RESPONSE / TIMER_MS);
				Serial.println(F(" ms"));
#endif
				SM8Timeout[i] = FS20_SM8_IN_RESPONSE / TIMER_MS;
			}
		}

		tmpSM8Button = valSM8Button;
	}
}

/*	====================================================================
	Function:	 ctrlMotorRelais
	Return:
	Arguments:
	Description: Kontrolle der Motor Ausgangssignale
	====================================================================
*/
void ctrlMotorRelais(void)
{
	static IOBITS tmpMotorRelais = IOBITS_ZERO;
	byte i;

	if ( tmpMotorRelais != valMotorRelais ) {
#ifdef DEBUG_OUTPUT_MOTOR
		printUptime();
		Serial.println(F("Motor output changed"));
#endif
		expanderWriteWord(MPC_MOTORRELAIS, GPIOA, valMotorRelais);
		for (i = 0; i < MAX_MOTORS; i++) {
			// Motor Timeout setzen, falls Motor
			// gerade aktiviert oder die Laufrichtung geändert wurde
			if (    (bitRead(tmpMotorRelais, i  ) == 0 && bitRead(valMotorRelais, i  ) != 0)
					|| (bitRead(tmpMotorRelais, i + 8) != bitRead(valMotorRelais, i + 8)           ) ) {
#ifdef DEBUG_OUTPUT_MOTOR
				printUptime();
				Serial.print(F("Set motor "));
				Serial.print(i + 1);
				Serial.print(F(" timeout to "));
				Serial.print(eepromMaxRuntime[i] / 1000);
				Serial.println(F(" sec."));
#endif
				MotorTimeout[i] = eepromMaxRuntime[i] / TIMER_MS;
			}
			// SM8 Ausgang für "Öffnen" aktiv, aber Motor ist AUS bzw. arbeitet auf "Schliessen"
			if ( bitRead(curSM8Status, i) != 0
				 && (bitRead(valMotorRelais, i) == 0
					 || (bitRead(valMotorRelais, i) != 0 && bitRead(valMotorRelais, i + 8) == 0) ) ) {
				// SM8 Taste für "Öffnen" zurücksetzen
				bitClear( valSM8Button, i);
				bitSet(SM8StatusIgnore, i);
			}

			// SM8 Ausgang für "Schliessen" aktiv, aber Motor ist AUS bzw. arbeitet auf "Öffnen"
			if ( bitRead(curSM8Status, i + 8) != 0
				 && (bitRead(valMotorRelais, i) == 0
					 || (bitRead(valMotorRelais, i) != 0 && bitRead(valMotorRelais, i + 8) != 0) ) ) {
				// SM8 Taste für "Schliessen" zurücksetzen
				bitClear(valSM8Button, i + 8);
				bitSet(SM8StatusIgnore, i + 8);
			}
		}

		tmpMotorRelais = valMotorRelais;
	}
}

/*	====================================================================
	Function:	 ctrlRainSensor
	Return:
	Arguments:
	Description: Kontrolle der Regensensor Eingänge
	====================================================================
*/
void ctrlRainSensor(void)
{
	bool RainInput;
	bool RainEnable;
	static bool prevRainInput = false;
	static bool prevRainEnable = false;


	// Debouncing inputs
	debEnable.update();
	debInput.update();

	// Get the updated value :
	RainEnable = debEnable.read() == RAIN_ENABLE_AKTIV;
	RainInput  = debInput.read() == RAIN_INPUT_AKTIV;

	if ( prevRainInput != RainInput || prevRainEnable != RainEnable) {
#ifdef DEBUG_OUTPUT_RAIN
		Serial.print(F("RainEnable: ")); Serial.println(RainEnable);
		Serial.print(F("RainInput:  ")); Serial.println(RainInput);
#endif
		if ( RainEnable ) {
#ifdef DEBUG_OUTPUT_RAIN
			printUptime();
			Serial.println(F("Rain inputs changed, sensor enabled"));
#endif
			if ( RainInput ) {
#ifdef DEBUG_OUTPUT_RAIN
				printUptime();
				Serial.println(F("Rain active, close all windows"));
#endif
				byte i;

				for (i = 0; i < MAX_MOTORS; i++) {
					if ( bitRead(eepromMTypeBitmask, i) ) {
#ifdef DEBUG_OUTPUT_RAIN
						Serial.print(F("Close window "));
						Serial.println(i);
#endif
						setMotorDirection(i, MOTOR_CLOSE);
					}
				}
				RainDetect = true;
				digitalWrite(ONBOARD_LED, HIGH);
			}
			else {
#ifdef DEBUG_OUTPUT_RAIN
				printUptime();
				Serial.println(F("Rain inactive, do nothing"));
#endif
				digitalWrite(ONBOARD_LED, LOW);
				RainDetect = false;
			}
		}
		else {
#ifdef DEBUG_OUTPUT_RAIN
			printUptime();
			Serial.println(F("Rain inputs changed, sensor disabled"));
#endif
			digitalWrite(ONBOARD_LED, LOW);
			RainDetect = false;
		}
		prevRainEnable = RainEnable;
		prevRainInput = RainInput;
	}
}


/*	====================================================================
	Function:	 beAlive()
	Return:
	Arguments:
	Description: Lebenszeichen (watchdog bedienen, debug output)
	====================================================================
*/
void beAlive(void)
{
#ifdef DEBUG_OUTPUT_LIVE
	static char liveToogle = 0;
	static byte liveDots = 0;
#endif

	// Live timer und watchdog handling
	if ( millis() > (runTimer + 500) )
	{
		runTimer = millis();
#ifdef WATCHDOG_ENABLED
		Watchdog.reset();
#endif
#ifdef DEBUG_OUTPUT_LIVE
		if ((liveToogle--) < 1) {
			Serial.print(F("."));
			liveToogle = 3;
			liveDots++;
			if ( liveDots > 76 ) {
				Serial.println();
				liveDots = 0;
			}
		}
#endif
	}
}


/*	====================================================================
	Function:	 blinkLED()
	Return:
	Arguments:
	Description: Onboard LED Blink-Funktion
	====================================================================
*/
void blinkLED(void) 
{
	// toggle LED to indicate main loop is running
	if ( millis() > (ledTimer + eepromBlinkLen) )
	{
		if ( --ledCounter == 1 ) {
			digitalWrite(ONBOARD_LED, RainDetect?LOW:HIGH);
		}
		else if ( --ledCounter < 1 ) {
			digitalWrite(ONBOARD_LED, RainDetect?HIGH:LOW);
			ledCounter = eepromBlinkInterval / eepromBlinkLen;
		}
		ledTimer = millis();
	}
}


/*	====================================================================
	Function:	 loop()
	Return:
	Arguments:
	Description: the loop function runs over and over again forever
	====================================================================
*/
void loop()
{
	// MPC Interrupt aufgetreten?
	if ( isrTrigger ) {
		// MPC Interrupt-Behandlung außerhalb extISR
		handleMPCInt();
	}

	// Kontrolle der Eingangssignale von SM8 und Wandtaster
	ctrlSM8Status();
	/* Wandtaster haben Vorrang vor SM8 Ausgänge, daher Auslesen nach SM8 */
	ctrlWallButton();
	// Kontrolle der SM8 Tastensteuerung
	ctrlSM8Button();
	// Kontrolle der Motor Ausgangssignale
	ctrlMotorRelais();
	// Regensensor abfragen
	ctrlRainSensor();

	beAlive();
	
	blinkLED();
	
	processSerialCommand();
}
