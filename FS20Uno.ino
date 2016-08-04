/* ===================================================================
 * File:    FS20Uno.ino
 * Author:  Norbert Richter <mail@norbert-richter.info>
 * Project: Dachflächenfenster/-Jalousie Steuerung
 * 	        mit 2x FS20 SM8
 * Desc:    Die Steuerung besteht aus
 *          - einem Arduino Uno
 *          - vier Porterweiterungen MCP23017
 *          - zwei ELV FS20 SM8
 *          - Der Uno steuert über 16 Relais ingesamt acht Motoren
 *            - 8 Relais für Motor Ein/Aus
 *          - 8 Relais für die Drehrichtung.
 *
 * Funktionsweise der Steuerung:
 * Zwei FS20 SM8 dienen als FS20 Empfänger für Dachflächenfenster-
 * steuerung (DFF)
 * Mit Hilfer zweier FS20-SM8 stehen stehen insgesamt 16 FS20 Kanäle
 * zur Verfügung.
 * Jeweils zwei steuern einen Motor:
 * - ein Kanal steuert Linkslauf (z. B. Öffnen)
 * - ein Kanal steuert Rechtslauf (z. B. Schliessen)
 *
 * Damit lassen sich insgesamt 4 DF-Fenster mit jeweils zwei 24V
 * Motoren ansteuern:
 * - ein Motor zum Öffnen/Schliessen des Fensters
 * - ein Motor für die Jalousie
 * Der Uno übernimmt dabei die 'Übersetzung' von Öffnen/Schliessen
 * auf die Motorzustände
 * - Aus
 * - Öffnen
 * - Schließen
 *
 * Für jeden Motor wird bestimmt:
 * - Fenster oder Jalousie (wichtig für Regensensor)
 * - Max. Motorlaufzeit
 *
 * Da wir in den Ablauf des FS20 SM8 nur bedingt eingreifen können,
 * werden dieFS20 SM8 nur als 'Geber' eingesetzt. Die Motorsteuerung
 * erfolgt komplett über dieses Steuerungsprogramm.
 *
 * Verdrahtung:
 * MPC1
 *   Port A (Output): Relais Motor 1-8 EIN/AUS
 *   Port B (Output): Relais Motor 1-8 Drehrichtung
 * MPC2
 *   Port A (Output): FS20-SM8 #1 Taster 1-8 ("Auf" Funktion)
 *   Port B (Output): FS20-SM8 #2 Taster 1-8 ("Zu" Funktion)
 * MPC3
 *   Port A (Input) : FS20-SM8 #1 Status 1-8 ("Auf" Funktion)
 *   Port B (Input) : FS20-SM8 #2 Status 1-8 ("Zu" Funktion)
 *   Die FS20-SM8 Ausgänge schalten das Signal gegen Masse (0=Aktiv)
 * MPC4
 *   Port A (Input) : Wandtaster             ("Auf" Funktion)
 *   Port B (Input) : Wandtaster             ("Zu" Funktion)
 *
 * Zwei Digitaleingänge des Uno werden für Regensensoraktivitäten 
 * verwendet.
 *
 * Motorsteuerung:
 * ----------------------------
 * Die Motoren werden vollständig über den Uno gesteuert, keine der
 * Tasten oder FS20-SM8 Ausgänge gehen an die Motoren.
 * Jeder Motor kennt zwei zusätzliche Eigenschaften (TYPE):
 * - Motortyp: "Fenster" (WINDOW) oder "Jalousie" (JALOUSIE)
 * - Motorlaufzeit: Maximale Laufzeit des Motors
 * Diese Eigenschaften lassen sich mit Hilfe der Standardwerte
 * MTYPE_BITMASK, MOTOR_WINDOW_MAXRUNTIME und MOTOR_JALOUSIE_MAXRUNTIME
 * vordefinieren, als auch später mit Hilfe der Control-Kommandos
 * "MOTORTYPE" und "MOTORTIME" verändern.
 * Motoren vom Typ Fenster werden bei aktivem Regensensoreingang
 * automatisch geschlossen.
 * Jeder Motor wird nach Ablauf seiner Motorlaufzeit abgeschaltet.
 *   
 * Motorschutz:
 * - Das Einschalten der Motorspannung erfolgt erst, nachdem das
 *   Richtungsrelais umgeschalten hat (OPERATE_TIME).
 * - Ebenso wird auch unter Berücksichtigung der Relais
 *   Ansprechzeiten (OPERATE_TIME) bei laufendem Motor und
 *   Richtungsumkehr der Motor zuerst abgeschaltet, die Laufrichtung
 *   geändert und danach der Motor wieder eingeschaltet
 *   (MOTOR_SWITCHOVER).
 *
 * Wandtaster:
 * ----------------------------
 * Die Taster schalten das Eingangssignal gegen Masse (0=Aktiv).
 * Die Wandtaster haben folgende Funktion:
 * - Taste Auf: "Auf" einschalten,
 *              nochmaliger Druck schaltet Motor ab.
 * - Taste Zu : "Zu" einschalten, 
 *              nochmaliger Druck schaltet Motor ab.
 * - Beide Tasten: Schaltet Motor ab.
 * Wandtaster haben Vorrang vor SM8 Steuerungseingängen. Eventuell
 * aktive SM8 Kanäle werden bei Betätigung des zugehörigen Wandtasters
 * angeschaltet.
 *
 * FS20-SM8:
 * ----------------------------
 * Vom SM8 werden jeweils die Kanalausgänge als Steuereingang für die
 * Motorsteuerung herangezogen. Die SM8-Kanaleingänge können mit Hilfe
 * eines weiteren MPC vom Uno gesteuert werden. Dadurch ist es
 * möglich, aktive SM8 Kanäle, die z. B. per Funk aktiviert wurden,
 * umzuschalten.
 * Folgende Bedingungen werden berücksichtigt:
 * - gleichzeitig aktivierte SM8 Ausgänge (z.b. AUF & ZU aktiv)
 *   werden entkoppelt, der zuletzt gegeben Befehl wird aktiviert,
 *   der zugehörige zu entkopplende Kanal deaktiviert.
 * - nur die steigende Flanke (0->1) eines Kanals aktiviert die
 *   zugehörige Funktion.
 * - die fallende Flanke (1->0) eines SM8-Kanals schaltet die
 *   jeweilige Funktion ab.
 * - ein Motor, der durch Wandtaster oder Timeout abgeschaltet wird,
 *   schaltet auch den zugehörige SM8-Kanal ab.
 *
 * Regensensor
 * ----------------------------
 * Regensensoreingänge:
 * Regensensoreingänge werden nur im Automatikmodus
 * (s.u. Control-Kommando "RAINSENSOR") abgefragt. Die Verwendung von
 * "RAINSENSOR" Kommandos schaltet Abfrage der Hardware-Eingänge ab.
 * Durch Verwendung des Control-Kommandos RAINSENSOR AUTO läßt sich
 * diese Automatik wieder einschalten.
 * 
 * Ein Regensensor kann am Eingang RAIN_INPUT angeschlossen werden.
 * Die Pegelaktivität kann mit Hilfe der Konstanten RAIN_INPUT_AKTIV 
 * festgelegt werden.
 * Ob der Regensensoreingang beachtet wird, läßt sich mit Hilfe eines
 * zweiten Steuereingangs RAIN_ENABLE (Pegelaktivität über die 
 * Konstante RAIN_ENABLE_AKTIV einstellbar) festlegen. Nur wenn
 * RAIN_ENABLE aktiv ist, wird der Regensensoreingang RAIN_INPUT
 * abgefragt.
 * Meldet der Regensensor RAIN_INPUT regen (Flanke Inaktiv->Aktiv)
 * Werden alle Motoren vom Typ "Fenster" (siehe Motorensteuerung)
 * geschlossen.
 *
 * Control-Kommando "RAINSENSOR"
 * Die beiden Eingänge lassen sich über Control-Kommandos "RAINSENSOR"
 * übersteuern:
 * - RAINSENSOR ON|OFF
 *     Simuliert den Regensensoreingang:  ON=Aktiv
 *                                        OFF=Inaktiv
 * - RAINSENSOR ENABLE|DISABLE
 *     Simuliert die Regensensor-Abfrage: ENABLE=Abfrage aktiv,
 *                                        DISABLE=Abfrage inaktiv
 * - RAINSENSOR AUTO
 *     Da bei Verwendung eines der o.a. Befehle der Automatikmodus
 *     (d.h. die Abfrage der Regensensor-Hardwareeingänge) abgeschaltet
 *     wird, kann hiermit die Abfrage der Regensensor-Hardwareeingänge
 *     wieder aktiviert werden. Die eingestellten Simulationswerte
 *     haben, solange sie nicht wieder verwendet werden,
 *     keine Bedeutung mehr
 * ===================================================================*/

/* ===================================================================
/* TODO
 * SM8 direkte Drehrichtungsumschaltung funktoniert nicht
 * Regensensor "Regen" nur bei Flanke Inaktiv->Aktiv auslösen
 * Eingangsänderung RAIN_ENABLE sollte Automatik wieder einschalten
 * ===================================================================*/

#include <Arduino.h>
#include <EEPROM.h>				// https://www.arduino.cc/en/Reference/EEPROM
#include <Wire.h>				// https://www.arduino.cc/en/Reference/Wire
#include <MsTimer2.h>			// http://playground.arduino.cc/Main/MsTimer2
#include <Bounce2.h>			// https://github.com/thomasfredericks/Bounce2
#include <Adafruit_SleepyDog.h> // https://github.com/adafruit/Adafruit_SleepyDog
#include <SerialCommand.h>		// https://github.com/scogswell/ArduinoSerialCommand
//#include <PrintEx.h>			// https://github.com/Chris--A/PrintEx#printex-library-for-arduino-
 
// Eigene includes
#include "FS20Uno.h"
#include "I2C.h"

#define PROGRAM "FS20Uno"
#define VERSION "2.13"
#include "REVISION.h"
#define DATAVERSION 106


// define next macros to output debug prints
#define DEBUG_OUTPUT
#undef DEBUG_PINS				// enable debug output pins
#undef DEBUG_OUTPUT_SETUP		// enable setup related outputs
#undef DEBUG_OUTPUT_WATCHDOG	// enable watchdog related outputs
#undef DEBUG_OUTPUT_EEPROM		// enable EEPROM related outputs
#define DEBUG_OUTPUT_SM8STATUS	// enable FS20-SM8-output related output
#undef DEBUG_OUTPUT_WALLBUTTON	// enable wall button related output
#undef DEBUG_OUTPUT_SM8OUTPUT	// enable FS20-SM8-key related output
#define DEBUG_OUTPUT_MOTOR		// enable motor control related output
#undef DEBUG_OUTPUT_RAIN		// enable rain sensor related output
#undef DEBUG_OUTPUT_ALIVE		// enable program alive signal output

#ifndef DEBUG_OUTPUT
	// enable next line to enable watchdog timer
	#define WATCHDOG_ENABLED
	#undef DEBUG_PINS
	#undef DEBUG_OUTPUT_SETUP
	#undef DEBUG_OUTPUT_WATCHDOG
	#undef DEBUG_OUTPUT_EEPROM
	#undef DEBUG_OUTPUT_SM8STATUS
	#undef DEBUG_OUTPUT_WALLBUTTON
	#undef DEBUG_OUTPUT_SM8OUTPUT
	#undef DEBUG_OUTPUT_MOTOR
	#undef DEBUG_OUTPUT_RAIN
	#undef DEBUG_OUTPUT_ALIVE
#endif
#ifdef DEBUG_PINS
	#define DBG_INT 			12			// Debug PIN = D12
	#define DBG_MPC 			11			// Debug PIN = D11
	#define DBG_TIMER	 		10			// Debug PIN = D10
	#define DBG_TIMERLEN 		9			// Debug PIN = D9
#endif

// loop() timer vars
TIMER ledTimer = 0;
WORD  ledDelay = 0;
bool  ledStatus = false;

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

// Speichert die beiden RAIN-Bits
#define RAIN_BIT_AUTO	0
#define RAIN_BIT_ENABLE	1
volatile byte eepromRain;

// Software Regendetection (wird nicht in EEPROM gespeichert)
volatile bool softRainInput = false;

// Sende autom. Statusänderung
volatile bool eepromSendStatus = true;


// Rain Sensor inputs and vars
Bounce debEnable = Bounce();
Bounce debInput = Bounce();
volatile bool RainDetect = false;



/* ===================================================================
 * Function:    setup
 * Return:
 * Arguments:
 * Description: setup function runs once
 *              when you press reset or power the board
 * ===================================================================*/
void setup()
{
	Serial.begin(SERIAL_BAUDRATE);

	printProgramInfo(true);

	#ifdef DEBUG_OUTPUT
	SerialTimePrintf(F("Debug output enabled\r\n"));
	#endif

	#ifdef DEBUG_PINS
	SerialTimePrintf(F("Debug pins enabled\r\n"));
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

	#ifdef WATCHDOG_ENABLED
	int countdownMS = Watchdog.enable(4000);
	#ifdef DEBUG_OUTPUT_WATCHDOG
	SerialTimePrintf(F("Enabled the watchdog with max countdown of %d ms\r\n"), countdownMS);
	#endif
	#endif

	setupSerialCommand();

#ifdef DEBUG_OUTPUT_SETUP
	SerialTimePrintf(F("Setup done, starting main loop()\r\n"));
#endif

	// indicate setup was done
	digitalWrite(ONBOARD_LED, LOW);

	// External interrupt
	attachInterrupt(digitalPinToInterrupt(ISR_INPUT), extISR, FALLING);

	// Timer2 interrupt
	MsTimer2::set(TIMER_MS, timerISR);
	MsTimer2::start();

}





/* ===================================================================
 * Function:    extISR
 * Return:
 * Arguments:
 * Description: Interrupt service routine
 *              called when external pin D2 goes from 1 to 0
 * ===================================================================*/
void extISR()
{
	#ifdef DEBUG_PINS
	digitalWrite(DBG_INT, !digitalRead(DBG_INT));  			// debugging
	#endif
	isrTrigger = true;
}

/* ===================================================================
 * Function:    timerISR
 * Return:
 * Arguments:
 * Description: Timer Interrupt service routine
 *              Wird alle 10 ms aufgerufen
 * ===================================================================*/
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
			bitSet(valMotorRelais, i + MAX_MOTORS);
			bitClear(valMotorRelais, i);
		}
		else if ( MotorCtrl[i] < MOTOR_CLOSE ) {
			++MotorCtrl[i];
			// Motor auf Schliessen, Motor AUS
			bitClear(valMotorRelais, i + MAX_MOTORS);
			bitClear(valMotorRelais, i);
		}
		else {
			if ( MotorCtrl[i] == MOTOR_OPEN ) {
				// Motor auf Öffnen, Motor EIN
				bitSet(valMotorRelais, i + MAX_MOTORS);
				bitSet(valMotorRelais, i);
			}
			else if ( MotorCtrl[i] == MOTOR_CLOSE ) {
				// Motor auf Schliessen, Motor EIN
				bitClear(valMotorRelais, i + MAX_MOTORS);
				bitSet(valMotorRelais, i);
			}
			else if ( MotorCtrl[i] == MOTOR_OFF ) {
				// Motor AUS, Motor auf Schliessen
				bitClear(valMotorRelais, i);
				bitClear(valMotorRelais, i + MAX_MOTORS);
			}
		}
	}

	#ifdef DEBUG_PINS
	digitalWrite(DBG_TIMERLEN, LOW);	// debugging
	#endif

}

/* ===================================================================
 * Function:    handleMPCInt
 * Return:
 * Arguments:
 * Description: MPC Interrupt-Behandlung außerhalb extISR
 *              Liest die MPC Register in globale Variablen
 *              Aufruf aus loop(), nicht von der ISR
 * ===================================================================*/
void handleMPCInt()
{
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



/* ===================================================================
 * Function:    setMotorDirection
 * Return:
 * Arguments:
 * Description: Motor in neue Laufrichtung (oder AUS) schalten
 * ===================================================================*/
bool setMotorDirection(byte motorNum, MOTOR_CTRL newDirection)
{
	MOTOR_CTRL currentMotorCtrl = MotorCtrl[motorNum];
	char strBuffer[80];

	if ( motorNum < MAX_MOTORS ) {
		// Neue Richtung: Öffnen
		if ( newDirection >= MOTOR_OPEN ) {
			// Motor läuft auf Schliessen
			if (MotorCtrl[motorNum] <= MOTOR_CLOSE) {
				// Motor auf Öffnen mit Umschaltdelay
				MotorCtrl[motorNum] = MOTOR_DELAYED_OPEN;
				sendStatus(F("01 M%i OPENING DELAYED"), motorNum);
			}
			// Motor läuft auf Öffnen
			else if (MotorCtrl[motorNum] >= MOTOR_OPEN) {
				// Motor aus
				MotorCtrl[motorNum] = MOTOR_OFF;
				sendStatus(F("01 M%i OFF"), motorNum);
			}
			// Motor ist aus
			else {
				// Motor auf öffnen ohne Umschaltdelay
				MotorCtrl[motorNum] = MOTOR_OPEN;
				sendStatus(F("01 M%i OPENING"), motorNum);
			}
		}
		// Neue Richtung: Schliessen
		else if ( newDirection <= MOTOR_CLOSE ) {
			// Motor läuft auf Öffnen
			if (MotorCtrl[motorNum] >= MOTOR_OPEN) {
				// Motor auf Schliessen mit Umschaltdelay
				MotorCtrl[motorNum] = MOTOR_DELAYED_CLOSE;
				sendStatus(F("01 M%i CLOSING DELAYED"), motorNum);
			}
			// Motor läuft auf Schliessen
			else if (MotorCtrl[motorNum] <= MOTOR_CLOSE) {
				// Motor aus
				MotorCtrl[motorNum] = MOTOR_OFF;
				sendStatus(F("01 M%i OFF"), motorNum);
			}
			// Motor ist aus
			else {
				// Motor auf Schliessen ohne Umschaltdelay
				MotorCtrl[motorNum] = MOTOR_CLOSE;
				sendStatus(F("01 M%i CLOSING"), motorNum);
			}
		}
		// Neue Richtung: AUS
		else {
			// Motor AUS
			MotorCtrl[motorNum] = MOTOR_OFF;
			sendStatus(F("01 M%i OFF"), motorNum);
		}
		return (currentMotorCtrl != MotorCtrl[motorNum]);
	}

	return false;
}

/* ===================================================================
 * Function:    getMotorDirection
 * Return:		Laufrichtung (MOTOR_OPEN, MOTOR_CLOSE, MOTOR_OFF)
 * Arguments:	motorNum - die Motorennummer [0..x]
 * Description: Motor Laufrichtung zurückgeben
 *              MOTOR_OPEN, MOTOR_CLOSE oder MOTOR_OFF
 * ===================================================================*/
char getMotorDirection(byte motorNum)
{
	if (MotorCtrl[motorNum] >= MOTOR_OPEN) {
		return MOTOR_OPEN;
	}
	else if (MotorCtrl[motorNum] <= MOTOR_CLOSE) {
		return MOTOR_CLOSE;
	}
	return MOTOR_OFF;
}


/* ===================================================================
 * Function:    ctrlSM8Status
 * Return:
 * Arguments:
 * Description: Kontrolle der Eingangssignale der SM8
 * ===================================================================*/
void ctrlSM8Status(void)
{
	static IOBITS tmpSM8Status   = IOBITS_ZERO;

	/* FS20 SM8 Output */
	static IOBITS SM8Status;
	static IOBITS prevSM8Status = IOBITS_ZERO;
	bool changeMotor;
	char strBuffer[80];

	#ifdef DEBUG_OUTPUT_MOTOR
	changeMotor = false;
	#endif

	if ( (tmpSM8Status != curSM8Status) ) {
		#ifdef DEBUG_OUTPUT_SM8STATUS
		SerialTimePrintf(F("SM8 Input Status changed\r\n"));
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
			SerialTimePrintf(F("----------------------------------------\r\n"));
			SerialTimePrintf(F("prevSM8Status:   0x%04x\r\n"), prevSM8Status);
			SerialTimePrintf(F("SM8Status:       0x%04x\r\n"), SM8Status);
			SerialTimePrintf(F("SM8StatusSlope:  0x%04x\r\n"), SM8StatusSlope);
			SerialTimePrintf(F("SM8StatusChange: 0x%04x\r\n"), SM8StatusChange);
			SerialTimePrintf(F("SM8StatusIgnore: 0x%04x\r\n"), SM8StatusIgnore);
			#endif
			// Eventuell Änderungen einzelner Bits ignorieren
			SM8StatusChange &= ~SM8StatusIgnore;
			SM8StatusIgnore = 0;
			#ifdef DEBUG_OUTPUT_SM8STATUS
			SerialTimePrintf(F("SM8StatusChange: 0x%04x\r\n"), SM8StatusChange);
			#endif

			for (i = 0; i < IOBITS_CNT; i++) {
				if ( bitRead(SM8StatusChange, i) ) {
					sendStatus(F("02 FS20 OUTPUT %2d %s"), i, bitRead(curSM8Status,i)?"ON":"OFF");
				}
			}
			for (i = 0; i < MAX_MOTORS; i++) {
				// Motor Öffnen
				if ( bitRead(SM8StatusChange, i) != 0 && bitRead(SM8StatusChange, i + MAX_MOTORS) == 0 ) {
					if ( bitRead(SM8StatusSlope, i) != 0 ) {
						// Flanke von 0 nach 1: Motor Ein
						changeMotor = setMotorDirection(i, MOTOR_OPEN);
						// Taste für Schliessen aktiv?
						if ( bitRead(SM8Status, i + MAX_MOTORS) != 0 ) {
							// Taste für Schliessen zurücksetzen
							bitClear(valSM8Button, i + MAX_MOTORS);
							bitSet(SM8StatusIgnore, i + MAX_MOTORS);
						}
					}
					else {
						// Flanke von 0 nach 1: Motor Aus
						changeMotor = setMotorDirection(i, MOTOR_OFF);
					}
				}
				// Motor Schliessen
				else if ( bitRead(SM8StatusChange, i) == 0 && bitRead(SM8StatusChange, i + MAX_MOTORS) != 0 ) {
					if ( bitRead(SM8StatusSlope, i + MAX_MOTORS) != 0 ) {
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
				else if ( bitRead(SM8StatusChange, i) != 0 && bitRead(SM8StatusChange, i + MAX_MOTORS) != 0 ) {
					// Beide Tasten zurücksetzen
					bitClear(valSM8Button, i);
					bitClear(valSM8Button, i + MAX_MOTORS);
					bitSet(SM8StatusIgnore, i);
					bitSet(SM8StatusIgnore, i + MAX_MOTORS);
				}
			}

			prevSM8Status = curSM8Status;
		}

		#ifdef DEBUG_OUTPUT_MOTOR
		if ( changeMotor ) {
			byte i;

			SerialTimePrintf(F("----------------------------------------\r\n"));
			SerialTimePrintf(F("   M1   M2   M3   M4   M5   M6   M7   M8\r\n"));
			SerialTimePrintf(F(""));
			for (i = 0; i < MAX_MOTORS; i++) {
				if (MotorCtrl[i] == 0) {
					SerialPrintf(F("  off"));
				}
				else {
					SerialPrintf(F("%5d"), MotorCtrl[i]);
				}
			}
			SerialPrintf(F("\r\n"));
		}
		#endif

		tmpSM8Status  = curSM8Status;
	}
}

/* ===================================================================
 * Function:    ctrlWallButton
 * Return:
 * Arguments:
 * Description: Kontrolle der Eingangssignale der Wandtaster
 * ===================================================================*/
void ctrlWallButton(void)
{
	static IOBITS tmpWallButton  = IOBITS_ZERO;

	/* Wall Button Output */
	static IOBITS WallButton;
	static IOBITS prevWallButton = IOBITS_ZERO;
	static IOBITS WallButtonLocked = IOBITS_ZERO;
	bool changeMotor;
	char strBuffer[80];

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

			for (i = 0; i < IOBITS_CNT; i++) {
				if ( bitRead(WallButtonChange, i) ) {
					sendStatus(F("04 KEY %2d %s"), i, bitRead(curWallButton,i)?"ON":"OFF");
				}
			}
			for (i = 0; i < MAX_MOTORS; i++) {
				// Flankenänderung von 0 auf 1 schaltet Motor ein/aus
				if ( bitRead(WallButtonChange, i) != 0 && bitRead(WallButtonSlope, i) != 0 ) {
					changeMotor = setMotorDirection(i, MOTOR_OPEN);
				}
				else if ( bitRead(WallButtonChange, i + MAX_MOTORS) != 0 && bitRead(WallButtonSlope, i + MAX_MOTORS) != 0 ) {
					changeMotor = setMotorDirection(i, MOTOR_CLOSE);
				}
				// Pegel Öffnen und Schliessen = 1:
				if ( bitRead(WallButton, i) != 0 && bitRead(WallButton, i + MAX_MOTORS) != 0 ) {
					changeMotor = setMotorDirection(i, MOTOR_OFF);
					bitSet(WallButtonLocked, i);
					bitSet(WallButtonLocked, i + MAX_MOTORS);
				}
				// Pegel Öffnen und Schliessen = 0:
				if ( bitRead(WallButton, i) == 0 && bitRead(WallButton, i + MAX_MOTORS) == 0 ) {
					bitClear(WallButtonLocked, i);
					bitClear(WallButtonLocked, i + MAX_MOTORS);
				}
			}

			#ifdef DEBUG_OUTPUT_WALLBUTTON
			SerialTimePrintf(F("----------------------------------------\r\n"));
			SerialTimePrintf(F("prevWallButton:   0x%04x\r\n"), prevWallButton);
			SerialTimePrintf(F("WallButton:       0x%04x\r\n"), WallButton);
			SerialTimePrintf(F("WallButtonSlope:  0x%04x\r\n"), WallButtonSlope);
			SerialTimePrintf(F("WallButtonChange: 0x%04x\r\n"), WallButtonChange);
			SerialTimePrintf(F("WallButtonLocked: 0x%04x\r\n"), WallButtonLocked);
			#endif

			prevWallButton = curWallButton;
		}

		#ifdef DEBUG_OUTPUT_MOTOR
		if ( changeMotor ) {
			byte i;

			SerialTimePrintf(F("----------------------------------------\r\n"));
			SerialTimePrintf(F("   M1   M2   M3   M4   M5   M6   M7   M8\r\n"));
			SerialTimePrintf(F(""));
			for (i = 0; i < MAX_MOTORS; i++) {
				if (MotorCtrl[i] == 0) {
					SerialPrintf(F("  off"));
				}
				else {
					SerialPrintf(F("%5d"), MotorCtrl[i]);
				}
			}
			SerialPrintf(F("\r\n"));
		}
		#endif

		tmpWallButton = curWallButton;
	}
}

/* ===================================================================
 * Function:    ctrlSM8Button
 * Return:
 * Arguments:
 * Description: Kontrolle der SM8 Tastensteuerung
 * ===================================================================*/
void ctrlSM8Button(void)
{
	static IOBITS tmpSM8Button   = ~IOBITS_ZERO;
	char strBuffer[80];
	byte i;

	if ( tmpSM8Button != valSM8Button ) {
		#ifdef DEBUG_OUTPUT_SM8OUTPUT
		SerialTimePrintf(F("SM8 output changed\r\n"));
		#endif
		expanderWriteWord(MPC_SM8BUTTON,   GPIOA, valSM8Button);

		for (i = 0; i < IOBITS_CNT; i++) {
			if ( (bitRead(tmpSM8Button, i) != bitRead(valSM8Button, i)) ) {
				sendStatus(F("03 FS20 INPUT  %2d %s"), i, bitRead(valSM8Button,i)?"OFF":"ON");
			}

			// SM8 Taste Timeout setzen, falls Tastenausgang gerade aktiviert wurde
			if ( (bitRead(tmpSM8Button, i) != bitRead(valSM8Button, i)) && (bitRead(valSM8Button, i) == 0) ) {
				#ifdef DEBUG_OUTPUT_SM8OUTPUT
				SerialTimePrintf(F("Set SM8 key %d timeout to %d ms\r\n"), i+1, FS20_SM8_IN_RESPONSE/TIMER_MS);
				#endif
				SM8Timeout[i] = FS20_SM8_IN_RESPONSE / TIMER_MS;
			}
		}

		tmpSM8Button = valSM8Button;
	}
}

/* ===================================================================
 * Function:    ctrlMotorRelais
 * Return:
 * Arguments:
 * Description: Kontrolle der Motor Ausgangssignale
 * ===================================================================*/
void ctrlMotorRelais(void)
{
	static IOBITS tmpMotorRelais = IOBITS_ZERO;
	static IOBITS tmpOutMotorRelais = IOBITS_ZERO;
	static byte   preMotorCount = 0;
	static IOBITS preMotorRelais[] = {IOBITS_ZERO, IOBITS_ZERO, IOBITS_ZERO};
	static TIMER  preMotorTimer = 0;
		   IOBITS outMotorRelais = IOBITS_ZERO;

	byte i;

	if ( (tmpMotorRelais != valMotorRelais) && (preMotorCount == 0) ) {
		#ifdef DEBUG_OUTPUT_MOTOR
		SerialTimePrintf(F("Motor output change test\r\n"));
		SerialTimePrintf(F("  tmpMotorRelais: 0x%04X\r\n"), tmpMotorRelais);
		SerialTimePrintf(F("  valMotorRelais: 0x%04X\r\n"), valMotorRelais);
		#endif
		/* Relais-Schaltzeiten beachten:
		 * Drehrichtungsrelais nicht bei laufendem Motor umschalten */
		MOTORBITS valMotorStat01;
		MOTORBITS valMotorStat11;
		MOTORBITS valMotorDirChange;

		preMotorTimer = 0;
		preMotorRelais[0] = valMotorRelais;
		preMotorRelais[1] = IOBITS_ZERO;
		preMotorRelais[2] = IOBITS_ZERO;

		// alle Relais, deren Drehrichtung wechselt
		valMotorDirChange = ((tmpMotorRelais>>MAX_MOTORS) & IOBITS_LOWMASK) ^ ((valMotorRelais>>MAX_MOTORS) & IOBITS_LOWMASK);
		// alle Relais, deren Motor von AUS auf EIN wechselt
		valMotorStat01 = ~(tmpMotorRelais & IOBITS_LOWMASK) & (valMotorRelais & IOBITS_LOWMASK);
		// alle Relais, deren Motor von EIN bleibt und deren Richtung wechselt
		valMotorStat11 = (tmpMotorRelais & IOBITS_LOWMASK) & (valMotorRelais & IOBITS_LOWMASK);
		valMotorStat11 &= valMotorDirChange;

		#ifdef DEBUG_OUTPUT_MOTOR
		SerialTimePrintf(F("    valMotorStat01:   0x%02X\r\n"), valMotorStat01);
		SerialTimePrintf(F("    valMotorStat11:   0x%02X\r\n"), valMotorStat11);
		SerialTimePrintf(F("    valMotorDirChange 0x%02X\r\n"), valMotorDirChange);
		#endif

		if ( valMotorStat11 ) {
			// Bei Motoren die bereits laufen: Zweistufiger Wechsel

			// 1. Zuerst die Drehrichtung behalten und Ausschalten
			// 		Drehrichtung aus vorherigem Wert Übernehmen
			preMotorRelais[2] |= tmpMotorRelais & ((IOBITS)valMotorDirChange<<MAX_MOTORS);
			// 		Motoren, die laufen, abschalten
			preMotorRelais[2] |= valMotorRelais & (~(IOBITS)valMotorStat11 | IOBITS_HIGHMASK);

			// 2. dann die Drehrichtung ändern und ausgeschaltet lassen
			// 		Drehrichtung aus aktuellem Wert Übernehmen
			preMotorRelais[1] |= valMotorRelais & ((IOBITS)valMotorDirChange<<MAX_MOTORS);
			// 		Motoren, die laufen, abschalten
			preMotorRelais[1] |= valMotorRelais & (~(IOBITS)valMotorStat11 | IOBITS_HIGHMASK);
			preMotorCount = 2;
			#ifdef DEBUG_OUTPUT_MOTOR
			SerialTimePrintf(F("A     preMotorRelais[2] 0x%04X\r\n"), preMotorRelais[2]);
			SerialTimePrintf(F("A     preMotorRelais[1] 0x%04X\r\n"), preMotorRelais[1]);
			#endif
		}
		else if ( valMotorStat01 ) {
			// Bei Motoren die eingeschaltet werden: Einstufiger Wechsel

			// 1. Die Drehrichtung ändern und ausgeschaltet lassen
			// 		Drehrichtung aus aktuellem Wert Übernehmen
			preMotorRelais[1] |= valMotorRelais & ((IOBITS)valMotorDirChange<<MAX_MOTORS);
			// 		Motoren, die eingeschaltet werden sollen, abschalten
			preMotorRelais[1] |= valMotorRelais & (~(IOBITS)valMotorStat01 | IOBITS_HIGHMASK);
			preMotorCount = 1;
			#ifdef DEBUG_OUTPUT_MOTOR
			SerialTimePrintf(F("B     preMotorRelais[1] 0x%04X\r\n"), preMotorRelais[1]);
			#endif
		}
		else {
			#ifdef DEBUG_OUTPUT_MOTOR
			SerialTimePrintf(F("C     preMotorCount=%d\r\n"), preMotorCount);
			#endif
		}
		// Nächste Ausgabe ohne Delay
		preMotorTimer = millis() - RELAIS_OPERATE_TIME;

		tmpMotorRelais = valMotorRelais;
	}

	if ( millis() > (preMotorTimer + RELAIS_OPERATE_TIME) ) {
		// Aktuelle Motorsteuerungsbits holen
		bool doSM8andTimeout = true;
		outMotorRelais = preMotorRelais[preMotorCount];
		
		if ( preMotorCount>0 ) {
			#ifdef DEBUG_OUTPUT_MOTOR
			SerialTimePrintf(F("D   a)preMotorCount=%d\r\n"), preMotorCount);
			#endif
			preMotorCount--;
			#ifdef DEBUG_OUTPUT_MOTOR
			SerialTimePrintf(F("D   b)preMotorCount=%d\r\n"), preMotorCount);
			#endif
			preMotorTimer = millis();
			doSM8andTimeout = false;
		}

		if ( tmpOutMotorRelais != outMotorRelais ) {
			#ifdef DEBUG_OUTPUT_MOTOR
			SerialTimePrintf(F("D   Output outMotorRelais 0x%04X\r\n"), outMotorRelais);
			#endif
			expanderWriteWord(MPC_MOTORRELAIS, GPIOA, outMotorRelais);

			if( doSM8andTimeout ) {
				for (i = 0; i < MAX_MOTORS; i++) {
					// Motor Timeout setzen
						// - falls Motor gerade aktiviert wurde
					if (    ( !bitRead(tmpOutMotorRelais, i) && bitRead(outMotorRelais, i) )
						// - oder bereits läuft und die Laufrichtung geändert wurde
						 || ( bitRead(outMotorRelais, i) && 
							  bitRead(tmpOutMotorRelais, i+MAX_MOTORS)!=bitRead(outMotorRelais, i+MAX_MOTORS) ) ) {
						#ifdef DEBUG_OUTPUT_MOTOR
						SerialTimePrintf(F("    Set motor %d timeout to %d.%-d s\r\n"), i+1, eepromMaxRuntime[i] / 1000, eepromMaxRuntime[i] % 1000);
						#endif
						MotorTimeout[i] = eepromMaxRuntime[i] / TIMER_MS;
					}
					// SM8 Ausgang für "Öffnen" aktiv, aber Motor ist AUS bzw. arbeitet auf "Schliessen"
					if (    bitRead(curSM8Status, i)
						 && (   bitRead(outMotorRelais, i)==0
							 || (bitRead(outMotorRelais, i)!=0 && bitRead(outMotorRelais, i+MAX_MOTORS)==0) ) ) {
						// SM8 Taste für "Öffnen" zurücksetzen
						#ifdef DEBUG_OUTPUT_MOTOR
						SerialTimePrintf(F("SM8 Taste %d (für \"Öffnen\") zurücksetzen\r\n"), i);
						#endif
						bitSet(SM8StatusIgnore, i);
						bitClear(valSM8Button, i);
					}

					// SM8 Ausgang für "Schliessen" aktiv, aber Motor ist AUS bzw. arbeitet auf "Öffnen"
					if (    bitRead(curSM8Status, i+MAX_MOTORS)
						 && (   bitRead(outMotorRelais, i)==0
							 || (bitRead(outMotorRelais, i)!=0 && bitRead(outMotorRelais, i + MAX_MOTORS)!=0) ) ) {
						// SM8 Taste für "Schliessen" zurücksetzen
						#ifdef DEBUG_OUTPUT_MOTOR
						SerialTimePrintf(F("SM8 Taste %d (für \"Schliessen\") zurücksetzen\r\n"), i+MAX_MOTORS);
						#endif
						bitSet(SM8StatusIgnore, i+MAX_MOTORS);
						bitClear(valSM8Button, i+MAX_MOTORS);
					}
				}
			}
			tmpOutMotorRelais = outMotorRelais;
		}
	}
	#ifdef DEBUG_OUTPUT_MOTOR
	else {
		SerialTimePrintf(F("Z   Timer waiting\r\n"));
	}
	#endif
}

/* ===================================================================
 * Function:    ctrlRainSensor
 * Return:
 * Arguments:
 * Description: Kontrolle der Regensensor Eingänge
 * ===================================================================*/
void ctrlRainSensor(void)
{
	bool RainInput;
	bool RainEnable;
	bool tmpRainEnable;
	static bool prevtmpRainEnable = false;
	static bool prevRainInput = false;
	static bool prevRainEnable = false;

	// Debouncing inputs
	debEnable.update();
	debInput.update();

	// Wenn Regensensor EIN/AUS-Schalter betätigt wird
	// aktivieren Automatik wieder
	tmpRainEnable = (debEnable.read() == RAIN_ENABLE_AKTIV);

	if ( tmpRainEnable != prevtmpRainEnable ) {
		//TODO: Eingangsänderung RAIN_ENABLE sollte Automatik wieder einschalten

		//bitSet(eepromRain, RAIN_BIT_AUTO);
		// Write new value into EEPROM
		//eepromWriteByte(EEPROM_ADDR_RAIN, eepromRain);
		// Write new EEPROM checksum
		//eepromWriteLong(EEPROM_ADDR_CRC32, eepromCalcCRC());
		prevtmpRainEnable = tmpRainEnable;
	}

	// Get the updated value :
	if ( bitRead(eepromRain, RAIN_BIT_AUTO)!=0 ) {
		RainEnable = tmpRainEnable;
		RainInput  = (debInput.read()  == RAIN_INPUT_AKTIV);
	}
	else {
		RainEnable = (bitRead(eepromRain, RAIN_BIT_ENABLE)!=0);
		RainInput  = softRainInput;
	}

	if ( prevRainInput != RainInput || prevRainEnable != RainEnable) {
		#ifdef DEBUG_OUTPUT_RAIN
		SerialTimePrintf(F("RainDetect: %s\r\n"), bitRead(eepromRain, RAIN_BIT_AUTO)!=0?"Auto":"Software");
		SerialTimePrintf(F("RainEnable: %d\r\n"), RainEnable);
		SerialTimePrintf(F("RainInput:  %d\r\n"), RainInput);
		#endif
		if ( RainEnable ) {
			#ifdef DEBUG_OUTPUT_RAIN
			SerialTimePrintf(F("Rain inputs changed, sensor enabled\r\n"));
			#endif
			if ( RainInput ) {
				#ifdef DEBUG_OUTPUT_RAIN
				SerialTimePrintf(F("Rain active, close all windows\r\n"));
				#endif
				byte i;

				for (i = 0; i < MAX_MOTORS; i++) {
					if ( bitRead(eepromMTypeBitmask, i) ) {
						if ( getMotorDirection(i)!=MOTOR_CLOSE ) {
							#ifdef DEBUG_OUTPUT_RAIN
							SerialTimePrintf(F("Close window %d\r\n"), i);
							#endif
							setMotorDirection(i, MOTOR_CLOSE);
						}
					}
				}
				RainDetect = true;
				digitalWrite(ONBOARD_LED, HIGH);
			}
			else {
				#ifdef DEBUG_OUTPUT_RAIN
				SerialTimePrintf(F("Rain inactive, do nothing"));
				#endif
				digitalWrite(ONBOARD_LED, LOW);
				RainDetect = false;
			}
		}
		else {
			#ifdef DEBUG_OUTPUT_RAIN
			SerialTimePrintf(F("Rain inputs changed, sensor disabled"));
			#endif
			digitalWrite(ONBOARD_LED, LOW);
			RainDetect = false;
		}
		prevRainEnable = RainEnable;
		prevRainInput  = RainInput;
	}
}


/* ===================================================================
 * Function:    beAlive()
 * Return:
 * Arguments:
 * Description: Lebenszeichen (watchdog bedienen, debug output)
 * ===================================================================*/
void beAlive(void)
{
	#ifdef DEBUG_OUTPUT_ALIVE
	static char liveToogle = 0;
	static byte liveDots = 80;
	#endif

	// Live timer und watchdog handling
	if ( millis() > (runTimer + 500) )
	{
		runTimer = millis();
		#ifdef WATCHDOG_ENABLED
		Watchdog.reset();
		#endif
		#ifdef DEBUG_OUTPUT_ALIVE
		if ((liveToogle--) < 1) {
			SerialPrintf(F("."));
			liveToogle = 3;
			liveDots++;
			if ( liveDots > 76 ) {
				SerialPrintf(F("\r\n"));
				liveDots = 0;
			}
		}
		#endif
	}
}


/* ===================================================================
 * Function:    blinkLED()
 * Return:
 * Arguments:
 * Description: LED Blinken, um anzuzeigen, dass die Hauptschleife läuft
 * ===================================================================*/
void blinkLED(void)
{
	if ( millis() > (ledTimer + ledDelay) )
	{
		ledTimer = millis();
		if ( !ledStatus ) {
			digitalWrite(ONBOARD_LED, RainDetect?LOW:HIGH);
			ledDelay = eepromBlinkLen;
			ledStatus = true;
		}
		else {
			digitalWrite(ONBOARD_LED, RainDetect?HIGH:LOW);
			ledDelay = eepromBlinkInterval;
			ledStatus = false;
		}
	}
}


/* ===================================================================
 * Function:    loop()
 * Return:
 * Arguments:
 * Description: the loop function runs over and over again forever
 * ===================================================================*/
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

	// Live timer und watchdog handling
	beAlive();
	// LED Blinken, um anzuzeigen, dass die Hauptschleife läuft
	blinkLED();

	// Pro
	processSerialCommand();
}
