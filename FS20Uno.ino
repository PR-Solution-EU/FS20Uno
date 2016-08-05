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
 * MTYPE_BITMASK, MOTOR_WINDOW_MAXRUNTIME und 
 * MOTOR_JALOUSIE_MAXRUNTIME vordefinieren, als auch später mit Hilfe
 * der Control-Kommandos "MOTORTYPE" und "MOTORTIME" verändern.
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
 * 
 * Ein Regensensor kann am Eingang "RAIN_INPUT" angeschlossen werden 
 * (Regensensor-Steuereingang). Die Pegelaktivität kann mit Hilfe
 * der Konstanten "RAIN_INPUT_AKTIV" festgelegt werden.
 * 
 * Die Abfrage des Regensensor-Steuereingangs läßt sich mit Hilfe
 * eines zweiten Steuereingangs "RAIN_ENABLE" (Pegelaktivität über
 * die Konstante "RAIN_ENABLE_AKTIV" einstellbar) festlegen. Nur wenn
 * der Steuereingang "RAIN_ENABLE" aktiv oder das Control-Kommando
 * "RAINSENSOR ENABLE" gegeben wurde, wird der Regensensor-
 * Steuereingang auch abgefragt, ansonsten ignoriert.
 * 
 * Der Regensensor-Steuereingang wird nur im Automatikmodus (s.u.
 * Control-Kommando "RAINSENSOR") abgefragt.
 * Die Verwendung der "RAINSENSOR" Kommandos (mit Ausnahme von
 * "RAINSENSOR AUTO") schaltet Abfrage des Regensensor-Steuereingangs
 * erstmal ab. Der Regensensor-Steuereingang wird entweder durch das
 * Control-Kommando "RAINSENSOR AUTO" oder durch Änderung des
 * Steuereingangs "RAIN_ENABLE" wieder aktiviert.
 * 
 * Bei aktivier Regenmeldung (Steuereingang RAIN_INPUT bzw. Control-
 * Kommando RAINSENSOR ON), werden alle Motoren vom Typ "Fenster"
 * (siehe Motorensteuerung) geschlossen.
 *
 * Control-Kommando "RAINSENSOR"
 * Die beiden Eingänge lassen sich über Control-Kommandos "RAINSENSOR"
 * übersteuern:
 * - RAINSENSOR ON|OFF
 *     Simuliert den Regensensoreingang:  ON =Aktiv
 *                                        OFF=Inaktiv
 * - RAINSENSOR ENABLE|DISABLE
 *     Simuliert die Regensensor-Abfrage: ENABLE =Abfrage aktiv,
 *                                        DISABLE=Abfrage inaktiv
 * - RAINSENSOR AUTO
 *     Da bei Verwendung eines der o.a. Befehle der Automatikmodus
 *     (d.h. die Abfrage der Regensensor-Hardwareeingänge)
 *     abgeschaltet wird, kann hiermit die Abfrage der Regensensor-
 *     Hardwareeingänge wieder aktiviert werden. Die eingestellten
 *     Simulationswerte haben, solange sie nicht wieder verwendet
 *     werden, keine Bedeutung mehr.
 * ===================================================================*/

/* ===================================================================
 * TODO
 * SerialCmd: LOGIN
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
#define VERSION "2.16"
#include "REVISION.h"
#define DATAVERSION 106


// define next macros to output debug prints
#undef DEBUG_OUTPUT
#undef DEBUG_PINS				// enable debug output pins
#undef DEBUG_OUTPUT_SETUP		// enable setup related outputs
#undef DEBUG_OUTPUT_WATCHDOG	// enable watchdog related outputs
#undef DEBUG_OUTPUT_EEPROM		// enable EEPROM related outputs
#define DEBUG_OUTPUT_SM8STATUS	// enable FS20-SM8-output related output
#undef DEBUG_OUTPUT_WALLBUTTON	// enable wall button related output
#define DEBUG_OUTPUT_SM8OUTPUT	// enable FS20-SM8-key related output
#define DEBUG_OUTPUT_MOTOR		// enable motor control related output
#undef DEBUG_OUTPUT_MOTOR_DETAILS// enable motor control details output
#undef DEBUG_OUTPUT_RAIN		// enable rain sensor related output
#undef DEBUG_OUTPUT_ALIVE		// enable program alive signal output

#ifndef DEBUG_OUTPUT
	#undef DEBUG_PINS
	#undef DEBUG_OUTPUT_SETUP
	#undef DEBUG_OUTPUT_WATCHDOG
	#undef DEBUG_OUTPUT_EEPROM
	#undef DEBUG_OUTPUT_SM8STATUS
	#undef DEBUG_OUTPUT_WALLBUTTON
	#undef DEBUG_OUTPUT_SM8OUTPUT
	#undef DEBUG_OUTPUT_MOTOR
	#undef DEBUG_OUTPUT_MOTOR_DETAILS
	#undef DEBUG_OUTPUT_RAIN
	#undef DEBUG_OUTPUT_ALIVE

	#define WATCHDOG_ENABLED
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

// Sende autom. Statusänderung
volatile bool eepromSendStatus = true;


// Software Regendetection (wird nicht in EEPROM gespeichert)
volatile bool softRainInput = false;

// Rain Sensor inputs and vars
Bounce debEnable = Bounce();
Bounce debInput = Bounce();
volatile bool isRaining = false;



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
	SerialTimePrintf(F("setup - Debug output enabled\r\n"));
	#endif

	#ifdef DEBUG_PINS
	SerialTimePrintf(F("setup - Debug pins enabled\r\n"));
	pinMode(DBG_INT, OUTPUT); 		// debugging
	pinMode(DBG_MPC, OUTPUT); 		// debugging
	pinMode(DBG_TIMER, OUTPUT);		// debugging
	pinMode(DBG_TIMERLEN, OUTPUT);	// debugging
	#endif

	pinMode(STATUS_LED, OUTPUT);   // for onboard LED
	// indicate setup started
	digitalWrite(STATUS_LED, HIGH);

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
	digitalWrite(DBG_TIMERLEN, LOW);// debugging
	#endif

	Wire.begin();
	// expander configuration register
	expanderWriteBoth(MPC_MOTORRELAIS, IOCON, 0b00100100);	//                    sequential mode, INT Open-drain output
	expanderWriteBoth(MPC_SM8BUTTON,   IOCON, 0b00100100);	//                    sequential mode, INT Open-drain output
	expanderWriteBoth(MPC_SM8STATUS,   IOCON, 0b01100100);	// mirror interrupts, sequential mode, INT Open-drain output
	expanderWriteBoth(MPC_WALLBUTTON,  IOCON, 0b01100100);	// mirror interrupts, sequential mode, INT Open-drain output

	// enable pull-up on switches
	expanderWriteBoth(MPC_MOTORRELAIS, GPPU, 0xFF);  	// pull-up resistor A/B
	expanderWriteBoth(MPC_SM8BUTTON,   GPPU, 0xFF);  	// pull-up resistor A/B
	expanderWriteBoth(MPC_SM8STATUS,   GPPU, 0xFF);  	// pull-up resistor A/B
	expanderWriteBoth(MPC_WALLBUTTON,  GPPU, 0xFF);  	// pull-up resistor A/B

	// port data
	expanderWriteWord(MPC_MOTORRELAIS, GPIOA, valMotorRelais);
	expanderWriteWord(MPC_SM8BUTTON,   GPIOA, valSM8Button);
	expanderWriteWord(MPC_SM8STATUS,   GPIOA, ~IOBITS_ZERO);
	expanderWriteWord(MPC_WALLBUTTON,  GPIOA, ~IOBITS_ZERO);

	// port direction
	expanderWriteBoth(MPC_MOTORRELAIS, IODIR, 0x00);	// OUTPUT
	expanderWriteBoth(MPC_SM8BUTTON,   IODIR, 0x00);	// OUTPUT
	expanderWriteBoth(MPC_SM8STATUS,   IODIR, 0xFF);	// INPUT
	expanderWriteBoth(MPC_WALLBUTTON,  IODIR, 0xFF);	// INPUT

	// invert polarity
	expanderWriteBoth(MPC_SM8STATUS,   IOPOL, 0xFF);	// invert polarity of signal
	expanderWriteBoth(MPC_WALLBUTTON,  IOPOL, 0xFF);	// invert polarity of signal

	// enable interrupts on input MPC
	expanderWriteBoth(MPC_SM8STATUS,   GPINTEN, 0xFF);	// enable interrupts
	expanderWriteBoth(MPC_WALLBUTTON,  GPINTEN, 0xFF);	// enable interrupts

	// read from interrupt capture ports to clear them
	expanderRead(MPC_SM8STATUS,  INTCAPA);
	expanderRead(MPC_SM8STATUS,  INTCAPB);
	expanderRead(MPC_WALLBUTTON, INTCAPA);
	expanderRead(MPC_WALLBUTTON, INTCAPB);

	// pin 19 of MCP23017 is plugged into D2 of the Arduino
	// which is interrupt 0
	pinMode(MPC_INT_INPUT, INPUT_PULLUP);// make int input
	digitalWrite(MPC_INT_INPUT, HIGH);	// enable pull-up as we have made
									// the interrupt pins open drain

	// Read EEPROM program variables
	setupEEPROMVars();

	#ifdef WATCHDOG_ENABLED
	#ifndef DEBUG_OUTPUT_WATCHDOG
	Watchdog.enable(4000);
	#else
	int countdownMS = Watchdog.enable(4000);
	SerialTimePrintf(F("setup - Enabled the watchdog with max countdown of %d ms\r\n"), countdownMS);
	#endif
	#endif

	setupSerialCommand();

#ifdef DEBUG_OUTPUT_SETUP
	SerialTimePrintf(F("setup - Setup done, starting main loop()\r\n"));
#endif

	// indicate setup was done
	digitalWrite(STATUS_LED, LOW);

	// External interrupt
	attachInterrupt(digitalPinToInterrupt(MPC_INT_INPUT), extISR, FALLING);

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
void setMotorDirection(byte motorNum, MOTOR_CTRL newDirection)
{
	if ( motorNum < MAX_MOTORS ) {
		// Neue Richtung: Öffnen
		if ( newDirection >= MOTOR_OPEN ) {
			// Motor läuft auf Schliessen
			if (MotorCtrl[motorNum] <= MOTOR_CLOSE) {
				// Motor auf Öffnen mit Umschaltdelay
				MotorCtrl[motorNum] = MOTOR_OPEN_DELAYED;
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
				MotorCtrl[motorNum] = MOTOR_CLOSE_DELAYED;
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
	}
}

/* ===================================================================
 * Function:    getMotorDirection
 * Return:		Laufrichtung
 *              MOTOR_OPEN, MOTOR_OPEN_DELAYED
 *              MOTOR_CLOSE, MOTOR_CLOSE_DELAYED
 *              MOTOR_OFF
 * Arguments:	motorNum - die Motorennummer [0..x]
 * Description: Motor Laufrichtung zurückgeben
 * ===================================================================*/
char getMotorDirection(byte motorNum)
{
	if (MotorCtrl[motorNum] == MOTOR_OPEN) {
		return MOTOR_OPEN;
	}
	else if (MotorCtrl[motorNum] > MOTOR_OPEN) {
		return MOTOR_OPEN_DELAYED;
	}
	else if (MotorCtrl[motorNum] == MOTOR_CLOSE) {
		return MOTOR_CLOSE;
	}
	else if (MotorCtrl[motorNum] < MOTOR_CLOSE) {
		return MOTOR_CLOSE_DELAYED;
	}
	return MOTOR_OFF;
}



#if defined(DEBUG_OUTPUT_MOTOR) || defined(DEBUG_OUTPUT_MOTOR_DETAILS)
void debugPrintMotorStatus(bool from)
{
	static MOTOR_CTRL prevMotorCtrl[MAX_MOTORS] = {MOTOR_OFF, MOTOR_OFF, MOTOR_OFF, MOTOR_OFF, MOTOR_OFF, MOTOR_OFF, MOTOR_OFF, MOTOR_OFF};
	bool motorChanged;
	byte i;

	motorChanged = false;
	for (i = 0; i < MAX_MOTORS && !motorChanged; i++) {
		motorChanged = prevMotorCtrl[i]!=MotorCtrl[i];
	}
	
	if ( motorChanged ) {
		byte i;

		SerialTimePrintf(F("debugPrintMotorStatus(%i) - ----------------------------------------\r\n"), from);
		SerialTimePrintf(F("debugPrintMotorStatus(%i) -    M1   M2   M3   M4   M5   M6   M7   M8\r\n"), from);
		SerialTimePrintf(F("debugPrintMotorStatus(%i) - "), from);
		for (i = 0; i < MAX_MOTORS; i++) {
			if (MotorCtrl[i] == 0) {
				SerialPrintf(F("  off"));
			}
			else {
				SerialPrintf(F("%5d"), MotorCtrl[i]);
			}
			prevMotorCtrl[i] = MotorCtrl[i];
		}
		SerialPrintf(F("\r\n"));
	}
}
#endif

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

	if ( (tmpSM8Status != curSM8Status) ) {
		#ifdef DEBUG_OUTPUT_SM8STATUS
		SerialTimePrintf(F("ctrlSM8Status   - ----------------------------------------\r\n"));
		SerialTimePrintf(F("ctrlSM8Status   - SM8 Input Status changed\r\n"));
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
			SerialTimePrintf(F("ctrlSM8Status   - prevSM8Status:   0x%04x\r\n"), prevSM8Status);
			SerialTimePrintf(F("ctrlSM8Status   - SM8Status:       0x%04x\r\n"), SM8Status);
			SerialTimePrintf(F("ctrlSM8Status   - SM8StatusSlope:  0x%04x\r\n"), SM8StatusSlope);
			SerialTimePrintf(F("ctrlSM8Status   - SM8StatusChange: 0x%04x\r\n"), SM8StatusChange);
			SerialTimePrintf(F("ctrlSM8Status   - SM8StatusIgnore: 0x%04x\r\n"), SM8StatusIgnore);
			#endif
			// Eventuell Änderungen einzelner Bits ignorieren
			SM8StatusChange &= ~SM8StatusIgnore;
			SM8StatusIgnore = 0;

			#ifdef DEBUG_OUTPUT_SM8STATUS
			SerialTimePrintf(F("ctrlSM8Status   - SM8StatusChange: 0x%04x\r\n"), SM8StatusChange);
			#endif

			for (i = 0; i < IOBITS_CNT; i++) {
				if ( bitRead(SM8StatusChange, i) ) {
					sendStatus(F("02 FS20 OUTPUT %2d %s"), i, bitRead(curSM8Status,i)?"ON":"OFF");
				}
			}
			for (i = 0; i < MAX_MOTORS; i++) {
				// Motor Öffnen
				if ( bitRead(SM8StatusChange, i) != 0 && bitRead(SM8StatusChange, i + MAX_MOTORS) == 0 ) {
					#ifdef DEBUG_OUTPUT_SM8STATUS
					SerialTimePrintf(F("ctrlSM8Status   - Motor %i should be opened\r\n"), i);
					#endif
					if ( bitRead(SM8StatusSlope, i) != 0 ) {
						#ifdef DEBUG_OUTPUT_SM8STATUS
						SerialTimePrintf(F("ctrlSM8Status   - Motor %i set to OPEN\r\n"), i);
						#endif
						// Flanke von 0 nach 1: Motor Ein
						setMotorDirection(i, MOTOR_OPEN);
						// Taste für Schliessen aktiv?
						if ( bitRead(SM8Status, i + MAX_MOTORS) != 0 ) {
							#ifdef DEBUG_OUTPUT_SM8STATUS
							SerialTimePrintf(F("ctrlSM8Status   - Reset FS20 'close' key %i\r\n"), i + MAX_MOTORS);
							#endif
							// Taste für Schliessen zurücksetzen
							bitClear(valSM8Button, i + MAX_MOTORS);
							bitSet(SM8StatusIgnore, i + MAX_MOTORS);
						}
					}
					else {
						// Flanke von 0 nach 1: Motor Aus
						#ifdef DEBUG_OUTPUT_SM8STATUS
						SerialTimePrintf(F("ctrlSM8Status   - Open 0>1 Slope - Motor %i off\r\n"), i);
						#endif
						setMotorDirection(i, MOTOR_OFF);
					}
				}
				// Motor Schliessen
				else if ( bitRead(SM8StatusChange, i) == 0 && bitRead(SM8StatusChange, i + MAX_MOTORS) != 0 ) {
					#ifdef DEBUG_OUTPUT_SM8STATUS
					SerialTimePrintf(F("ctrlSM8Status   - Motor %i should be closed\r\n"), i);
					#endif
					if ( bitRead(SM8StatusSlope, i + MAX_MOTORS) != 0 ) {
						#ifdef DEBUG_OUTPUT_SM8STATUS
						SerialTimePrintf(F("ctrlSM8Status   - Motor %i set to CLOSE\r\n"), i);
						#endif
						// Flanke von 0 nach 1: Motor Ein
						setMotorDirection(i, MOTOR_CLOSE);
						// Taste für Öffnen aktiv?
						if ( bitRead(SM8Status, i) != 0 ) {
							#ifdef DEBUG_OUTPUT_SM8STATUS
							SerialTimePrintf(F("ctrlSM8Status   - Reset FS20 'open' key %i\r\n"), i);
							#endif
							// Taste für Öffnen zurücksetzen
							bitClear(valSM8Button, i);
							bitSet(SM8StatusIgnore, i);
						}
					}
					else {
						// Flanke von 0 nach 1: Motor Aus
						#ifdef DEBUG_OUTPUT_SM8STATUS
						SerialTimePrintf(F("ctrlSM8Status   - Close 0>1 Slope - Motor %i off\r\n"), i);
						#endif
						setMotorDirection(i, MOTOR_OFF);;
					}
				}
				// Ungültig: Änderungen beider Eingänge zur gleichen Zeit
				else if ( bitRead(SM8StatusChange, i) != 0 && bitRead(SM8StatusChange, i + MAX_MOTORS) != 0 ) {
					#ifdef DEBUG_OUTPUT_SM8STATUS
					SerialTimePrintf(F("ctrlSM8Status   - Invalid 0>1 Slope, reset key %i and %i\r\n"), i, i+MAX_MOTORS);
					#endif
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
		debugPrintMotorStatus(false);
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
					setMotorDirection(i, MOTOR_OPEN);
				}
				else if ( bitRead(WallButtonChange, i + MAX_MOTORS) != 0 && bitRead(WallButtonSlope, i + MAX_MOTORS) != 0 ) {
					setMotorDirection(i, MOTOR_CLOSE);
				}
				// Pegel Öffnen und Schliessen = 1:
				if ( bitRead(WallButton, i) != 0 && bitRead(WallButton, i + MAX_MOTORS) != 0 ) {
					setMotorDirection(i, MOTOR_OFF);
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
			SerialTimePrintf(F("ctrlWallButton  - ----------------------------------------\r\n"));
			SerialTimePrintf(F("ctrlWallButton  - prevWallButton:   0x%04x\r\n"), prevWallButton);
			SerialTimePrintf(F("ctrlWallButton  - WallButton:       0x%04x\r\n"), WallButton);
			SerialTimePrintf(F("ctrlWallButton  - WallButtonSlope:  0x%04x\r\n"), WallButtonSlope);
			SerialTimePrintf(F("ctrlWallButton  - WallButtonChange: 0x%04x\r\n"), WallButtonChange);
			SerialTimePrintf(F("ctrlWallButton  - WallButtonLocked: 0x%04x\r\n"), WallButtonLocked);
			#endif

			prevWallButton = curWallButton;
		}

		#ifdef DEBUG_OUTPUT_MOTOR
		debugPrintMotorStatus(true);
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
	static IOBITS tmpSM8Button = ~IOBITS_ZERO;
	byte i;

	if ( tmpSM8Button != valSM8Button ) {
		#ifdef DEBUG_OUTPUT_SM8OUTPUT
		SerialTimePrintf(F("ctrlSM8Button   - ----------------------------------------\r\n"));
		SerialTimePrintf(F("ctrlSM8Button   - SM8 key output changed\r\n"));
		#endif
		expanderWriteWord(MPC_SM8BUTTON,   GPIOA, valSM8Button);

		for (i = 0; i < IOBITS_CNT; i++) {
			if ( (bitRead(tmpSM8Button, i) != bitRead(valSM8Button, i)) ) {
				sendStatus(F("03 FS20 KEY  %2d %s"), i, bitRead(valSM8Button,i)?"OFF":"ON");
			}

			// SM8 Taste Timeout setzen, falls Tastenausgang gerade aktiviert wurde
			if ( (bitRead(tmpSM8Button, i) != bitRead(valSM8Button, i)) && (bitRead(valSM8Button, i) == 0) ) {
				#ifdef DEBUG_OUTPUT_SM8OUTPUT
				SerialTimePrintf(F("ctrlSM8Button   - SM8 key %d output set timeout to %d ms\r\n"), i+1, FS20_SM8_IN_RESPONSE/TIMER_MS);
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
	static IOBITS preMotorRelais[4];
	static TIMER  preMotorTimer = 0;
		   IOBITS outMotorRelais = IOBITS_ZERO;

	byte i;

	if ( (tmpMotorRelais != valMotorRelais) && (preMotorCount == 0) ) {
		byte k;
		byte curTarget;
		byte targetSteps;
		byte targetStep[4];

		#ifdef DEBUG_OUTPUT_MOTOR
		SerialTimePrintf(F("ctrlMotorRelais - ----------------------------------------\r\n"));
		SerialTimePrintf(F("ctrlMotorRelais - Motor output change test\r\n"));
		SerialTimePrintf(F("ctrlMotorRelais -   tmpMotorRelais: 0x%04X\r\n"), tmpMotorRelais);
		SerialTimePrintf(F("ctrlMotorRelais -   valMotorRelais: 0x%04X\r\n"), valMotorRelais);
		#endif
		/* Relais-Schaltzeiten beachten:
		 * Generell: Drehrichtungsrelais nicht bei laufendem Motor umschalten
		 */
		//~ MOTORBITS valMotorStat01;
		//~ MOTORBITS valMotorStat11;
		//~ MOTORBITS valMotorDirChange;

		preMotorTimer = 0;
		preMotorCount = 0;
		preMotorRelais[0] = valMotorRelais;
		preMotorRelais[1] = valMotorRelais;
		preMotorRelais[2] = valMotorRelais;
		preMotorRelais[3] = valMotorRelais;

		/* Tabelle (M=Motorbit, D=Directionbit, S=Schritte)
		 * Ist Soll Schritte
		 * MD  MD   MD          S MC MD 
		 * 00  00   -        -  0 00 00
		 * 00  01   01       -  1 00 01
		 * 00  10   10       -  1 01 00
		 * 00  11   01 11    x  2 01 01
		 * 01  00   00       -  1 00 10
		 * 01  01   -        -  0 00 11
		 * 01  10   00 10    x  2 01 10
		 * 01  11   11       -  1 01 11
		 * 10  00   00       -  1 10 00
		 * 10  01   00 01    x  2 10 01
		 * 10  10   -        -  0 11 00
		 * 10  11   00 01 11 x  3 11 01
		 * 11  00   01 00    x  2 10 10
		 * 11  01   01       -  1 10 11
		 * 11  10   01 00 10 x  3 11 00
		 * 11  11   -        -  0 11 11
		 * 
		 */
		// Jeden Motor einzeln testen
		for(i=0; i<MAX_MOTORS; i++) {
			curTarget = 0;
			bitWrite(curTarget, 3, bitRead(tmpMotorRelais, i));
			bitWrite(curTarget, 2, bitRead(tmpMotorRelais, i+MAX_MOTORS));
			bitWrite(curTarget, 1, bitRead(valMotorRelais, i));
			bitWrite(curTarget, 0, bitRead(valMotorRelais, i+MAX_MOTORS));
			// Letzter Schritt (Relais Endzustand) wird immer
			// ausgegeben
			targetSteps = 0;
			//targetStep[0] = curTarget & 0b11;

			switch (curTarget) {
			case 0b0000:
				break;
			case 0b0001:
				break;
			case 0b0010:
				break;
			case 0b0011:
				targetStep[1] = 0b01;
				targetSteps = 1;
				break;
			case 0b0100:
				break;
			case 0b0101:
				break;
			case 0b0110:
				targetStep[1] = 0b00;
				targetSteps = 1;
				break;
			case 0b0111:
				break;
			case 0b1000:
				break;
			case 0b1001:
				targetStep[1] = 0b00;
				targetSteps = 1;
				break;
			case 0b1010:
				break;
			case 0b1011:
				targetStep[2] = 0b00;
				targetStep[1] = 0b01;
				targetSteps = 2;
				break;
			case 0b1100:
				targetStep[1] = 0b01;
				targetSteps = 1;
				break;
			case 0b1101:
				break;
			case 0b1110:
				targetStep[2] = 0b01;
				targetStep[1] = 0b00;
				targetSteps = 2;
				break;
			case 0b1111:
				break;
			}
			#ifdef DEBUG_OUTPUT_MOTOR_DETAILS
			SerialTimePrintf(F("ctrlMotorRelais -   Motor %d, curTarget 0x%02X, Steps %d\r\n"), i, curTarget, targetSteps);
			#endif

			// Alle Schritte in die entgültigen Masken kopieren
			for(k=targetSteps; k>0; k--) {
				#ifdef DEBUG_OUTPUT_MOTOR_DETAILS
				SerialTimePrintf(F("ctrlMotorRelais -     targetStep[%d]=0x%02X\r\n"), k, targetStep[k]);
				#endif
				// Jetzt die Bits des gerade betrachteten Motors ausblenden
				bitClear(preMotorRelais[k], i);
				bitClear(preMotorRelais[k], i+MAX_MOTORS);
				// Die neuen Bits dieses Schrittes für Motor
				bitWrite(preMotorRelais[k], i, targetStep[k]>>1);
				// Die neuen Bits dieses Schrittes für Drehrichtung
				bitWrite(preMotorRelais[k], i+MAX_MOTORS, targetStep[k] & 1);
			}

			if( targetSteps > preMotorCount ) {
				preMotorCount = targetSteps;
			}
		}
		#ifdef DEBUG_OUTPUT_MOTOR_DETAILS
		SerialTimePrintf(F("ctrlMotorRelais -   preMotorCount=%d\r\n"), preMotorCount);
		for(i=0; i<preMotorCount; i++) {
			SerialTimePrintf(F("ctrlMotorRelais -     preMotorRelais[%d] =0x%04X\r\n"), i, preMotorRelais[i]);
		}
		#endif

		// Nächste Ausgabe ohne Delay
		preMotorTimer = millis() - RELAIS_OPERATE_TIME;

		tmpMotorRelais = valMotorRelais;
	}

	if ( millis() > (preMotorTimer + RELAIS_OPERATE_TIME) ) {
		// Aktuelle Motorsteuerungsbits holen
		bool doSM8andTimeout = true;

		outMotorRelais = preMotorRelais[preMotorCount];
		
		if ( preMotorCount>0 ) {
			#ifdef DEBUG_OUTPUT_MOTOR_DETAILS
			SerialTimePrintf(F("ctrlMotorRelais - D   a)preMotorCount=%d\r\n"), preMotorCount);
			#endif
			preMotorCount--;
			#ifdef DEBUG_OUTPUT_MOTOR_DETAILS
			SerialTimePrintf(F("ctrlMotorRelais - D   b)preMotorCount=%d\r\n"), preMotorCount);
			#endif
			preMotorTimer = millis();
			doSM8andTimeout = false;
		}

		if ( tmpOutMotorRelais != outMotorRelais ) {
			#ifdef DEBUG_OUTPUT_MOTOR
			SerialTimePrintf(F("ctrlMotorRelais -     Output outMotorRelais 0x%04X\r\n"), outMotorRelais);
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
						SerialTimePrintf(F("ctrlMotorRelais -     Set motor %d timeout to %d.%-d s\r\n"), i+1, eepromMaxRuntime[i] / 1000, eepromMaxRuntime[i] % 1000);
						#endif
						MotorTimeout[i] = eepromMaxRuntime[i] / TIMER_MS;
					}
					// SM8 Ausgang für "Öffnen" aktiv, aber Motor ist AUS bzw. arbeitet auf "Schliessen"
					if ( bitRead(curSM8Status, i)
						 && (getMotorDirection(i)==MOTOR_OFF || getMotorDirection(i)==MOTOR_CLOSE) ) {
						// SM8 Taste für "Öffnen" zurücksetzen
						#if defined(DEBUG_OUTPUT_MOTOR) || defined(DEBUG_OUTPUT_SM8OUTPUT)
						SerialTimePrintf(F("ctrlMotorRelais - SM8 Taste %d (für \"Öffnen\") zurücksetzen\r\n"), i);
						#endif
						bitSet(SM8StatusIgnore, i);
						bitClear(valSM8Button, i);
					}

					// SM8 Ausgang für "Schliessen" aktiv, aber Motor ist AUS bzw. arbeitet auf "Öffnen"
					if (    bitRead(curSM8Status, i+MAX_MOTORS)
						 && (getMotorDirection(i)==MOTOR_OFF || getMotorDirection(i)==MOTOR_OPEN) ) {
						// SM8 Taste für "Schliessen" zurücksetzen
						#if defined(DEBUG_OUTPUT_MOTOR) || defined(DEBUG_OUTPUT_SM8OUTPUT)
						SerialTimePrintf(F("ctrlMotorRelais - SM8 Taste %d (für \"Schliessen\") zurücksetzen\r\n"), i+MAX_MOTORS);
						#endif
						bitSet(SM8StatusIgnore, i+MAX_MOTORS);
						bitClear(valSM8Button, i+MAX_MOTORS);
					}
				}
			}
			tmpOutMotorRelais = outMotorRelais;
		}
	}
	#ifdef DEBUG_OUTPUT_MOTOR_DETAILS
	else {
		SerialTimePrintf(F("ctrlMotorRelais - Z   Timer waiting\r\n"));
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
	bool sensRainInput;
	bool sensRainEnable;
	static bool prevRainInput = false;
	static bool prevRainEnable = false;
	static bool prevRainEnableInput = false;

	// Eingänge entprellen
	debEnable.update();
	debInput.update();

	// Entprellte Eingangssignale lesen
	sensRainEnable = (debEnable.read() == RAIN_ENABLE_AKTIV);
	sensRainInput  = (debInput.read()  == RAIN_INPUT_AKTIV);

	if( sensRainEnable != prevRainEnableInput ) {
		// AUTO Modus wieder aktivieren, wenn Enable-Eingang sich
		// geändert hat
		bitSet(eepromRain, RAIN_BIT_AUTO);
		prevRainEnableInput = sensRainEnable;
	}

	// Regensignale abhängig vom Modus lesen
	if ( bitRead(eepromRain, RAIN_BIT_AUTO) ) {
		// AUTO Modus aktiv
		RainEnable = sensRainEnable;
		RainInput  = sensRainInput;
	}
	else {
		// Modus manuell
		RainEnable = bitRead(eepromRain, RAIN_BIT_ENABLE);
		RainInput  = softRainInput;
	}

	if ( prevRainInput != RainInput || prevRainEnable != RainEnable ) {
		#ifdef DEBUG_OUTPUT_RAIN
		SerialTimePrintf(F("ctrlRainSensor  - ----------------------------------------\r\n"));
		SerialTimePrintf(F("ctrlRainSensor  - RainMode:   %s\r\n"), bitRead(eepromRain, RAIN_BIT_AUTO)!=0?"Auto":"Software");
		SerialTimePrintf(F("ctrlRainSensor  - RainEnable: %d\r\n"), RainEnable);
		SerialTimePrintf(F("ctrlRainSensor  - RainInput:  %d\r\n"), RainInput);
		#endif
		if ( RainEnable ) {
			#ifdef DEBUG_OUTPUT_RAIN
			SerialTimePrintf(F("ctrlRainSensor  - Rain inputs changed, sensor enabled\r\n"));
			#endif
			if ( RainInput ) {
				#ifdef DEBUG_OUTPUT_RAIN
				SerialTimePrintf(F("ctrlRainSensor  - Rain active, close all windows\r\n"));
				#endif
				byte i;

				for (i = 0; i < MAX_MOTORS; i++) {
					if ( bitRead(eepromMTypeBitmask, i) ) {
						if ( getMotorDirection(i)!=MOTOR_CLOSE && getMotorDirection(i)!=MOTOR_CLOSE_DELAYED) {
							#ifdef DEBUG_OUTPUT_RAIN
							SerialTimePrintf(F("ctrlRainSensor  - Close window %d\r\n"), i);
							#endif
							setMotorDirection(i, MOTOR_CLOSE);
						}
					}
				}
				isRaining = true;
				#ifdef DEBUG_OUTPUT_RAIN
				SerialTimePrintf(F("ctrlRainSensor  - Now it's raining\r\n"));
				#endif
				digitalWrite(STATUS_LED, HIGH);
			}
			else {
				#ifdef DEBUG_OUTPUT_RAIN
				SerialTimePrintf(F("ctrlRainSensor  - Rain inactive, do nothing\r\n"));
				#endif
				isRaining = false;
				#ifdef DEBUG_OUTPUT_RAIN
				SerialTimePrintf(F("ctrlRainSensor  - Now it's not raining\r\n"));
				#endif
				digitalWrite(STATUS_LED, LOW);
			}
		}
		else {
			#ifdef DEBUG_OUTPUT_RAIN
			SerialTimePrintf(F("ctrlRainSensor  - Rain inputs changed, sensor disabled\r\n"));
			#endif
			#ifdef DEBUG_OUTPUT_RAIN
			SerialTimePrintf(F("ctrlRainSensor  - Now it's not raining\r\n"));
			#endif
			isRaining = false;
			digitalWrite(STATUS_LED, LOW);
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
			digitalWrite(STATUS_LED, isRaining?LOW:HIGH);
			ledDelay = eepromBlinkLen;
			ledStatus = true;
		}
		else {
			digitalWrite(STATUS_LED, isRaining?HIGH:LOW);
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

	// Auslesen der Eingangssignale von SM8
	ctrlSM8Status();

	// Auslesen der Wandtaster
	// Wandtaster haben Vorrang vor SM8 Ausgänge,
	// daher Auslesen nach SM8
	ctrlWallButton();

	// Kontrolle der Motor Ausgangssignale
	ctrlMotorRelais();

	// Kontrolle der SM8 Tastensteuerung
	ctrlSM8Button();

	// Regensensor abfragen
	ctrlRainSensor();

	// Live timer und watchdog handling
	beAlive();
	// LED Blinken, um anzuzeigen, dass die Hauptschleife läuft
	blinkLED();

	// Pro
	processSerialCommand();
}
