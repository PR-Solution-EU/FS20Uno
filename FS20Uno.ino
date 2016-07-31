/* ==========================================================================
 * File:    FS20Uno.ino
 * Author:  Norbert Richter <mail@norbert-richter.info>
 * Project: Dachflächenfenster/-Jalousie Steuerung
 *          mit 2x FS20 SM8
 * Desc:    Die Steuerung besteht aus
 * 			- einem Arduino Uno
 * 			- drei Porterweterungen MCP23017
 * 			- zwei ELV FS20 SM8
 *          Der Uno über 16 Relais ingesamt acht Motoren
 *          - Ein Relais für Motor Ein/Aus
 * 			- Ein Relais für die Drehrichtung.
 *
 * Funktionsweise der Steuerung:
 * Zwei FS20 SM8 dienen als FS20 Empfänger für Dachflächenfenstersteuerung (DFF)
 * Mit Hilfer zweier FS20-SM8 stehen stehen 16 FS20 Kanäle zur Verfügung.
 * Jeweils zwei steuern einen Motor:
 * - ein Kanal steuert Linkslauf
 * - ein Kanal steuert Rechtslauf
 *
 * Damit lassen sich insgesamt 4 DF-Fenster mit jeweils zwei 24V Motoren ansteuern:
 * - ein Motor zum Öffnen/Schliessen des Fensters
 * - ein Motor für die Jalousie
 * Der Uno übernimmt dabei die 'Übersetzung' von Öffnen/Schliessen
 * auf die Motorzustände
 * - Aus
 * - Öffnen
 * - Schließen
 *
 * Da wir in den Ablauf des FS20 SM8 nur bedingt eingreifen können, werden die
 * FS20 SM8 nur als 'Geber' eingesetzt. Die Motorsteuerung erfolgt komplett
 * über dieses Steuerungsprogramm.
 *
 * Verdrahtung:
 * MPC1
 *   Port A (Output): Relais Motor 1-8 EIN/AUS
 *   Port B (Output): Relais Motor 1-8 Drehrichtung
 * MPC2
 *   Port A (Output): FS20-SM8 #1 Taster 1-8 ("Auf" Funktion: 1=DFF1, 2=DFF2, 3=DFF3, 4=DFF4, 5=Jal1...)
 *   Port B (Output): FS20-SM8 #2 Taster 1-8 ("Zu" Funktion)
 * MPC3
 *   Port A (Input) : FS20-SM8 #1 Status 1-8 ("Auf" Funktion)
 *   Port B (Input) : FS20-SM8 #2 Status 1-8 ("Zu" Funktion)
 *   Die FS20-SM8 Ausgänge schalten das Signal gegen Masse (0=Aktiv)
 * MPC4
 *   Port A (Input) : Wandtaster             ("Auf" Funktion)
 *   Port B (Input) : Wandtaster             ("Zu" Funktion)
 *   Die Taster schalten das Signal gegen Masse (0=Aktiv).
 *   Die Wandtaster haben folgende Funktion:
 *   - Taste Auf: "Auf" einschalten, nochmaliger Druck schaltet Motor ab.
 *   - Taste Zu : "Zu" einschalten, nochmaliger Druck schaltet Motor ab.
 *   - Beide Tasten: Schaltet Motor ab.
 *
 * Zwei Digitaleingänge des Uno werden für Zustandsvorgänge benutzt:
 * D7: Regensensor (Aktiv Low)
 * D8: Regensensor aktiv (Aktiv Low)
 *
 * Vom SM8 werden jeweils die Ausgänge als Steuereingang für die Motorsteuerung
 * herangezogen, dabei werden folgende Bedingungen besonders berücksichtigt:
 * - gleichzeitig aktivierte Ausgänge (Auf & Zu aktiv) werden entkoppelt
 *   Nur die jweils steigende Flanke eines Ausgangs steuert die Richtung.
 *   War bereits die entgegengesetzte Richtung aktiv, wird die Richtung
 *   umgeschaltet.
 * - Motorschutz:
 *   Das Einschalten der Motorspannung erfolgt erst, nachdem das Richtungsrelais
 *   auch wirklich umgeschalten hat (OPERATE_TIME).
 *   Ebenso wird bei laufendem Motor und Richtungsumkehr der Motor zuerst abge-
 *   schaltet, die Laufrichtung geändert und danach der Motor wieder einge-
 *   schaltet (MOTOR_SWITCHOVER).
 * - Die fallende Flanke des SM8 Ausgangs schaltet die jeweilige Drehrichtung ab
 *
 *
 * ========================================================================== */
/* TODO
 * - Fehler bei Motor Unlock (beide Richtungen gleichzeitig aktiv)
 *   wenn dies per FS20 Funk ausgelöst wurde.
 * - Falls beide Wandtasten gedrückt werden, darf ein Motor erst wieder
 *   aktiv werden, sobald beide Tasten losgelassen wurden
 * - Lernbarer Timeout (optional)
 * - Commands über RS232:
 * - ATx[=...]
 * - 'ATI' - Get Info
 * - 'ATD' - Get/Set Date/Time
 * - 'ATF' - New Firmware
 * - '
 */

#include <Wire.h>
#include <MsTimer2.h>	// http://playground.arduino.cc/Main/MsTimer2
#include <Adafruit_SleepyDog.h>
#include "FS20Uno.h"
#include "I2C.h"


// enable next line to enable debug output pins
#define DEBUG_PINS
// enable next line to output debug prints
#define DEBUG_OUTPUT

#ifndef DEBUG_OUTPUT
// enable next line to enable watchdog timer
#define WATCHDOG_ENABLED
#endif


#define MPC1    0x20    // MCP23017 #1 I2C address
#define MPC2    0x21    // MCP23017 #2 I2C address
#define MPC3    0x22    // MCP23017 #3 I2C address
#define MPC4    0x23    // MCP23017 #4 I2C address

#define MPC_MOTORRELAIS	MPC1
#define MPC_SM8BUTTON	MPC2
#define MPC_SM8STATUS	MPC3
#define MPC_WALLBUTTON	MPC4


#define ISR_INPUT 		2			// ISR Input from MPC = D2
#define ONBOARD_LED 	LED_BUILTIN	// LED = D13
#ifdef DEBUG_PINS
#define DBG_INT 		12			// Debug PIN = D12
#define DBG_MPC 		11			// Debug PIN = D11
#define DBG_TIMER	 	10			// Debug PIN = D10
#define DBG_TIMERLEN 	9			// Debug PIN = D9
#endif




/* ==========================================================================
 * Hardware definition (I/O Ports...)
 * Zeiten in ms müssen Vielfache von 10 sein
 * ========================================================================== */
#define MAX_MOTORS				8
// timer period in ms
#define TIMER_MS				10

// Relais Ansprechzeit in ms (Datenblatt 5 ms)
#define OPERATE_TIME			20
// Relais Rückfallzeit in ms (Datenblatt 4 ms))
#define RELEASE_TIME			20
// Motor Umschaltdelay in ms
#define MOTOR_SWITCHOVER		250
// SM8 IN Schaltzeit in ms (muss Vielfaches von 10 sein)
#define FS20_SM8_IN_RESPONSE	150
// Tasten Entprellzeit in ms
#define DEBOUNCE_TIME			20


// MPC Data
// MPC output data
volatile WORD valMotorRelais = 0x0000;
volatile WORD valSM8Button   = 0x0000;

// values read from MPC port during MPC interrupt
volatile WORD irqSM8Status   = 0x0000;
volatile WORD irqWallButton  = 0x0000;

// values currently used within program
volatile WORD curSM8Status   = 0x0000;
volatile WORD curWallButton  = 0x0000;

// debounce counter für keys
volatile char debSM8Status[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
volatile char debWallButton[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

// Motor Steuerungskommandos:
//   0: Motor AUS
//  >0: Motor Öffnen
//  <0: Motor Schliessen
// abs(Werte) <> 0 haben folgende Bedeutung:
//   1: Motor sofort schalten
//  >1: Motor Delay in (abs(Wert) - 1) * 10 ms
#define MOTOR_OPEN	1
#define MOTOR_CLOSE	-1
#define MOTOR_OFF	0
volatile char MotorCtrl[8] = {0,0,0,0,0,0,0,0};

// time we turned LED on
volatile DWORD ledSignalTimer = 0;
volatile DWORD ledTimer = 0;

volatile bool isrTrigger = false;



/*
 * Function:	setup
 * Return:
 * Arguments:
 * Description: setup function runs once when you press reset or power the board
 */
void setup()
{
	Serial.begin(115200);

	Serial.print(PROGRAM);
	Serial.print(" v");
	Serial.print(VERSION);
	Serial.print(" (build ");
	Serial.print(REVISION);
	Serial.println(")");
	Serial.println();

#ifdef DEBUG_PINS
	pinMode(DBG_INT, OUTPUT); 		// debugging
	pinMode(DBG_MPC, OUTPUT); 		// debugging
	pinMode(DBG_TIMER, OUTPUT);		// debugging
	pinMode(DBG_TIMERLEN, OUTPUT);		// debugging
#endif

	pinMode(ONBOARD_LED, OUTPUT);   // for onboard LED
	digitalWrite(ONBOARD_LED, HIGH);

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
	expanderWriteWord(MPC_SM8STATUS,   GPIOA, 0xFFFF);
	expanderWriteWord(MPC_WALLBUTTON,  GPIOA, 0xFFFF);

	// port direction
	expanderWriteBoth(MPC_MOTORRELAIS,IODIRA,0x00);		// OUTPUT
	expanderWriteBoth(MPC_SM8BUTTON,  IODIRA,0x00);		// OUTPUT
	expanderWriteBoth(MPC_SM8STATUS,  IODIRA,0xFF);		// INPUT
	expanderWriteBoth(MPC_WALLBUTTON, IODIRA,0xFF);		// INPUT

	// invert polarity
	expanderWriteBoth(MPC_SM8STATUS,  IOPOLA,0xFF);		// invert polarity of signal
	expanderWriteBoth(MPC_WALLBUTTON, IOPOLA,0xFF);		// invert polarity of signal

	// enable interrupts on input MPC
	expanderWriteBoth(MPC_SM8STATUS,  GPINTENA, 0xFF); 		// enable interrupts
	expanderWriteBoth(MPC_WALLBUTTON, GPINTENA, 0xFF); 		// enable interrupts

	// read from interrupt capture ports to clear them
	expanderRead(MPC_SM8STATUS,  INTCAPA);
	expanderRead(MPC_SM8STATUS,  INTCAPB);
	expanderRead(MPC_WALLBUTTON, INTCAPA);
	expanderRead(MPC_WALLBUTTON, INTCAPB);

	// pin 19 of MCP23017 is plugged into D2 of the Arduino which is interrupt 0
	pinMode(ISR_INPUT, INPUT);					// make sure input
	digitalWrite(ISR_INPUT, HIGH);				// enable pull-up as we have made the interrupt pins open drain

	attachInterrupt(digitalPinToInterrupt(2), extISR, FALLING);

	MsTimer2::set(TIMER_MS, timerISR); 				// 20ms period
	MsTimer2::start();

#ifdef WATCHDOG_ENABLED
	int countdownMS = Watchdog.enable(4000);
#ifdef DEBUG_OUTPUT
	Serial.print("Enabled the watchdog with max countdown of ");
	Serial.print(countdownMS, DEC);
	Serial.println(" milliseconds!");
	Serial.println();
#endif
#endif

#ifdef DEBUG_OUTPUT
	Serial.println("Setup done, starting main loop()");
#endif
}

/*
 * Function:	extISR
 * Return:
 * Arguments:
 * Description: Interrupt service routine
 * 				called when external pin D2 goes from 1 to 0
 */
void extISR()
{
#ifdef DEBUG_PINS
	digitalWrite(DBG_INT, !digitalRead(DBG_INT));  			// debugging
#endif
	isrTrigger = true;
	ledSignalTimer = millis();  							// remember when IR occured
	digitalWrite(ONBOARD_LED, !digitalRead(ONBOARD_LED));   // turn the LED on (HIGH is the voltage level)
}

/*
 * Function:	timerISR
 * Return:
 * Arguments:
 * Description: Timer Interrupt service routine
 * 				Wird alle 10 ms aufgerufen
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

	for(i=0; i<16; i++) {
		if( debSM8Status[i]>0 ) {
			debSM8Status[i] -= TIMER_MS;
		}
		else if( (curSM8Status & (1<<i)) != (irqSM8Status & (1<<i)) ) {
			curSM8Status = curSM8Status & (WORD)(~(1<<i)) | (irqSM8Status & (1<<i));
		}

		if( debWallButton[i]>0 ) {
			debWallButton[i] -= TIMER_MS;
		}
		else if( (curWallButton & (1<<i)) != (irqWallButton & (1<<i)) ) {
			curWallButton = curWallButton & (WORD)(~(1<<i)) | (irqWallButton & (1<<i));
		}
	}
	for(i=0; i<8; i++) {
		if( MotorCtrl[i] > 1 ) {
			--MotorCtrl[i];
		}
		else if( MotorCtrl[i] < -1 ) {
			++MotorCtrl[i];
		}
	}

#ifdef DEBUG_PINS
	digitalWrite(DBG_TIMERLEN, LOW);	// debugging
#endif

}

/*
 * Function:	handleMPCInt
 * Return:
 * Arguments:
 * Description: MPC Interrupt handling outside extISR
 *              Reads the input status from MPCs into vars
 *              Called from main loop, not from ISR
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
		for(i=0; i<16; i++) {
			if( (curSM8Status & (1<<i)) != (irqSM8Status & (1<<i)) ) {
				debSM8Status[i]=DEBOUNCE_TIME;
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
		for(i=0; i<16; i++) {
			if( (curWallButton & (1<<i)) != (irqWallButton & (1<<i)) ) {
				debWallButton[i]=DEBOUNCE_TIME;
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

#ifdef DEBUG_OUTPUT
String addLeadingSpace(char value, byte len)
{
	int val = value;
	int space = len - String(val).length();
	String returnValue = "";
	for (int i = 0; i < space; i++) {
		returnValue += " ";
	}
	returnValue += String(val);

	return returnValue;
}
String addLeadingZeros(byte value)
{
	int zeros = 8 - String(value,BIN).length();
	String returnValue = "";
	for (int i = 0; i < zeros; i++) {
		returnValue += "0";
	}
	returnValue += String(value, BIN);

	return returnValue;
}

void printWordBin(const char *head, WORD value)
{
	Serial.print(head);
	Serial.print( addLeadingZeros(value >> 8) );
	Serial.print(" ");
	Serial.print( addLeadingZeros(value & 0xff) );
	Serial.println();
}
#endif

bool newMotorDirection(char newDirection, char *newMotorDirection)
{
	char oldMotorCtrl = *newMotorDirection;

	if( newDirection>0 ) {
		if (*newMotorDirection<0) {	// Motor läuft auf Schliessen
			// Motor auf Öffnen mit Umschaltdelay
			*newMotorDirection = MOTOR_SWITCHOVER/TIMER_MS;
		}
		else if (*newMotorDirection>0) {	// Motor läuft auf Öffnen
			// Motor aus
			*newMotorDirection = 0;
		}
		else {	// Motor ist aus
			// Motor auf öffnen ohne Umschaltdelay
			*newMotorDirection = 1;
		}
	}
	else if ( newDirection<0 ) {
		if (*newMotorDirection>0) {	// Motor läuft auf Öffnen
			// Motor auf Schliessen mit Umschaltdelay
			*newMotorDirection = (MOTOR_SWITCHOVER/TIMER_MS) * -1;
		}
		else if (*newMotorDirection<0) {	// Motor läuft auf Schliessen
			// Motor aus
			*newMotorDirection = 0;
		}
		else {	// Motor ist aus
			// Motor auf Schliessen ohne Umschaltdelay
			*newMotorDirection = -1;
		}
	}
	else {
		// Motor aus
		*newMotorDirection = 0;
	}
	return (oldMotorCtrl != *newMotorDirection);
}


/*
 * Function:	ctrlInput
 * Return:
 * Arguments:
 * Description: Do the motor control by SM8 and key status
 */
void ctrlInput(void)
{
	/* FS20 SM8 Output */
	static WORD SM8Status;
	static WORD prevSM8Status = 0x0000;
	static WORD SM8StatusIgnore = 0x0000;

	/* Wall Button Output */
	static WORD WallButton;
	static WORD prevWallButton = 0x0000;
	static WORD WallButtonLocked = 0x0000;

	static WORD MotorRelais = 0x0000;
	static WORD prevMotorRelais = 0x0000;

	bool changeMotor;


	changeMotor = false;
	/* Wandtaster haben Vorrang vor SM8 Ausgänge */

	/* Lese SM8 Ausgänge
		* die unteren 8-bits sind "Öffnen" Befehle
		* die oberen 8-bits sind "Schliessen" Befehle
		* Flankengesteuert
			* Flanke von 0 nach 1 bedeuted: Motor Ein
			* Flanke von 1 nach 0 bedeuted: Motor Aus
			* Gleiche Flanken für Öffnen und Schliessen
			  sind ungültig
		  Tabelle
		  Fo Fc  Motor
		  0  0   Aus
		  1  0   Öffnen
		  0  1   Schliessen
		  1  1   Ignorieren
	*/
	if( prevSM8Status != curSM8Status ) {
		byte i;
		WORD SM8StatusSlope;
		WORD SM8StatusChange;

		SM8Status = curSM8Status;
		SM8StatusSlope = ~prevSM8Status & SM8Status;
		SM8StatusChange= prevSM8Status ^ SM8Status;

		if( bitRead(SM8StatusChange,i)!=0 ) {
			if ( bitRead(SM8StatusSlope,i)!=0 ) {
				MotorCtrl[i] = (MotorCtrl[i]<0) ? (MOTOR_SWITCHOVER/TIMER_MS) : 1;
			}
			else {
				MotorCtrl[i] = 0;
			}
		}
		else if( bitRead(SM8StatusChange,i+8)!=0 ) {
			if ( bitRead(SM8StatusSlope,i+8)!=0 ) {
				MotorCtrl[i] = (MotorCtrl[i]>0) ? (MOTOR_SWITCHOVER/TIMER_MS)*-1 : -1;
			}
			else {
				MotorCtrl[i] = 0;
			}
		}

#ifdef DEBUG_OUTPUT
		Serial.println();
		Serial.println("----------------------------------------");
		printWordBin("prevSM8Status:   ", prevSM8Status);
		printWordBin("SM8Status:       ", SM8Status);
		printWordBin("SM8StatusSlope:  ", SM8StatusSlope);
		printWordBin("SM8StatusChange: ", SM8StatusChange);
#endif

		prevSM8Status = curSM8Status;
		changeMotor = true;
	}

	/* Lese Wandtaster
		* die unteren 8-bits sind "Öffnen" Befehle
		* die oberen 8-bits sind "Schliessen" Befehle
		* Flankenänderung von 0 auf 1 schaltet Motor ein/aus
		* Flankenänderung von 1 auf 0 bewirkt nichts
		* Einzelpegel bewirkt nichts
		* Pegel Öffnen und Schliessen = 1:
		  - Motor Aus
		  - Taster verriegeln
		* Pegel Öffnen und Schliessen = 0:
		  - Taster entriegeln
	*/
	if( prevWallButton != curWallButton ) {
		byte i;

		WORD WallButtonSlope;
		WORD WallButtonChange;

		WallButton = curWallButton;
		WallButtonSlope = ~prevWallButton & WallButton;
		WallButtonChange = prevWallButton ^ WallButton;

		for(i=0; i<8; i++) {
			// Flankenänderung von 0 auf 1 schaltet Motor ein/aus
			if( bitRead(WallButtonChange,i)!=0 && bitRead(WallButtonSlope,i)!=0 ) {
				changeMotor=newMotorDirection(MOTOR_OPEN,  &MotorCtrl[i]);
			}
			else if( bitRead(WallButtonChange,i+8)!=0 && bitRead(WallButtonSlope,i+8)!=0 ) {
				changeMotor=newMotorDirection(MOTOR_CLOSE, &MotorCtrl[i]);
			}
			// Pegel Öffnen und Schliessen = 1:
			if( bitRead(WallButton,i)!=0 && bitRead(WallButton,i+8)!=0 ) {
				changeMotor=newMotorDirection(MOTOR_OFF,   &MotorCtrl[i]);
				bitSet(WallButtonLocked,i);
				bitSet(WallButtonLocked,i+8);
			}
			// Pegel Öffnen und Schliessen = 0:
			if( bitRead(WallButton,i)==0 && bitRead(WallButton,i+8)==0 ) {
				bitClear(WallButtonLocked,i);
				bitClear(WallButtonLocked,i+8);
			}
		}

#ifdef DEBUG_OUTPUT_WALLBUTTON
		Serial.println();
		Serial.println("----------------------------------------");
		printWordBin("prevWallButton:  ", prevWallButton);
		printWordBin("WallButton:      ", WallButton);
		printWordBin("WallButtonSlope: ", WallButtonSlope);
		printWordBin("WallButtonChange:", WallButtonChange);
		printWordBin("WallButtonLocked:", WallButtonLocked);

#endif

		prevWallButton = curWallButton;
	}

	if( changeMotor ) {
		byte i;

#ifdef DEBUG_OUTPUT
		Serial.println();
		Serial.println("----------------------------------------");
		Serial.println("   M1   M2   M3   M4   M5   M6   M7   M8");
		for(i=0; i<8; i++) {
			if (MotorCtrl[i]==0) {
				Serial.print("  off");
			}
			else {
				String mtime =  String(MotorCtrl[i]);
				Serial.print(addLeadingSpace(MotorCtrl[i], 5));
			}
		}
		Serial.println();
#endif

	}
}


// the loop function runs over and over again forever
WORD tmpSM8Status  = 0x0000;
WORD tmpWallButton  = 0x0000;

#ifdef DEBUG_OUTPUT
char liveToogle=0;
byte liveDots=0;
#endif

void loop()
{

	if ( isrTrigger ) {
		//Serial.println("ISR occured");
		handleMPCInt();
	}

	if ( (tmpSM8Status != curSM8Status) || (tmpWallButton != curWallButton) ) {
		//Serial.println("Status changed");
		ctrlInput();
		tmpSM8Status = curSM8Status;
		tmpWallButton = curWallButton;
	}

	// revers LED after 50 ms
	if ( (ledSignalTimer != 0) && (millis() > (ledSignalTimer + 50)) )
	{
		digitalWrite(ONBOARD_LED, !digitalRead(ONBOARD_LED));
		ledSignalTimer = 0;
	}
	else
	{
		// toggle LED to indicate main loop is running
		if ( millis()>(ledTimer+500) )
		{
			digitalWrite(ONBOARD_LED, !digitalRead(ONBOARD_LED));
			ledTimer = millis();
#ifdef DEBUG_OUTPUT
			if ((liveToogle--) < 1) {
				Serial.print(".");
				liveToogle=3;
				liveDots++;
				if( liveDots>76 ) {
					Serial.println();
					liveDots = 0;
				}
			}
#endif
			Watchdog.reset();
		}
	}

}
