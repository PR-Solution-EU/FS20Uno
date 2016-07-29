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
#include "I2C.h"

#define VERSION "1.00.0001"

#define MPC1    0x20    // MCP23017 #1 I2C address
#define MPC2    0x21    // MCP23017 #2 I2C address
#define MPC3    0x22    // MCP23017 #3 I2C address
#define MPC4    0x23    // MCP23017 #4 I2C address

#define MPC_MOTORRELAIS	MPC1
#define MPC_SM8BUTTON	MPC2
#define MPC_SM8STATUS	MPC3
#define MPC_WALLBUTTON	MPC4


#define ISR_INPUT 		2			// ISR Input from MPC = D2
#define DBG_INT 		12			// Debug PIN = D12
#define DBG_MPC 		11			// Debug PIN = D11
#define DBG_TIMER	 	10			// Debug PIN = D10
#define DBG_TIMERLEN 	9			// Debug PIN = D9
#define ONBOARD_LED 	LED_BUILTIN	// LED = D13



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


// FS20 output default timer in 1/10s
#define FS20_WIN_TIMEOUT	550
#define FS20_JL_TIMEOUT		200

/* FS20 SM8 Output */
byte SM8Out = 0;
byte prevSM8Out = 0;
// Count down SM8 output timeout
unsigned long SM8OutCounter[MAX_MOTORS*2];
// Contains Timeout values in 1/10s value;
unsigned long SM8OutTimeoutValue[MAX_MOTORS*2];


/* Wall pushbuttons */
byte MCPWallPB;	/* store value from MCP23017 wall pushbutton
						 * read within main loop, because we can't
						 * read MCP within handleMPCInt */
/* Wall pushbuttons can not be used directly from MCP
 * it must be first debounced.
 * The following variables will be used for debouncing
 */
byte WallPB = 0x0000;		// Debounced Wall PB bits
byte prevWallPB = 0xffff;
byte WallPBDebounce = 0;

/* Wall pushbutton values used by program main loop */
byte WallPBOpen = 0;
byte WallPBClose = 0;
byte PrevWallPBOpen = 0;
byte PrevWallPBClose = 0;

/* FS20 SM8 Inputs */
byte SM8In = 0x0000;
byte prevSM8In = 0x0000;
byte SM8InKey[MAX_MOTORS*2];		/* count down if SM8 input is active */


/* Motor control logic */
byte MotorPower = 0;
byte MotorDir = 0;
byte PrevMotorPower = 0;
byte PrevMotorDir = 0;
volatile char MotorPowerDelay[MAX_MOTORS] = {0,0,0,0,0,0,0,0};
volatile char MotorDirDelay[MAX_MOTORS] = {0,0,0,0,0,0,0,0};
bool prevRainClosed;

/* Rain detect input */
#define RAINBITMASK			0b11110000				// OR Bitmask for Close output during rain
bool RainDetect;
bool prevRainDetect;

/* Disable Rain detect input */
bool DisableRainDetect;
bool prevDisableRainDetect;


volatile unsigned int valMotorRelais = 0x0000;
volatile unsigned int valSM8Button   = 0x0000;

// values read from MPC port during MPC interrupt
volatile unsigned int irqSM8Status  = ~0x0000;
volatile unsigned int irqWallButton = ~0x0000;
// previous read values from MPC port
//volatile unsigned int valSM8Status  = 0x0000;
//volatile unsigned int valWallButton = 0x0000;
// values currently used within program
volatile unsigned int curSM8Status  = 0x0000;
volatile unsigned int curWallButton = 0x0000;
// debounce counter für keys
volatile char debSM8Status[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
volatile char debWallButton[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

volatile bool intSM8Status;		// true if SM8 Status (Output) has new data
volatile bool intWallButton;	// true if Wall button (Output) has new data



// time we turned LED on
volatile unsigned long ledSignalTimer = 0;
volatile unsigned long ledTimer = 0;


// the setup function runs once when you press reset or power the board
void setup()
{
	byte i;

	for(i=0; i<MAX_MOTORS;i++ ) {
		SM8OutTimeoutValue[i]   =
		SM8OutTimeoutValue[i+8] = (RAINBITMASK & (1<<i))?FS20_WIN_TIMEOUT:FS20_JL_TIMEOUT;
	}


	pinMode(DBG_INT, OUTPUT); 		// debugging
	pinMode(DBG_MPC, OUTPUT); 		// debugging
	pinMode(DBG_TIMER, OUTPUT);		// debugging
	pinMode(ONBOARD_LED, OUTPUT);   // for onboard LED

	digitalWrite(DBG_INT, LOW);  	// debugging
	digitalWrite(DBG_MPC, LOW);		// debugging
	digitalWrite(DBG_TIMER, LOW);	// debugging

	Wire.begin();
	Serial.begin(115200);
	Serial.println("Setup");

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

	// no interrupt yet
	intSM8Status = false;
	intWallButton = false;

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

	attachInterrupt(digitalPinToInterrupt(2), isr, FALLING);
	
	MsTimer2::set(TIMER_MS, timer10ms); 				// 20ms period
	MsTimer2::start();	

	Serial.print("Starting v");
	Serial.print(VERSION);
	Serial.println("...");

	int countdownMS = Watchdog.enable(4000);
	Serial.print("Enabled the watchdog with max countdown of ");
	Serial.print(countdownMS, DEC);
	Serial.println(" milliseconds!");
	Serial.println();
}

void timer10ms()
{
	byte i;

	digitalWrite(DBG_TIMER, !digitalRead(DBG_TIMER));	// debugging
	digitalWrite(DBG_TIMERLEN, HIGH);	// debugging
	for(i=0; i<16; i++) {
		if( debSM8Status[i]>0 ) {
			debSM8Status[i] -= TIMER_MS;
		}
		else if( (curSM8Status & (1<<i)) != (irqSM8Status & (1<<i)) ) {
			curSM8Status = curSM8Status & (unsigned int)(~(1<<i)) | (irqSM8Status & (1<<i));
		}

		if( debWallButton[i]>0 ) {
			debWallButton[i] -= TIMER_MS;
		}
		else if( (curWallButton & (1<<i)) != (irqWallButton & (1<<i)) ) {
			curWallButton = curWallButton & (unsigned int)(~(1<<i)) | (irqWallButton & (1<<i));
		}
	}
	for(i=0; i<MAX_MOTORS; i++) {
		if( MotorPowerDelay[i]>0 ) {
			MotorPowerDelay[i] -= TIMER_MS;
		}
		if( MotorDirDelay[i]>0 ) {
			MotorDirDelay[i] -= TIMER_MS;
		}
	}
	for(i=0; i<(MAX_MOTORS*2); i++ ) {
		if( SM8OutCounter[i]>0 ) {
			SM8OutCounter[i] -= TIMER_MS;
		}
	}
	
	digitalWrite(DBG_TIMERLEN, LOW);	// debugging
	
}

/*
 * Function:	handleMPCInt
 * Return:
 * Arguments:
 * Description: Interrupt service routine, called when pin D2 goes from 1 to 0
 *              Handle MPC Interrupts
 *              Reads the input status from MPCs into vars
 */
volatile bool isrTrigger = false;
void isr()
{
	digitalWrite(DBG_INT, !digitalRead(DBG_INT));  			// debugging
	isrTrigger = true;
	ledSignalTimer = millis();  							// remember when IR occured
	digitalWrite(ONBOARD_LED, !digitalRead(ONBOARD_LED));   // turn the LED on (HIGH is the voltage level)
}

void handleMPCInt()
{
	byte portValue;
	byte i;

	digitalWrite(DBG_INT, !digitalRead(DBG_INT));  		// debugging

	isrTrigger = false;
	
	if ( expanderReadWord(MPC_SM8STATUS, INFTFA) )
	{
		digitalWrite(DBG_MPC, HIGH);	// debugging
		irqSM8Status = expanderReadWord(MPC_SM8STATUS, INTCAPA);
		intSM8Status = true;
		for(i=0; i<16; i++) {
			if( (curSM8Status & (1<<i)) != (irqSM8Status & (1<<i)) ) {
				debSM8Status[i]=DEBOUNCE_TIME;
			}
		}
		digitalWrite(DBG_MPC, LOW);		// debugging
	}

	if ( expanderReadWord(MPC_WALLBUTTON, INFTFA) )
	{
		digitalWrite(DBG_MPC, HIGH);	// debugging
		irqWallButton = expanderReadWord(MPC_WALLBUTTON, INTCAPA);
		intWallButton = true;
		for(i=0; i<16; i++) {
			if( (curWallButton & (1<<i)) != (irqWallButton & (1<<i)) ) {
				debWallButton[i]=DEBOUNCE_TIME;
			}
		}
		digitalWrite(DBG_MPC, LOW);		// debugging
	}

	digitalWrite(DBG_INT, !digitalRead(DBG_INT));  		// debugging

}


void printCtrlMotor(unsigned int motorRelais)
{
	int i;
	String str;
	char c;
	
	/* Head
	 * Motor     8 7 6 5 4 3 2 1
	 * Direction 8 7 6 5 4 3 2 1 */
	str = String("8 7 6 5 4 3 2 1 ");
	for(i=0; i<8; i++) {
		if( motorRelais & i ) {
			c = '1';
		}
		else {
			c = '0';
		}
		str.setCharAt(i*2, c);
	}
	Serial.print("    Motor     ");
	Serial.println(str);

	str = String("8 7 6 5 4 3 2 1 ");
	for(i=8; i<16; i++) {
		if( motorRelais & i ) {
			c = '1';
		}
		else {
			c = '0';
		}
		str.setCharAt((i-8)*2, c);
	}
	Serial.print("    Direction ");
	Serial.println(str);
}

/*
 * Function:	ctrlMotor
 * Return:
 * Arguments:
 * Description: Do the motor control by SM8 and key status
 */

void ctrlMotor(void)
{
		byte SlopeOpen, SlopeClose;
		byte WallPBSlopeOpen, WallPBSlopeClose;
		byte MaskOpen, MaskClose;
		byte SM8OutOpen = 0;
		byte SM8OutClose = 0;
		byte PrevSM8OutOpen = 0;
		byte PrevSM8OutClose = 0;
		byte LatchOpen = 0;
		byte LatchClose = 0;
		byte i;
		byte MotorSDelay;
		byte MotorLDelay;
		byte DirDelay;
		byte tmpMotorPower;
		unsigned long t = millis();

		// Read FS20 SM8 output
		// lower 8-bits are 'Open', higher 8-bits are 'Close'
		if( intSM8Status )
		{
			byte SM8OutSlope;

			SM8Out = curSM8Status;
			SM8OutSlope = ~prevSM8Out & SM8Out;

			for( i=0; i<sizeof(SM8OutCounter); i++) {
				if( SM8OutSlope & (1<<i) ) {
					SM8OutCounter[i] = SM8OutTimeoutValue[i];
				}
			}
			prevSM8Out = SM8Out;
			intSM8Status = false;
		}

		// Read wall pushbuttons
		// lower 8-bits are 'Open', higher 8-bits are 'Close'
		if( intWallButton )
		{
			MCPWallPB = curWallButton;
			intWallButton = false;
		}

		SM8OutOpen  =  SM8Out & 0xff;
		SM8OutClose = (SM8Out>>8) & 0xff;

		WallPBOpen  =  WallPB & 0xff;
		WallPBClose = (WallPB>>8) & 0xff;

		// wall push button slope
		WallPBSlopeOpen	 = ~PrevWallPBOpen & WallPBOpen;
		WallPBSlopeClose = ~PrevWallPBClose & WallPBClose;

		// real output implemented as 2-phase shift register within isr2 routine

		// Power depends on low-high/high-low slope, not on level
		// Set only those bits to 1 where there was a 0->1 transition (Slope)
		SlopeOpen  = ~PrevSM8OutOpen  & SM8OutOpen;		// Remember which bits having a 0-> slope
		SlopeClose = ~PrevSM8OutClose & SM8OutClose;

		LatchOpen  |= SlopeOpen;
		LatchClose |= SlopeClose;

		// Delete those bits to 0 where there was a 1->0 transition (Slope)
		//	L	C~	=	~
		//	0	01	=0	1
		//	0	10	=0	1
		//	1	01	=1	0
		//	1	10	=0	1
		LatchOpen  &= ~(PrevSM8OutOpen  & ~SM8OutOpen);
		LatchClose &= ~(PrevSM8OutClose & ~SM8OutClose);

		// disable opposite directions on FS20 Inputs based by slope

		// Condition to push a FS20 In Key:
		// Out	Slope	Key
		// SM8OutOpen	SM8OutClose	SlopeOpen	SlopeClose	^	KeyOpen	KeyClose
		// 1			1			x			x			0	1		1
		// 1			1			0			1			1	1		0
		// 1			1			1			0			1	0		1
		MaskOpen   = (SM8OutOpen & SM8OutClose) & (SlopeOpen ^ SlopeClose) & SlopeClose;
		MaskClose  = (SM8OutOpen & SM8OutClose) & (SlopeOpen ^ SlopeClose) & SlopeOpen;
		MaskOpen  |= (SM8OutOpen & SM8OutClose) & ((SlopeOpen & SlopeClose) | (~SlopeOpen & ~SlopeClose));
		MaskClose |= (SM8OutOpen & SM8OutClose) & ((SlopeOpen & SlopeClose) | (~SlopeOpen & ~SlopeClose));
		SM8In |=   MaskOpen  | WallPBSlopeOpen | ((MaskClose | WallPBSlopeClose)<<8)
				   // If WallPBOpen & WallPBclose hit, switch motor out off if on
				 | SM8OutOpen & (WallPBOpen & WallPBClose) | ((SM8OutClose & (WallPBOpen & WallPBClose))<<8);


		// 2. condition to push FS20 key is the timeout
		for( i=0; i<sizeof(SM8OutCounter); i++) {
			// disable timeout counter for outputs which are already inactive
			if( (SM8Out & (1<<i)) == 0 ) {
				SM8OutCounter[i] = -1;
			}
			// time out and output still active
			if( SM8OutCounter[i]==0 && (SM8Out & (1<<i)) ) {
				SM8OutCounter[i] = -1;	// disable timeout counter
				SM8In |= (1<<i);		// push key for this ouput
			}
		}


		// disable opposite directions based by slope
		LatchOpen  &= ~SlopeClose;
		LatchClose &= ~SlopeOpen;

		if( RainDetect ) {
			prevRainClosed = RainDetect;
			LatchClose |=  RAINBITMASK;
			LatchOpen  &= ~RAINBITMASK;
		}
		else if ( prevRainClosed ) {
			prevRainClosed = false;
			LatchClose &= ~RAINBITMASK;
			LatchOpen  &= ~RAINBITMASK;
		}

		// power results from open XOR close:
		//	open	close	power
		//	0		0		off (0)
		//	0		1		on  (1)
		//	1		0		on  (1)
		//	1		1		invalid (off)
		MotorPower = (LatchOpen | LatchClose) & ~(LatchOpen & LatchClose);

		// direction results from open and close pin:
		//	open	close	dir
		//	0		0 		close (0)
		//	0		1 		close (0)
		//	1		0 		open  (1)
		//  1   	1		open  (1)
		MotorDir = LatchOpen;

		// Delay for Power on when Power previously was switched off and Dir has switchover
		// PP	P	PD	D
		//	0	1	x	~x
		MotorSDelay = (~PrevMotorPower & MotorPower) & (PrevMotorDir ^ MotorDir);

		// Delay for Power when Power is still on and Dir has switchover
		MotorLDelay = (PrevMotorPower & MotorPower) & (PrevMotorDir ^ MotorDir);

		// Delay for Power change when power switch from on to off and dir changed
		// PP	P	PD	D
		//	1	0	x	~x
		DirDelay = (PrevMotorPower & ~MotorPower) & (PrevMotorDir ^ MotorDir);

		for(i=0; i<MAX_MOTORS; i++) {
			if( MotorLDelay & (1<<i) ) {
				MotorPowerDelay[i] = MOTOR_SWITCHOVER;
			}
			else if( MotorSDelay & (1<<i) ) {
				MotorPowerDelay[i] = OPERATE_TIME;
			}
			if( DirDelay & (1<<i) ) {
				MotorDirDelay[i] = MOTOR_SWITCHOVER;
			}
		}

		if( PrevMotorPower != MotorPower ) {
			Serial.print("MotorPower: ");
			Serial.println(MotorPower,HEX);
		}
		if( PrevMotorDir != MotorDir ) {
			Serial.print("MotorDir: ");
			Serial.println(MotorDir,HEX);
		}

		// Remember last status
		PrevSM8OutOpen  = SM8OutOpen;
		PrevSM8OutClose = SM8OutClose;

		PrevWallPBOpen  = WallPBOpen;
		PrevWallPBClose = WallPBClose;

		PrevMotorDir   = MotorDir;
		PrevMotorPower = MotorPower;

		tmpMotorPower = MotorPower;


		for(i=0; i<MAX_MOTORS; i++) {
			if( MotorPowerDelay[i] > millis() ) {
				// do not power motor as long as power delay is > 0
				tmpMotorPower &= ~(1<<i);
			}
			if( MotorDirDelay[i] > millis() ) {
				// do not change power as long as dir delay is > 0
				tmpMotorPower = (tmpMotorPower & ~(1<<i)) | ( (byte)(valMotorRelais & 0x00FF) & ~(1<<i));
			}
		}

		if ( valMotorRelais != ((MotorDir<<8) | tmpMotorPower) ) {

			Serial.println("---------------------------------------");
			Serial.println("Old Motor control: ");
			Serial.println(valMotorRelais, HEX);
			// printCtrlMotor(valMotorRelais);
			
			valMotorRelais = (MotorDir<<8) | tmpMotorPower;
			Serial.println("New Motor control: ");
			Serial.println(valMotorRelais, HEX);
			// printCtrlMotor(valMotorRelais);
			
			expanderWriteWord(MPC_MOTORRELAIS, GPIOA, valMotorRelais);
		}

}


unsigned int tmpSM8Status  = 0x0000;
unsigned int tmpWallButton  = 0x0000;


// the loop function runs over and over again forever
void loop()
{
	if ( isrTrigger ) {
		handleMPCInt();
		ctrlMotor();
	}

	// turn LED off after 100 ms
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
			Watchdog.reset();
		}
	}
	
	// debug
	//~ if( tmpSM8Status != curSM8Status ) {
		//~ Serial.print("curSM8Status:  ");
		//~ Serial.println(curSM8Status, HEX);
		//~ tmpSM8Status = curSM8Status;
	//~ }
	//~ if( tmpWallButton != curWallButton ) {
		//~ Serial.print("curWallButton: ");
		//~ Serial.println(curWallButton, HEX);
		//~ tmpWallButton = curWallButton;
	//~ }
}
