/* ===================================================================
 * File:    FS20Uno.ino
 * Author:  Norbert Richter <mail@norbert-richter.info>
 * Project: Electric rooflight window and shutter controller
 *          with wallbuttons and FS20-SM8 control using Arduino Uno
 * Desc:    The control unit consists of
 *          * Arduino Uno
 *          * 4 I2C Portexpander MCP23017
 *          * 2 ELV FS20-SM8 8-channel receiver
 *          * 16 Relais to control 8 motors:
 *            * 8 Relais for motor on/off
 *            * 8 Relais for motot direction.
 *
 * Operation mode:
 * ==============
 * Features:
 * * Can control up to 4 electrical roof windows with shutter
 * * Control windows by using
 *   * two pushbuttons ("Open" and "Close")
 *   * two FS20-SM8 channel ("Open" and "Close" via FS20 sender)
 *   * command using serial interface
 *     (connected to a Serial2Ethernet interface like WIZNET or a
 *      RaspberryPi using ser2net it is also usable via network)
 *  * Optional rain sensor to close roof windows when it is raining
 *  * Optional restore window position after rain has gone.
 *
 * Functions:
 * * 2 pushbutton (wall button)
 *    * Opening a motor by pushing the "Open" pushbutton for a short
 *      time.
 *      When using the "Open" pushbutton again when motor runs in "Open"
 *      direction will stop the motor.
 *    * Closing a motor by pushing the "Close" pushbutton for a short
 *      time.
 *      When using the "Close" pushbutton again when motor runs in
 *      "Close" direction will stop the motor.
 *    * Pushing both buttons together will stop the current running
 *      direction anyway.
 *    * If pushing a button longer than 10 sec, the pressed time until
 *      releasing the button will be used as new motor running time.
 *
 * * 2 FS20-SM8 channel
 *    * Opening a motor by enable the related FS20-SM8 "Open" channel.
 *      Stop this mode by enable again the related FS20-SM8 "Open"
 *      channel.
 *    * Closing a motor by enable the related FS20-SM8 "Close" channel.
 *      Stop this mode by enable again the related FS20-SM8 "Close"
 *      channel.
 *
 * * Serial interface with simple commands
 *
 *
 * Two FS20-SM8 8 channel recevier are used as FS20 receiver, so we
 * have 16 channels to control 8 motors in open and close direction.
 *
 * With this setup we control 4 electric roof windows with electric
 * shutter having 24V direct current motors:
 * - one motor open/close the window
 * - one motor open/close the shutter (jalousie)
 *
 * Using the Arduino Uno we translate the user inputs from pushbuttons
 * and FS20-SM8 channels to motor signals for
 * - Off
 * - Opening
 * - Closing
 *
 * For further functionality (e. g. command to position the motor to a
 * desired position), we keep the following attributes:
 * * Motor type (window or jalousie)
 * * Motor running time (from open <-> close position)
 *
 * I2C MCP23017 port enhancer connection:
 * MPC1
 *   Port A (Output): Relais Motor 1-8 ON/OFF
 *   Port B (Output): Relais Motor 1-8 Direction
 * MPC2
 *   Port A (Output): FS20-SM8 #1 Key 1-8    ("OPEN")
 *   Port B (Output): FS20-SM8 #2 Key 1-8    ("CLOSE")
 * MPC3
 *   Port A (Input) : FS20-SM8 #1 Status 1-8 ("OPEN")
 *   Port B (Input) : FS20-SM8 #2 Status 1-8 ("CLOSE")
 *   FS20-SM8 outputs connect signal to GND (0=active)
 * MPC4
 *   Port A (Input) : Pushbutton             ("OPEN")
 *   Port B (Input) : Pushbutton             ("CLOSE")
 *
 * Two additonal inputs are used as rains sensor and rain sensor enabled.
 *
 * Motor control:
 * ----------------------------
 * Each motor will be completyl controlled by Arduino Uno. There are
 * no direct connections of FS20-SM8 channel outputs or pushbutton to
 * motor relais control signal.
 *
 * Each motor used two additional porperties:
 * * Motor type (window or jalousie)
 * * Motor running time (from open <-> close position)
 * Both properties are set to default values using the program defines
 * DEFAULT_MOTORTYPE, MOTOR_WINDOW_MAXRUNTIME and MOTOR_JALOUSIE_MAXRUNTIME.
 * They can be changed during operation with the commands  "MOTORTYPE"
 * and "MOTORTIME". Values are stored in EEPROM.
 * Motors having property WINDOW are closed when rain sensor is enable
 * and rain is active.
 * Each motor will be automatically switch off after a maximum runtime
 * including an additonal overtravel time. The motor runtime is used to
 * properly calculate the current position between open and close.
 * To be sure that a window will be realy closed, each motor can have
 * and optional overtravel time.
 * The runtime and overtime values can be set via program defaults, by
 * using the serial command interface or (runtime only) by using the
 * auto-learn function pressing the related pushbutton for longer than
 * 10 seconds. Keep the pushbutton as long as the window or shutter
 * moves, then release it.
 *
 * Motor protection:
 * Due to the electromagnetic induction effect of the direct current
 * motors, there are several protection implemented:
 * * The motor voltage is switched on only after direction relais
 *   was switched into proper direction and attend relais operation
 *   time (OPERATE_TIME).
 * * Also motor is switched off first if the motor is still running
 *   and we need to reverse the direction attend relais operation
 *   time (OPERATE_TIME). Before motor is switch on in reverse direction
 *   we also attend an additonal MOTOR_SWITCHOVER time to protect motor
 *   itself.
 *
 * Pushbutton:
 * ----------------------------
 * Pusbutton connects a pull-up input to GND (means 0=active).
 * Pushbutton function:
 * - Button "OPEN":  Switch motor to "Open",
 *                   pushng a second time switch motor off.
 * - Button "CLOSE": Switch motor to "Close",
 *                   pushng a second time switch motor off.
 * - Push both buttons together also swich motor off.
 * - Keep a pushbutton pressed longer than 10 sec, activates a auto-
 *   learn function to measure the real runtime for a motor needs to
 *   move a window or shutter.
 * Pusbuttons having a higher priority than FS20-SM8 channel outputs
 * so if a FS20-SM8 channel is active, pushbutton overwrite the state
 * and possibly switchs FS20-SM8 channel off.
 *
 * FS20-SM8:
 * ----------------------------
 * FS20-SM8 channel outputs are also used to control the motors. Because
 * we connect the FS20-SM8 channel key inputs also to Arduino, we are
 * able to activly control the FS20-SM8 channel by soft-push the
 * FS20-SM8 key. So we can switch FS20-SM8 channel, which are switch
 * by key pressing or by FS20 control signal.
 * FS20 control conditions:
 * * two active FS20-SM8 channels, which are related to the same motor
 *   will be decoupled:
 *   If a FS20-SM8 output will be go active even the related other
 *   FS20-SM8 channel output is active, the other FS20-SM8 channel
 *   will be switch off - means the latest trigger wins.
 * * only rising slope will activate the function.
 * * only falling slope will deactive the function.
 * * Pushbutton using will overwrite FS20-SM8 channel, so it will be
 *   disabled if necessary.
 *
 * Rainsensor
 * ----------------------------
 * Rain sensor can be used to automatically close all motors of type
 * WINDOW. An optional RESUME function will reopen all windows which
 * are not closed when rain begins to the previous position.
 * The resume functionaliy only works properly, if the motor runtime
 * are properly set to a real value.
 *
 * Rain sensor input:
 * Can be connect to input "RAIN_INPUT", the active level polarity can
 * be set using "RAIN_INPUT_ACTIVE".
 *
 * Rain sensor enable input:
 * A second input connected to "RAIN_ENABLE" can be used to enable/dis-
 * able Rain sensor input. This is usefull if you want to temporarly
 * switch the rain sensor function off using a wall button.
 * The active level polarity can be set using "RAIN_ENABLE_ACTIVE".
 * If input "RAIN_ENABLE" is inactive, than rain sensor input signal
 * will be ignored.
 *
 * Control-Kommando "RAIN"
 * Both inputs for rain can be overruled by control command "RAIN":
 * * RAIN WET|ON|DRY|OFF
 *     Simulate the rain input signal: WET|ON =Raining
 *                                     DRY|OFF=Inaktiv
 * * RAIN ENABLE|DISABLE
 *     Simulate rain enable input:    ENABLE =Rain function enabled
 *                                    DISABLE=Rain function disabled
 * * RAIN AUTO
 *     Because using of one of the above simulation commands switch
 *     of the automatic mode, this command re-enables the physical input
 *
 * * RAIN RESUME <s>/FORGET
 *     RESUME <s> reopen all windows after rain has gone to the previous
 *     position after a delay of <s> seconds.
 *     The delay <s> can be used to prevent that the windows are closed
 *     and reopened in short times when it is not permanently raining.
 *     The resume functonaly can be disabled by using "RAIN FORGET". In
 *     this case the windows are close on rain but not reopened after
 *     rain has gone.
 * ===================================================================*/

#include <Arduino.h>
#include <errno.h>
#include <EEPROM.h>				// https://www.arduino.cc/en/Reference/EEPROM
#include <Wire.h>				// https://www.arduino.cc/en/Reference/Wire
#include <MsTimer2.h>			// http://playground.arduino.cc/Main/MsTimer2
#include <Bounce2.h>			// https://github.com/thomasfredericks/Bounce2
#include <Adafruit_SleepyDog.h> // https://github.com/adafruit/Adafruit_SleepyDog

// Eigene includes
#include "SerialCommand.h"		// https://github.com/scogswell/ArduinoSerialCommand
#include "Xtea.h"
#include "FS20Uno.h"
#include "I2C.h"

#define PROGRAM F("FS20Uno")	// program name
#define VERSION F("4.04")		// program version
#include "REVISION.h"			// Build (changed from git hook)
#define DATAVERSION 126			// can be used to invalidate EEPROM data

#define WATCHDOG_ENABLED		// #undef to disable watchdog function


/* ===================================================================
 * DEBUG SETTINGS
 * ===================================================================*/

/* Uncomment next line to test the Arduino library millis() overflow
 * millis() will then start with TEST_MILLIS_TIMER [ms]
 * before first overflow (max 49 days 17:02:47.295),
 */
//#define TEST_MILLIS_TIMER	30000L	// start with 30 sec before overflow

/* Debug outputs:
 * You can not use all debug outputs together because default program
 * size is to big. Use the debug code size value to estimate which
 * debug may be enabled together.
 * Note: Debugging disables all the build-in online help */
#undef DEBUG_PINS			// 0,2 kB: enable debug output pins
#undef DEBUG_RUNTIME		// 0.4 kB: enable runtime debugging
#undef DEBUG_SETUP			// 0.1 kB: enable setup related outputs
#undef DEBUG_WATCHDOG		// 0.2 kB: enable watchdog related outputs
#undef DEBUG_EEPROM			// 2.0 kB: enable EEPROM related outputs
#undef DEBUG_SERIALCMD		// 0.3 kB: enable Serial cmd interface related outputs
#undef DEBUG_PASSWD		// 0.5 kB: enable password related outputs
#undef DEBUG_CMD_RESTORE	// 0.1 kB: enable CMD RESTORE related outputs
#undef DEBUG_SM8STATUS		// 1.1 kB: enable FS20-SM8-output related output
#undef DEBUG_PUSHBUTTON		// 0.9 kB: enable pushbutton related output
#undef DEBUG_SM8OUTPUT		// 0.5 kB: enable FS20-SM8-key related output
#undef DEBUG_MOTOR			// 1.7 kB: enable motor control related output
#undef DEBUG_MOTOR_DETAILS	// 0.5 kB: enable motor control details output
#undef DEBUG_RAIN			// 1.0 kB: enable rain sensor related output
#undef DEBUG_ALIVE			// 0.1 kB: enable program alive signal output

#if defined(DEBUG_PINS) || \
	defined(DEBUG_RUNTIME) || \
	defined(DEBUG_SETUP) || \
	defined(DEBUG_WATCHDOG) || \
	defined(DEBUG_EEPROM) || \
	defined(DEBUG_SERIALCMD) || \
	defined(DEBUG_PASSWD) || \
	defined(DEBUG_CMD_RESTORE) || \
	defined(DEBUG_SM8STATUS) || \
	defined(DEBUG_PUSHBUTTON) || \
	defined(DEBUG_SM8OUTPUT) || \
	defined(DEBUG_MOTOR) || \
	defined(DEBUG_MOTOR_DETAILS) || \
	defined(DEBUG_RAIN) || \
	defined(DEBUG_ALIVE)
	#define DEBUG_OUTPUT
	#undef WATCHDOG_ENABLED
#endif

#ifdef DEBUG_PINS
	#define DBG_INT 			12			// Debug PIN = D12
	#define DBG_MPC 			11			// Debug PIN = D11
	#define DBG_TIMER	 		10			// Debug PIN = D10
#endif

/* Run time measurement */
#ifdef DEBUG_RUNTIME
	#define DEBUG_RUNTIME_START(val) unsigned long val = micros();
	#define DEBUG_RUNTIME_END(funcName,val) printRuntime(F(funcName), val);
#else
	#define DEBUG_RUNTIME_START(val)
	#define DEBUG_RUNTIME_END(funcName,val)
#endif



/* ===================================================================
 * Global Vars
 * ===================================================================*/

// Uptime
volatile unsigned int millisOverflow = 0;
unsigned long prevMillis = 0;
unsigned long savedOperationTime = 0;


// Interrup soft enable flags
volatile bool extISREnabled   = false;
volatile bool timerISREnabled = false;
volatile bool isrTrigger      = false;

// timer vars
TIMER ledTimer;
LEDPATTERN currentLEDPattern;
byte     currentLEDBitCount;
TIMER 	runTimer;
#if (1000/TIMER_MS)<=UINT8_MAX
byte	secTimerCount;
#elif (1000/TIMER_MS)<=UINT16_MAX
WORD	secTimerCount;
#else
DWORD	secTimerCount;
#endif



// MPC output data

/* Motor
 * Lower Bits: Motor on/off
 * Upper Bits: Motor direction */
volatile IOBITS valMotorRelais = IOBITS_ZERO;	// wanted value
volatile IOBITS regMotorRelais = IOBITS_ZERO;	// value put to MPC

/* SM8 Keys
 * Lower Bits: Motor opening
 * Upper Bits: Motor closing */
volatile IOBITS valSM8Button   = ~IOBITS_ZERO;

/* Values read out from MCP23017 when MPC triggers an IRQ
 * Lower Bits: Motor opening
 * Upper Bits: Motor closing */
volatile IOBITS irqSM8Status   = IOBITS_ZERO;
volatile IOBITS irqPushButton  = IOBITS_ZERO;

/* Values used within program logic
 * Lower Bits: Motor opening
 * Upper Bits: Motor closing */
volatile IOBITS curSM8Status    = IOBITS_ZERO;
         IOBITS SM8StatusIgnore = IOBITS_ZERO;
volatile IOBITS curPushButton   = IOBITS_ZERO;

/* Debounce counter for FS20-SM8 outputs and pushbuttons */
volatile char debSM8Status[IOBITS_CNT]  = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
volatile char debPushButton[IOBITS_CNT] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


/* Motor control values:
 *  0: Motor off
 * >0: Motor opening   1=immediately, >1=delay in ms before opening)
 * <0: Motor closing  -1=immediately, <1=abs(delay) in ms before closing)
 *                       delay values are in ms/TIMER_MS
 */
volatile MOTOR_CTRL  MotorCtrl[MAX_MOTORS]	= {MOTOR_OFF,MOTOR_OFF,MOTOR_OFF,MOTOR_OFF,MOTOR_OFF,MOTOR_OFF,MOTOR_OFF,MOTOR_OFF};
// Bitmask for motors which are switch off during IRQ
volatile MOTORBITS sendStatusMOTOR_OFF;

/* Motor timeout counter: Counted down within timer IRQ and switches
 * motor off if getting 0 */
volatile MOTOR_TIMER MotorTimeout[MAX_MOTORS] = {0,0,0,0,0,0,0,0};

/* Current motor positions (counter values) */
volatile MOTOR_TIMER MotorPosition[MAX_MOTORS] = {0,0,0,0,0,0,0,0};

/* Requested motor positions (counter values) or NO_POSITION if inactive */
volatile MOTOR_TIMER destMotorPosition[MAX_MOTORS] = {NO_POSITION,NO_POSITION,NO_POSITION,NO_POSITION,NO_POSITION,NO_POSITION,NO_POSITION,NO_POSITION};

/* Stored motor positions (percent values) when rain starts */
volatile byte resumeMotorPosition[MAX_MOTORS] = {NO_RESUME_POSITION,NO_RESUME_POSITION,NO_RESUME_POSITION,NO_RESUME_POSITION,NO_RESUME_POSITION,NO_RESUME_POSITION,NO_RESUME_POSITION,NO_RESUME_POSITION};
/* Delay to count down before resumes window position after rain */
volatile WORD resumeDelay = NO_RESUME_DELAY;

/* Timer for auto-learn function: Counts the time of pressed pushbutton */
TIMER PushButtonTimer[MAX_MOTORS] = {0,0,0,0,0,0,0,0};

/* SM8 key control "pushed"-time */
volatile SM8_TIMEOUT SM8Timeout[IOBITS_CNT] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};



/* Rain sensor */
/* Soft rain detection (not permanently stored in EEPROM) */
bool softRainInput = false;
/* Rain sensor input and enable input */
Bounce debEnable = Bounce();
Bounce debInput = Bounce();
/* Rain flag */
bool isRaining = false;

/* EEPROM Variablen */
struct MYEEPROM eeprom;

// Command interface login status
bool prevUnlocked;				// remember value for output new status
volatile bool cmdUnlocked;		// global cmd interface unlock state
volatile WORD cmdLoginTimeout;	// auto logout timer


/* Strings in PROGMEM */
const char fstrON[]			PROGMEM = "ON";
const char fstrOFF[]		PROGMEM = "OFF";



/* ===================================================================
 * Program
 * ===================================================================*/

#ifdef DEBUG_RUNTIME
void printRuntime(const __FlashStringHelper *funcName, unsigned long starttime)
{
	unsigned long duration = micros() - starttime;

	SerialPrintUptime();
	SerialPrintf(F("RUNTIME - "));
	Serial.print(funcName);
	SerialPrintfln(F(" duration: %lu.%03lu ms"), duration / 1000L, duration % 1000L);
}
#endif


/* ===================================================================
 * Function:    setup
 * Return:
 * Arguments:
 * Description: setup function runs once
 *              when you press reset or power the board
 * ===================================================================*/
void setup()
{
	Watchdog.disable();

	#ifdef TEST_MILLIS_TIMER
    cli(); //halt the interrupts
    //timer0_millis =  UINT32_MAX - TEST_MILLIS_TIMER; //change the value of the register
    timer0_millis =  (3600L-30L)*1000L;
    sei(); //re-enable the interrupts
    #endif

	DEBUG_RUNTIME_START(msSetup);

	// indicate setup started
	pinMode(STATUS_LED, OUTPUT);   // for onboard LED
	digitalWrite(STATUS_LED, HIGH);

	Serial.begin(SERIAL_BAUDRATE);
	Serial.println();

	// Init random generator
	long r = millis();
	#ifdef DEBUG_SETUP
	SerialTimePrintfln(F("setup - r=%ld"), r);
	#endif
	int an = 0;
	#ifdef RANDOM_SEED_ANALOG_READ1
	an = analogRead(RANDOM_SEED_ANALOG_READ1);
	r = an;
	#ifdef DEBUG_SETUP
	SerialTimePrintfln(F("setup - AN1: %d"), an);
	#endif
	#endif
	#ifdef RANDOM_SEED_ANALOG_READ2
	an = analogRead(RANDOM_SEED_ANALOG_READ2);
	r <<= 10;
	r |= an;
	#ifdef DEBUG_SETUP
	SerialTimePrintfln(F("setup - AN2: %d"), an);
	#endif
	#endif
	#ifdef RANDOM_SEED_ANALOG_READ3
	an = analogRead(RANDOM_SEED_ANALOG_READ3);
	r <<= 10;
	r |= an;
	#ifdef DEBUG_SETUP
	SerialTimePrintfln(F("setup - AN3: %d"), an);
	#endif
	#endif
	#ifdef DEBUG_SETUP
	SerialTimePrintfln(F("setup - randomSeed(%ld)"), r);
	#endif
	randomSeed(r);

	// Read EEPROM program setting
	eepromInitVars();

	// Init program vars
	initVars();

	// Init command interface
	setupSerialCommand();

	if ( !eeprom.SendStatus ) {
		printProgramInfo(true);
	}
	sendStatus(false,SYSTEM, F("%S %S.%S"), PROGRAM, VERSION, REVISION);

	#ifdef DEBUG_OUTPUT
	SerialTimePrintfln(F("setup - Debug output enabled"));
	#endif
	#ifdef DEBUG_PINS
	SerialTimePrintfln(F("setup - Debug pins enabled"));
	pinMode(DBG_INT, OUTPUT);
	pinMode(DBG_MPC, OUTPUT);
	pinMode(DBG_TIMER, OUTPUT);
	#endif

	// Input pins pulled-up
	pinMode(RAIN_ENABLE, INPUT_PULLUP);
	digitalWrite(RAIN_ENABLE, RAIN_ENABLE_ACTIVE==0 ? LOW:HIGH );
	// After setting up the button, setup the Bounce instance
	debEnable.attach(RAIN_ENABLE);
	debEnable.interval(RAIN_DEBOUNCE_TIME); // interval in ms

	pinMode(RAIN_INPUT, INPUT_PULLUP);
	digitalWrite(RAIN_INPUT, RAIN_INPUT_ACTIVE==0 ? HIGH:LOW);
	// After setting up the button, setup the Bounce instance
	debInput.attach(RAIN_INPUT);
	debInput.interval(RAIN_DEBOUNCE_TIME);


	#ifdef DEBUG_PINS
	digitalWrite(DBG_INT, LOW);
	digitalWrite(DBG_MPC, LOW);
	digitalWrite(DBG_TIMER, LOW);
	#endif

	Wire.begin();
	// expander configuration register
	expanderWriteBoth(MPC_MOTORRELAIS, IOCON, 0b00100100);	//                    sequential mode, INT Open-drain output
	expanderWriteBoth(MPC_SM8BUTTON,   IOCON, 0b00100100);	//                    sequential mode, INT Open-drain output
	expanderWriteBoth(MPC_SM8STATUS,   IOCON, 0b01100100);	// mirror interrupts, sequential mode, INT Open-drain output
	expanderWriteBoth(MPC_PUSHBUTTON,  IOCON, 0b01100100);	// mirror interrupts, sequential mode, INT Open-drain output

	// enable pull-up on switches
	expanderWriteBoth(MPC_MOTORRELAIS, GPPU, 0xFF);  	// pull-up resistor A/B
	expanderWriteBoth(MPC_SM8BUTTON,   GPPU, 0xFF);  	// pull-up resistor A/B
	expanderWriteBoth(MPC_SM8STATUS,   GPPU, 0xFF);  	// pull-up resistor A/B
	expanderWriteBoth(MPC_PUSHBUTTON,  GPPU, 0xFF);  	// pull-up resistor A/B

	// port data
	expanderWriteWord(MPC_MOTORRELAIS, GPIO, regMotorRelais);
	expanderWriteWord(MPC_SM8BUTTON,   GPIO, valSM8Button);
	expanderWriteWord(MPC_SM8STATUS,   GPIO, ~IOBITS_ZERO);
	expanderWriteWord(MPC_PUSHBUTTON,  GPIO, ~IOBITS_ZERO);

	// port direction
	expanderWriteBoth(MPC_MOTORRELAIS, IODIR, 0x00);	// OUTPUT
	expanderWriteBoth(MPC_SM8BUTTON,   IODIR, 0x00);	// OUTPUT
	expanderWriteBoth(MPC_SM8STATUS,   IODIR, 0xFF);	// INPUT
	expanderWriteBoth(MPC_PUSHBUTTON,  IODIR, 0xFF);	// INPUT

	// invert polarity
	expanderWriteBoth(MPC_SM8STATUS,   IOPOL, 0xFF);	// invert polarity of signal
	expanderWriteBoth(MPC_PUSHBUTTON,  IOPOL, 0xFF);	// invert polarity of signal

	// interrupt on change to previous one
	expanderWriteBoth(MPC_PUSHBUTTON,  INTCON, 0x00);	// enable interrupts

	// enable interrupts on input MPC
	expanderWriteBoth(MPC_SM8STATUS,   GPINTEN, 0xFF);	// enable interrupts
	expanderWriteBoth(MPC_PUSHBUTTON,  GPINTEN, 0xFF);	// enable interrupts

	// read from interrupt capture ports to clear them
	expanderRead(MPC_SM8STATUS,  INTCAPA);
	expanderRead(MPC_SM8STATUS,  INTCAPB);
	expanderRead(MPC_PUSHBUTTON, INTCAPA);
	expanderRead(MPC_PUSHBUTTON, INTCAPB);


	// MPC23017 pin 19 are connected together to an Ardunino IRQ input
	pinMode(MPC_INT_INPUT, INPUT_PULLUP);// make int input
	digitalWrite(MPC_INT_INPUT, HIGH);	// enable pull-up as we have made
										// the interrupt pins open drain

	// Disable controller interrupts
	noInterrupts();
	// Disable interrupt processing too
	extISREnabled = false;
	timerISREnabled = false;

	// External interrupt
	attachInterrupt(digitalPinToInterrupt(MPC_INT_INPUT), extISR, FALLING);

	// Timer2 interrupt
	MsTimer2::set(TIMER_MS, timerISR);
	MsTimer2::start();

	// Enable controller interrupts
	interrupts();

	// Init watchdog
	watchdogInit();

	// Reset FS20-SM8
	clrSM8Status();

	// Clear interrupt flag register by reading data
	curSM8Status  = expanderReadWord(MPC_SM8STATUS,  GPIO);
	curPushButton = expanderReadWord(MPC_PUSHBUTTON, GPIO);

	// clear interrupt capture register by reading
	expanderReadWord(MPC_SM8STATUS,  INTCAP);
	expanderReadWord(MPC_PUSHBUTTON, INTCAP);

	// clear again interrupt flag register by reading flag register
	expanderReadWord(MPC_SM8STATUS,  INFTF);
	expanderReadWord(MPC_PUSHBUTTON, INFTF);

	// Enable interrupt processing
	extISREnabled   = true;
	timerISREnabled = true;

	prevMillis = millis();

	// Finished
	digitalWrite(STATUS_LED, LOW);

	#ifdef DEBUG_SETUP
	SerialTimePrintfln(F("setup - done, starting main loop()"));
	#endif
	DEBUG_RUNTIME_END("setup()",msSetup);

	// Manual settings of single EEPROM vars
	//~ eeprom.OperatingHours = 0;
	//~ eepromWriteVars();

	sendStatus(false,SYSTEM, F("START"));

}

/* ===================================================================
 * Function:    initVars
 * Return:
 * Arguments:
 * Description: Init program vars
 * ===================================================================*/
void initVars()
{
	byte i;

	for(i=0; i<MAX_MOTORS; i++) {
		MotorPosition[i] = eeprom.MotorPosition[i];
	}

	// Variablen initalisieren
	currentLEDPattern = eeprom.LEDPatternNormal;
	currentLEDBitCount = 0;
	ledTimer = millis() + (TIMER)eeprom.LEDBitLenght;

	runTimer = millis() + (TIMER)ALIVE_TIMER;

	secTimerCount = 0;

	sendStatusMOTOR_OFF = 0;

	cmdUnlocked = false;
	prevUnlocked = false;
	cmdLoginTimeout = 0;
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
	if ( extISREnabled )
	{
		#ifdef DEBUG_PINS
		digitalWrite(DBG_INT, !digitalRead(DBG_INT));
		#endif
		isrTrigger = true;
	}
}

/* ===================================================================
 * Function:    timerISR
 * Return:
 * Arguments:
 * Description: Timer Interrupt service routine
 *              called every TIMER_MS ms
 * ===================================================================*/
void timerISR()
{
	if ( timerISREnabled )
	{
		byte i;

		#ifdef DEBUG_PINS
		digitalWrite(DBG_TIMER, !digitalRead(DBG_TIMER));
		#endif

		// Check millis() timer overflow
		if ( millis() < prevMillis ) {
			millisOverflow++;
			prevMillis = millis();
		}

		// Delay for rain RESUME command
		if ( resumeDelay!=NO_RESUME_DELAY && resumeDelay>0 ) {
			resumeDelay--;
		}

		for (i = 0; i < IOBITS_CNT; i++) {
			// Debounce FS20-SM8 outputs
			if ( debSM8Status[i] > 0 ) {
				debSM8Status[i]--;
			}
			else {
				bitWrite(curSM8Status, i, bitRead(irqSM8Status, i));
			}

			// Debounce pushbuttons
			if ( debPushButton[i] > 0 ) {
				debPushButton[i]--;
			}
			else {
				bitWrite(curPushButton, i, bitRead(irqPushButton, i));
			}

			/* SM8 key control timeout
			 * FS20-SM8 key control press signal must be carefully
			 * not longer than a short time, because otherwise the
			 * FS20-SM8 channel will go into programming mode (> 5 s) */
			if ( SM8Timeout[i] > 0 ) {
				if ( --SM8Timeout[i] == 0 ) {
					// SM8 key timer timout, reset FS20-SM8 key output
					bitSet(valSM8Button, i);
				}
			}

		}

		// Control MPC motor bits
		// (MPC register will not be written within IRQ service rouutine)
		for (i = 0; i < MAX_MOTORS; i++) {
			// Measure runtime
			if ( bitRead(regMotorRelais, i) ) {
				// Motor is on, determine direction
				if ( bitRead(regMotorRelais, i + MAX_MOTORS) ) {
					// Motor runs in open direction
					if ( MotorPosition[i] < (eeprom.MaxRuntime[i] / TIMER_MS) ) {
						++MotorPosition[i];
					}
				}
				else {
					// Motor runs in close direction
					if ( MotorPosition[i]>0 ) {
						--MotorPosition[i];
					}
				}
				// If a motor destination position is set, check it
				if ( destMotorPosition[i] != NO_POSITION ) {
					// Motor position achieved?
					if ( MotorPosition[i] == destMotorPosition[i] ) {
						// Motor OFF
						if ( MotorCtrl[i] != MOTOR_OFF ) {
							bitSet(sendStatusMOTOR_OFF, i);
						}
						MotorCtrl[i] = MOTOR_OFF;
						// Clear destination position
						destMotorPosition[i] = NO_POSITION;
					}
				}

			}

			// Check motor timeout
			if ( MotorTimeout[i] > 0 ) {
				--MotorTimeout[i];
				if ( MotorTimeout[i] == 0 ) {
					// Motor timeout, switch motor off
					if ( MotorCtrl[i] != MOTOR_OFF ) {
						bitSet(sendStatusMOTOR_OFF, i);
					}
					MotorCtrl[i] = MOTOR_OFF;
				}
			}

			// Motor delay control
			if ( MotorCtrl[i] > MOTOR_OPEN ) {
				--MotorCtrl[i];
				// Motor opening, motor off
				bitSet(valMotorRelais, i + MAX_MOTORS);
				bitClear(valMotorRelais, i);
			}
			else if ( MotorCtrl[i] < MOTOR_CLOSE ) {
				++MotorCtrl[i];
				// Motor closing, motor off
				bitClear(valMotorRelais, i + MAX_MOTORS);
				bitClear(valMotorRelais, i);
			}
			else {
				// No more delay, motor should be set to a defined state
				if ( MotorCtrl[i] == MOTOR_OPEN ) {
					// Motor to opening, motor on
					bitSet(valMotorRelais, i + MAX_MOTORS);
					bitSet(valMotorRelais, i);
				}
				else if ( MotorCtrl[i] == MOTOR_CLOSE ) {
					// Motor to closing, motor on
					bitClear(valMotorRelais, i + MAX_MOTORS);
					bitSet(valMotorRelais, i);
				}
				else if ( MotorCtrl[i] == MOTOR_OFF ) {
					// Motor off, direction relais to close
					bitClear(valMotorRelais, i);
					bitClear(valMotorRelais, i + MAX_MOTORS);
				}
			}
		}

		// Timer in sec steps
		if ( ++secTimerCount>=(1000/TIMER_MS) ) {
			// executed every second

			if ( cmdLoginTimeout>0 ) {
				cmdLoginTimeout--;
				if ( cmdLoginTimeout == 0 ) {
					cmdUnlocked = false;
				}
			}
			// Reset second counter
			secTimerCount = 0;
		}
		#ifdef DEBUG_PINS
		digitalWrite(DBG_TIMER, !digitalRead(DBG_TIMER));
		#endif
	}
}

/* ===================================================================
 * Function:    handleMPCInt
 * Return:
 * Arguments:
 * Description: MPC IRQ handling outside IRQ routine, because we can
 * 				not handle I2C interfacing within IRQ subroutine.
 *              Read out MPC Register into global variables
 * ===================================================================*/
void handleMPCInt()
{
	// Check if an MPC irq occured
	if ( isrTrigger ) {
		byte i;

		#ifdef DEBUG_PINS
		digitalWrite(DBG_INT, !digitalRead(DBG_INT));
		#endif
		isrTrigger = false;

		if ( expanderReadWord(MPC_SM8STATUS, INFTF) )
		{
			#ifdef DEBUG_PINS
			digitalWrite(DBG_MPC, HIGH);
			#endif
			irqSM8Status = expanderReadWord(MPC_SM8STATUS, GPIO);
			for (i = 0; i < IOBITS_CNT; i++) {
				if ( bitRead(curSM8Status,i) != bitRead(irqSM8Status,i) ) {
					debSM8Status[i] = SM8_DEBOUNCE_TIME / TIMER_MS;
				}
			}
			#ifdef DEBUG_PINS
			digitalWrite(DBG_MPC, LOW);
			#endif
		}

		if ( expanderReadWord(MPC_PUSHBUTTON, INFTF) )
		{
			static IOBITS tmpPushButton = IOBITS_ZERO;
			#ifdef DEBUG_PINS
			digitalWrite(DBG_MPC, HIGH);
			#endif
			irqPushButton = expanderReadWord(MPC_PUSHBUTTON, GPIO);
			#ifdef DEBUG_PUSHBUTTON
			SerialTimePrintfln(F("handleMPCInt    - IRQ - irqPushButton: 0x%04x"), irqPushButton);
			#endif
			for (i = 0; i < IOBITS_CNT; i++) {
				if ( bitRead(tmpPushButton,i) != bitRead(irqPushButton,i) ) {
					#ifdef DEBUG_PUSHBUTTON
					SerialTimePrintfln(F("handleMPCInt    - IRQ - debounce key %d"), i);
					#endif
					debPushButton[i] = WPB_DEBOUNCE_TIME / TIMER_MS;
					bitWrite(tmpPushButton, i, bitRead(irqPushButton,i));
				}
			}
			#ifdef DEBUG_PINS
			digitalWrite(DBG_MPC, LOW);
			#endif
		}

		#ifdef DEBUG_PINS
		digitalWrite(DBG_INT, !digitalRead(DBG_INT));
		#endif
	}
	watchdogReset();
}



#if defined(DEBUG_MOTOR) || defined(DEBUG_MOTOR_DETAILS)
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

		SerialTimePrintfln(F("debugPrintMotorStatus(%i) - ----------------------------------------"), from);
		SerialTimePrintfln(F("debugPrintMotorStatus(%i) -    M1   M2   M3   M4   M5   M6   M7   M8"), from);
		SerialTimePrintf  (F("debugPrintMotorStatus(%i) - "), from);
		for (i = 0; i < MAX_MOTORS; i++) {
			if (MotorCtrl[i] == 0) {
				SerialPrintf(F("  off"));
			}
			else {
				SerialPrintf(F("%5d"), MotorCtrl[i]);
			}
			prevMotorCtrl[i] = MotorCtrl[i];
		}
		printCRLF();
	}
}
#endif

/* ===================================================================
 * Function:    clrSM8Status
 * Return:
 * Arguments:
 * Description: FS20-SM8 channel reset
 *              Switches all channel in ON-state to OFF
 * ===================================================================*/
void clrSM8Status(void)
{
	// Read current value from input MPC
	curSM8Status  = expanderReadWord(MPC_SM8STATUS, GPIO);
	curPushButton = expanderReadWord(MPC_PUSHBUTTON, GPIO);

	for(byte channel=0; channel<IOBITS_CNT; channel++) {
		if ( bitRead(curSM8Status, channel) ) {
			bitClear(valSM8Button, channel);
		}
	}
	expanderWriteWord(MPC_SM8BUTTON,   GPIO, valSM8Button);
	delay(FS20_SM8_IN_RESPONSE);

	for(byte channel=0; channel<IOBITS_CNT; channel++) {
		if ( bitRead(curSM8Status, channel) ) {
			bitSet(valSM8Button, channel);
		}
	}
	expanderWriteWord(MPC_SM8BUTTON,   GPIO, valSM8Button);

	watchdogReset();
}


/* ===================================================================
 * Function:    ctrlSM8Status
 * Return:
 * Arguments:
 * Description: FS20-SM8 channel output handling
 * ===================================================================*/
void ctrlSM8Status(void)
{
	static IOBITS tmpSM8Status   = IOBITS_ZERO;

	/* FS20 SM8 Output */
	static IOBITS SM8Status;
	static IOBITS prevSM8Status = IOBITS_ZERO;

	if ( (tmpSM8Status != curSM8Status) ) {
		#ifdef DEBUG_SM8STATUS
		SerialTimePrintfln(F("ctrlSM8Status   - ----------------------------------------"));
		SerialTimePrintfln(F("ctrlSM8Status   - SM8 Input Status changed"));
		#endif
		/* Read FS20-SM8 Outputs
			 - Lower 8-bits are "open" commands
			 - Upper 8-bits are "close" commands
			 - Outputs are edge trigger
				 - Slope from 0 to 1 means: Motor on
				 - Slope from 1 to 0 means: Motor off
				 - Slope for both, motor "open" and "close" within same
				   time are invalid

				So Sc  Motor
				-- --  ----------
				0  0   Off
				1  0   Opening
				0  1   Closing
				1  1   ignored (illegal state)
		*/
		if ( prevSM8Status != curSM8Status ) {
			byte i;

			IOBITS SM8StatusSlope;
			IOBITS SM8StatusChange;

			SM8Status = curSM8Status;
			SM8StatusSlope   = ~prevSM8Status & SM8Status;
			SM8StatusChange  =  prevSM8Status ^ SM8Status;

			#ifdef DEBUG_SM8STATUS
			SerialTimePrintfln(F("ctrlSM8Status   - prevSM8Status:   0x%04x"), prevSM8Status);
			SerialTimePrintfln(F("ctrlSM8Status   - SM8Status:       0x%04x"), SM8Status);
			SerialTimePrintfln(F("ctrlSM8Status   - SM8StatusSlope:  0x%04x"), SM8StatusSlope);
			SerialTimePrintfln(F("ctrlSM8Status   - SM8StatusChange: 0x%04x"), SM8StatusChange);
			SerialTimePrintfln(F("ctrlSM8Status   - SM8StatusIgnore: 0x%04x"), SM8StatusIgnore);
			#endif
			// Possibly ignore changes of single bits
			SM8StatusChange &= ~SM8StatusIgnore;
			SM8StatusIgnore = 0;

			#ifdef DEBUG_SM8STATUS
			SerialTimePrintfln(F("ctrlSM8Status   - SM8StatusChange: 0x%04x"), SM8StatusChange);
			#endif

			for (i = 0; i < IOBITS_CNT; i++) {
				if ( bitRead(SM8StatusChange, i) ) {
					sendStatus(false,FS20IN, F("%02d %S"), i+1, bitRead(curSM8Status,i)?fstrON:fstrOFF);
				}
				watchdogReset();
			}
			for (i = 0; i < MAX_MOTORS; i++) {
				// Motor opening
				if ( bitRead(SM8StatusChange, i) != 0 && bitRead(SM8StatusChange, i + MAX_MOTORS) == 0 ) {
					#ifdef DEBUG_SM8STATUS
					SerialTimePrintfln(F("ctrlSM8Status   - Motor %i should be opened"), i);
					#endif
					if ( bitRead(SM8StatusSlope, i) != 0 ) {
						#ifdef DEBUG_SM8STATUS
						SerialTimePrintfln(F("ctrlSM8Status   - Motor %i set to OPEN"), i);
						#endif
						// Slope from 0 to 1 means: Motor on
						setMotorDirection(i, MOTOR_OPEN);
						// FS20-SM8 key for "Close" active?
						if ( bitRead(SM8Status, i + MAX_MOTORS) != 0 ) {
							#ifdef DEBUG_SM8STATUS
							SerialTimePrintfln(F("ctrlSM8Status   - Reset FS20 'close' key %i"), i + MAX_MOTORS);
							#endif
							// Clear FS20-SM8 key for "Close"
							bitClear(valSM8Button, i + MAX_MOTORS);
							bitSet(SM8StatusIgnore, i + MAX_MOTORS);
						}
					}
					else {
						// Slope from 1 to 0 means: Motor off
						#ifdef DEBUG_SM8STATUS
						SerialTimePrintfln(F("ctrlSM8Status   - Open 0>1 Slope - Motor %i off"), i);
						#endif
						setMotorDirection(i, MOTOR_OFF);
					}
				}
				// Motor closing
				else if ( bitRead(SM8StatusChange, i) == 0 && bitRead(SM8StatusChange, i + MAX_MOTORS) != 0 ) {
					#ifdef DEBUG_SM8STATUS
					SerialTimePrintfln(F("ctrlSM8Status   - Motor %i should be closed"), i);
					#endif
					if ( bitRead(SM8StatusSlope, i + MAX_MOTORS) != 0 ) {
						#ifdef DEBUG_SM8STATUS
						SerialTimePrintfln(F("ctrlSM8Status   - Motor %i set to CLOSE"), i);
						#endif
						// Slope from 0 to 1 means: Motor on
						setMotorDirection(i, MOTOR_CLOSE);
						// FS20-SM8 key for "Open" active?
						if ( bitRead(SM8Status, i) != 0 ) {
							#ifdef DEBUG_SM8STATUS
							SerialTimePrintfln(F("ctrlSM8Status   - Reset FS20 'open' key %i"), i);
							#endif
							// Clear FS20-SM8 key for "Close"
							bitClear(valSM8Button, i);
							bitSet(SM8StatusIgnore, i);
						}
					}
					else {
						// Slope from 1 to 0 means: Motor off
						#ifdef DEBUG_SM8STATUS
						SerialTimePrintfln(F("ctrlSM8Status   - Close 0>1 Slope - Motor %i off"), i);
						#endif
						setMotorDirection(i, MOTOR_OFF);;
					}
				}
				// Ignored (illegal state)
				else if ( bitRead(SM8StatusChange, i) != 0 && bitRead(SM8StatusChange, i + MAX_MOTORS) != 0 ) {
					#ifdef DEBUG_SM8STATUS
					SerialTimePrintfln(F("ctrlSM8Status   - Invalid 0>1 Slope, reset key %i and %i"), i, i+MAX_MOTORS);
					#endif
					// Clear both FS20-SM8 key for "Open" and "Close"
					bitClear(valSM8Button, i);
					bitClear(valSM8Button, i + MAX_MOTORS);
					bitSet(SM8StatusIgnore, i);
					bitSet(SM8StatusIgnore, i + MAX_MOTORS);
				}
				watchdogReset();
			}

			prevSM8Status = curSM8Status;
		}

		#ifdef DEBUG_MOTOR
		debugPrintMotorStatus(false);
		#endif

		tmpSM8Status  = curSM8Status;
	}

	watchdogReset();
}

/* ===================================================================
 * Function:    ctrlPushButton
 * Return:
 * Arguments:
 * Description: Pushbutton handling
 * ===================================================================*/
void ctrlPushButton(void)
{
	static IOBITS tmpPushButton  = ~IOBITS_ZERO;
	       IOBITS PushButton;		// Push button output
	static IOBITS prevPushButton = ~IOBITS_ZERO;

	if ( (tmpPushButton != curPushButton) ) {
		/* Read pushbuttons
			 - Lower 8-bits are "Open" buttons
			 - Upper 8-bits are "Close" buttons
			 - Slope from 0 to 1: Toggle motor
			 - Slope from 1 to 0: Do nothing
			 - Single levels are unused, only slopes
			 - Level for "Open" and "Close" button are together 1:
				- Motor off
				- Lock button pair
			 - Level for "Open" and "Close" button are together 0:
				- Unlock button pair
		*/
		#ifdef DEBUG_PUSHBUTTON
		SerialTimePrintfln(F("ctrlPushButton  - tmpPushButton: 0x%04x"), tmpPushButton);
		SerialTimePrintfln(F("ctrlPushButton  - curPushButton: 0x%04x"), curPushButton);
		#endif
		if ( prevPushButton != curPushButton ) {
			byte i;

			IOBITS PushButtonSlope;
			IOBITS PushButtonChange;

			PushButton = curPushButton;
			PushButtonSlope = ~prevPushButton & PushButton;
			PushButtonChange = prevPushButton ^ PushButton;

			for (i = 0; i < IOBITS_CNT; i++) {
				if ( bitRead(PushButtonChange, i) ) {
					sendStatus(false,PUSHBUTTON, F("%02d %S"), i+1, bitRead(curPushButton,i)?fstrON:fstrOFF);
					if ( bitRead(curPushButton,i) ) {
						PushButtonTimer[i % MAX_MOTORS] = millis();
					}
					else {
						TIMER dt = 0;
						TIMER maxTime = 0;
						if ( millis() > PushButtonTimer[i % MAX_MOTORS] ) {
							dt  = millis() - PushButtonTimer[i % MAX_MOTORS];
							// Aufrunden
							maxTime =  dt + (TIMER_MS / 2);
							maxTime /= TIMER_MS;
							maxTime *= TIMER_MS;
						}
						#ifdef DEBUG_PUSHBUTTON
						SerialTimePrintfln(F("ctrlPushButton  - motor %d: delta_t=%ld ms"), i % MAX_MOTORS, dt);
						#endif
						if ( maxTime > 10000 ) {
							#ifdef DEBUG_PUSHBUTTON
							SerialTimePrintfln(F("ctrlPushButton  - Setting new timeout for motor %d: %ld ms"), i % MAX_MOTORS, maxTime);
							#endif
							sendStatus(false,MOTOR, F("%02d TIMEOUT %ld"), (i % MAX_MOTORS)+1, maxTime);
							eeprom.MaxRuntime[i % MAX_MOTORS] = maxTime;
							eepromWriteVars();
						}
					}
				}
				watchdogReset();
			}
			for (i = 0; i < MAX_MOTORS; i++) {
				// Slope from 0 to 1: Toggle motor
				if ( bitRead(PushButtonChange, i) != 0 && bitRead(PushButtonSlope, i) != 0 ) {
					setMotorDirection(i, MOTOR_OPEN);
				}
				else if ( bitRead(PushButtonChange, i + MAX_MOTORS) != 0 && bitRead(PushButtonSlope, i + MAX_MOTORS) != 0 ) {
					setMotorDirection(i, MOTOR_CLOSE);
				}

				// Level for "Open" and "Close" button are together 1:
				if ( bitRead(PushButton, i) != 0 && bitRead(PushButton, i + MAX_MOTORS) != 0 ) {
					setMotorDirection(i, MOTOR_OFF);
				}
				watchdogReset();
			}

			#ifdef DEBUG_PUSHBUTTON
			SerialTimePrintfln(F("ctrlPushButton  - ----------------------------------------"));
			SerialTimePrintfln(F("ctrlPushButton  - prevPushButton:   0x%04x"), prevPushButton);
			SerialTimePrintfln(F("ctrlPushButton  - PushButton:       0x%04x"), PushButton);
			SerialTimePrintfln(F("ctrlPushButton  - PushButtonSlope:  0x%04x"), PushButtonSlope);
			SerialTimePrintfln(F("ctrlPushButton  - PushButtonChange: 0x%04x"), PushButtonChange);
			#endif

			prevPushButton = curPushButton;
		}

		#ifdef DEBUG_MOTOR
		debugPrintMotorStatus(true);
		#endif

		tmpPushButton = curPushButton;
	}

	watchdogReset();
}

/* ===================================================================
 * Function:    ctrlSM8Button
 * Return:
 * Arguments:
 * Description: FS20-SM8 key button handling
 * ===================================================================*/
void ctrlSM8Button(void)
{
	static IOBITS tmpSM8Button = ~IOBITS_ZERO;
	byte i;

	if ( tmpSM8Button != valSM8Button ) {
		#ifdef DEBUG_SM8OUTPUT
		SerialTimePrintfln(F("ctrlSM8Button   - ----------------------------------------"));
		SerialTimePrintfln(F("ctrlSM8Button   - SM8 key output changed"));
		#endif
		expanderWriteWord(MPC_SM8BUTTON,   GPIO, valSM8Button);

		for (i = 0; i < IOBITS_CNT; i++) {
			if ( (bitRead(tmpSM8Button, i) != bitRead(valSM8Button, i)) ) {
				sendStatus(false,FS20OUT, F("%2d %S"), i+1, bitRead(valSM8Button,i)?fstrOFF:fstrON);
			}

			// If key output just activated, set FS20-SM8 key timeout
			if ( (bitRead(tmpSM8Button, i) != bitRead(valSM8Button, i)) && (bitRead(valSM8Button, i) == 0) ) {
				#ifdef DEBUG_SM8OUTPUT
				SerialTimePrintfln(F("ctrlSM8Button   - SM8 key %d output set timeout to %d ms"), i+1, FS20_SM8_IN_RESPONSE/TIMER_MS);
				#endif
				SM8Timeout[i] = FS20_SM8_IN_RESPONSE / TIMER_MS;
			}
			watchdogReset();
		}

		tmpSM8Button = valSM8Button;
	}

	watchdogReset();
}

/* ===================================================================
 * Function:    ctrlMotorRelais
 * Return:
 * Arguments:
 * Description: Motor relais output bit handling
 * ===================================================================*/
void ctrlMotorRelais(void)
{
	static IOBITS tmpMotorRelais = IOBITS_ZERO;
	static IOBITS tmpOutMotorRelais = IOBITS_ZERO;
	static byte   preMotorCount = 0;
	static IOBITS preMotorRelais[4];
	static TIMER  preMotorTimer = 0;
		   IOBITS outMotorRelais = IOBITS_ZERO;
	static IOBITS prevRegMotorRelais = IOBITS_ZERO;

	byte i;

	if ( (tmpMotorRelais != valMotorRelais) && (preMotorCount == 0) ) {
		byte k;
		byte curTarget;
		byte targetSteps;
		byte targetStep[4];

		#ifdef DEBUG_MOTOR
		SerialTimePrintfln(F("ctrlMotorRelais - ----------------------------------------"));
		SerialTimePrintfln(F("ctrlMotorRelais - Motor output change test"));
		SerialTimePrintfln(F("ctrlMotorRelais -   tmpMotorRelais: 0x%04X"), tmpMotorRelais);
		SerialTimePrintfln(F("ctrlMotorRelais -   valMotorRelais: 0x%04X"), valMotorRelais);
		#endif

		preMotorTimer = millis() + RELAIS_OPERATE_TIME;
		preMotorCount = 0;
		preMotorRelais[0] = valMotorRelais;
		preMotorRelais[1] = valMotorRelais;
		preMotorRelais[2] = valMotorRelais;
		preMotorRelais[3] = valMotorRelais;

		/* Table (M=Motorbit, D=Directionbit, S=Steps)
		 * Current Target Steps
		 * MD      MD     MD          S MC MD
		 * 00      00     -        -  0 00 00
		 * 00      01     01       -  1 00 01
		 * 00      10     10       -  1 01 00
		 * 00      11     01 11    x  2 01 01
		 * 01      00     00       -  1 00 10
		 * 01      01     -        -  0 00 11
		 * 01      10     00 10    x  2 01 10
		 * 01      11     11       -  1 01 11
		 * 10      00     00       -  1 10 00
		 * 10      01     00 01    x  2 10 01
		 * 10      10     -        -  0 11 00
		 * 10      11     00 01 11 x  3 11 01
		 * 11      00     01 00    x  2 10 10
		 * 11      01     01       -  1 10 11
		 * 11      10     01 00 10 x  3 11 00
		 * 11      11     -        -  0 11 11	 */
		// Check motor by motor
		for(i=0; i<MAX_MOTORS; i++) {
			curTarget = 0;
			bitWrite(curTarget, 3, bitRead(tmpMotorRelais, i));
			bitWrite(curTarget, 2, bitRead(tmpMotorRelais, i+MAX_MOTORS));
			bitWrite(curTarget, 1, bitRead(valMotorRelais, i));
			bitWrite(curTarget, 0, bitRead(valMotorRelais, i+MAX_MOTORS));
			// everytime output last step (relais final state)
			targetSteps = 0;

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
			#ifdef DEBUG_MOTOR_DETAILS
			SerialTimePrintfln(F("ctrlMotorRelais -   Motor %d, curTarget 0x%02X, Steps %d"), i, curTarget, targetSteps);
			#endif

			// Copy all steps to final mask
			for(k=targetSteps; k>0; k--) {
				#ifdef DEBUG_MOTOR_DETAILS
				SerialTimePrintfln(F("ctrlMotorRelais -     targetStep[%d]=0x%02X"), k, targetStep[k]);
				#endif
				// mask bits of the current motor
				bitClear(preMotorRelais[k], i);
				bitClear(preMotorRelais[k], i+MAX_MOTORS);
				// New motor bit for this step and motor
				bitWrite(preMotorRelais[k], i, targetStep[k]>>1);
				// New direction bit for this step and motor
				bitWrite(preMotorRelais[k], i+MAX_MOTORS, targetStep[k] & 1);
			}

			if ( targetSteps > preMotorCount ) {
				preMotorCount = targetSteps;
			}
		}
		#ifdef DEBUG_MOTOR_DETAILS
		SerialTimePrintfln(F("ctrlMotorRelais -   preMotorCount=%d"), preMotorCount);
		for(i=0; i<preMotorCount; i++) {
			SerialTimePrintfln(F("ctrlMotorRelais -     preMotorRelais[%d] =0x%04X"), i, preMotorRelais[i]);
		}
		#endif

		// Next bit output without delay
		preMotorTimer = millis();

		tmpMotorRelais = valMotorRelais;
	}

	if ( (long)( millis() - preMotorTimer) >= 0 ) {
		// Get current motor bits
		bool doSM8andTimeout = true;

		outMotorRelais = preMotorRelais[preMotorCount];

		if ( preMotorCount>0 ) {
			#ifdef DEBUG_MOTOR_DETAILS
			SerialTimePrintfln(F("ctrlMotorRelais - D   a)preMotorCount=%d"), preMotorCount);
			#endif
			preMotorCount--;
			#ifdef DEBUG_MOTOR_DETAILS
			SerialTimePrintfln(F("ctrlMotorRelais - D   b)preMotorCount=%d"), preMotorCount);
			#endif
			preMotorTimer += RELAIS_OPERATE_TIME;
			doSM8andTimeout = false;
		}

		if ( tmpOutMotorRelais != outMotorRelais ) {
			#ifdef DEBUG_MOTOR
			SerialTimePrintfln(F("ctrlMotorRelais -     Output outMotorRelais 0x%04X"), outMotorRelais);
			#endif
			regMotorRelais = outMotorRelais;
			expanderWriteWord(MPC_MOTORRELAIS, GPIO, regMotorRelais);

			if ( doSM8andTimeout ) {
				for (i = 0; i < MAX_MOTORS; i++) {
					// Set motor timeout
						// - if motor just activated
					if (    ( !bitRead(tmpOutMotorRelais, i) && bitRead(outMotorRelais, i) )
						// - or still running but direction changed
						 || ( bitRead(outMotorRelais, i) &&
							  bitRead(tmpOutMotorRelais, i+MAX_MOTORS)!=bitRead(outMotorRelais, i+MAX_MOTORS) ) ) {
						#ifdef DEBUG_MOTOR
						SerialTimePrintfln(F("ctrlMotorRelais -     Set motor %d timeout to %d.%-d s"), i+1, eeprom.MaxRuntime[i] / 1000, eeprom.MaxRuntime[i] % 1000);
						#endif
						MotorTimeout[i] = (eeprom.MaxRuntime[i] / TIMER_MS) + (eeprom.OvertravelTime[i] / TIMER_MS);
					}
					// FS20-SM8 Output for "Open" active, but motor is off or closing
					if ( bitRead(curSM8Status, i)
						 && (getMotorDirection(i)==MOTOR_OFF || getMotorDirection(i)==MOTOR_CLOSE) ) {
						// Reset FS20-SM8 key for "Open"
						#if defined(DEBUG_MOTOR) || defined(DEBUG_SM8OUTPUT)
						SerialTimePrintfln(F("ctrlMotorRelais - SM8 Taste %d (für \"Öffnen\") zurücksetzen"), i);
						#endif
						bitSet(SM8StatusIgnore, i);
						bitClear(valSM8Button, i);
					}

					// FS20-SM8 Output for "Close" active, but motor is off or opening
					if (    bitRead(curSM8Status, i+MAX_MOTORS)
						 && (getMotorDirection(i)==MOTOR_OFF || getMotorDirection(i)==MOTOR_OPEN) ) {
						// Reset FS20-SM8 key for "Close"
						#if defined(DEBUG_MOTOR) || defined(DEBUG_SM8OUTPUT)
						SerialTimePrintfln(F("ctrlMotorRelais - SM8 Taste %d (für \"Schliessen\") zurücksetzen"), i+MAX_MOTORS);
						#endif
						bitSet(SM8StatusIgnore, i+MAX_MOTORS);
						bitClear(valSM8Button, i+MAX_MOTORS);
					}
					watchdogReset();
				}
			}
			tmpOutMotorRelais = outMotorRelais;

			// Possibly remember motor position and write it to EEPROM
			if ( prevRegMotorRelais != regMotorRelais ) {
				bool changed = false;
				#ifdef DEBUG_MOTOR
				SerialTimePrintfln(F("ctrlMotorRelais -     regMotorRelais changed, check positions"));
				#endif
				for (i = 0; i < MAX_MOTORS; i++) {
					if ( !bitRead(regMotorRelais, i) && (eeprom.MotorPosition[i] != MotorPosition[i]) ) {
						// Only when motor is off, write position to EEPROM
						#ifdef DEBUG_MOTOR
						SerialTimePrintfln(F("ctrlMotorRelais -     Store motor %d position %d"), i, MotorPosition[i]);
						#endif
						eeprom.MotorPosition[i] = MotorPosition[i];
						changed = true;
					}
				}
				prevRegMotorRelais = regMotorRelais;
				if ( changed ) {
					eepromWriteVars();
				}
			}

		}
	}
	#ifdef DEBUG_MOTOR_DETAILS
	else {
		SerialTimePrintfln(F("ctrlMotorRelais - Z   Timer waiting"));
	}
	#endif

	watchdogReset();
}

/* ===================================================================
 * Function:    ctrlRainSensor
 * Return:
 * Arguments:
 * Description: Rain sensor handling
 * ===================================================================*/
#ifdef DEBUG_RAIN
const char dbgCtrlRainSensor[] PROGMEM = "ctrlRainSensor  - ";
#endif
void ctrlRainSensor(void)
{
	static bool firstRun = true;
	bool RainInput;
	bool RainEnable;
	bool sensRainInput;
	bool sensRainEnable;
	static bool prevRainInput;
	static bool prevRainEnable;
	static bool prevRainEnableInput;

	// Debounce inputs
	debEnable.update();
	debInput.update();

	// Read debounced inputs
	sensRainEnable = (debEnable.read() == RAIN_ENABLE_ACTIVE);
	sensRainInput  = (debInput.read()  == RAIN_INPUT_ACTIVE);

	if ( firstRun ) {
		prevRainEnableInput = sensRainEnable;
	}

	/* If enable input has changed, activate AUTO mode
	 * regardless what user has set with RAIN DISABLE/ENABLE command */
	if ( prevRainEnableInput != sensRainEnable ) {
		#ifdef DEBUG_RAIN
		SerialTimePrintfln(F("%SsensRainEnable      = %d"), dbgCtrlRainSensor, sensRainEnable);
		SerialTimePrintfln(F("%SprevRainEnableInput = %d"), dbgCtrlRainSensor, prevRainEnableInput);
		#endif
		bitSet(eeprom.Rain, RAIN_BIT_AUTO);
		eepromWriteVars();
		prevRainEnableInput = sensRainEnable;
	}

	// Read rain "input" related to mode
	if ( bitRead(eeprom.Rain, RAIN_BIT_AUTO) ) {
		// AUTO mode active: Read rain enable signal from input
		RainEnable = sensRainEnable;
	}
	else {
		// MANUAL mode active: Read rain enable status from setting
		RainEnable = bitRead(eeprom.Rain, RAIN_BIT_ENABLE);
	}

	// Rain is active if settings set to rain or rain sensor input is active
	RainInput  = (sensRainInput || softRainInput);

	if ( firstRun ) {
		prevRainInput  = RainInput;
		prevRainEnable = RainEnable;
		firstRun = false;
	}

	if ( prevRainInput != RainInput || prevRainEnable != RainEnable ) {
		#ifdef DEBUG_RAIN
		SerialTimePrintfln(F("%S----------------------------------------"), dbgCtrlRainSensor);
		SerialTimePrintfln(F("%SdigitalRead(%d): %d"), dbgCtrlRainSensor, RAIN_INPUT, digitalRead(RAIN_INPUT));
		SerialTimePrintfln(F("%SdigitalRead(%d): %d"), dbgCtrlRainSensor, RAIN_ENABLE, digitalRead(RAIN_ENABLE));
		SerialTimePrintfln(F("%SRainMode:   %s"), dbgCtrlRainSensor, bitRead(eeprom.Rain, RAIN_BIT_AUTO) ? "AUTO" : "MANUAL" );
		SerialTimePrintfln(F("%SRainEnable: %d"), dbgCtrlRainSensor, RainEnable);
		SerialTimePrintfln(F("%SRainInput:  %d"), dbgCtrlRainSensor, RainInput);
		#endif
		if ( RainEnable ) {
			if ( RainInput ) {
				byte i;

				#ifdef DEBUG_RAIN
				SerialTimePrintfln(F("%SRain enabled, wet"), dbgCtrlRainSensor);
				#endif

				for (i = 0; i < MAX_MOTORS; i++) {
					if ( getMotorType(i)==WINDOW ) {
						if ( bitRead(eeprom.Rain, RAIN_BIT_RESUME)
							&& getMotorDirection(i)==MOTOR_OFF
							&& MotorPosition[i]!=0 ) {
							resumeMotorPosition[i] = (MOTOR_TIMER)(((unsigned long)MotorPosition[i]*100L) / (unsigned long)(eeprom.MaxRuntime[i] / TIMER_MS));
							#ifdef DEBUG_RAIN
							SerialTimePrintfln(F("%SRemember window %d position %d%% (%d)"), dbgCtrlRainSensor, i, resumeMotorPosition[i], MotorPosition[i]);
							#endif
						}
						if ( getMotorDirection(i)!=MOTOR_CLOSE && getMotorDirection(i)!=MOTOR_CLOSE_DELAYED) {
							#ifdef DEBUG_RAIN
							SerialTimePrintfln(F("%SClose window %d"), dbgCtrlRainSensor, i);
							#endif
							setMotorDirection(i, MOTOR_CLOSE);
						}
					}
				}
				cmdRainSensorPrintStatus();
				isRaining = true;
				currentLEDPattern = eeprom.LEDPatternRain;
			}
			else {
				#ifdef DEBUG_RAIN
				SerialTimePrintfln(F("%SRain enabled, dry"), dbgCtrlRainSensor);
				#endif
				if ( bitRead(eeprom.Rain, RAIN_BIT_RESUME) ) {
					resumeDelay = (WORD)((unsigned long)eeprom.RainResumeTime * 1000L / TIMER_MS);
				}
				cmdRainSensorPrintStatus();
				isRaining = false;
				currentLEDPattern = eeprom.LEDPatternNormal;
			}
		}
		else {
			if ( RainInput ) {
				#ifdef DEBUG_RAIN
				SerialTimePrintfln(F("%SRain disabled, wet"), dbgCtrlRainSensor);
				#endif
				cmdRainSensorPrintStatus();
			}
			else {
				#ifdef DEBUG_RAIN
				SerialTimePrintfln(F("%SRain disabled, dry"), dbgCtrlRainSensor);
				#endif
				cmdRainSensorPrintStatus();
			}
			isRaining = false;
			currentLEDPattern = eeprom.LEDPatternNormal;
		}
		prevRainEnable = RainEnable;
		prevRainInput  = RainInput;
	}

	if ( resumeDelay==0 ) {
		#ifdef DEBUG_RAIN
		SerialTimePrintfln(F("%SResume delay expired"), dbgCtrlRainSensor);
		#endif
		for (byte i = 0; i < MAX_MOTORS; i++) {
			if ( bitRead(eeprom.Rain, RAIN_BIT_RESUME)
				&& getMotorType(i)==WINDOW
				&& getMotorDirection(i)==MOTOR_OFF
				&& resumeMotorPosition[i]!=NO_RESUME_POSITION ) {
					#ifdef DEBUG_RAIN
					SerialTimePrintfln(F("%SResume motor %d to %d%%"), dbgCtrlRainSensor, i+1, resumeMotorPosition[i] );
					#endif
					setMotorPosition(i, resumeMotorPosition[i]);
					resumeMotorPosition[i] = NO_RESUME_POSITION;
			}
		}
		resumeDelay = NO_RESUME_DELAY;
	}

	watchdogReset();
}


/* ===================================================================
 * Function:    beAlive()
 * Return:
 * Arguments:
 * Description: Alive timer (watchdog trigger)
 * ===================================================================*/
void beAlive(void)
{
	#ifdef DEBUG_ALIVE
	static char liveToogle = 0;
	static byte liveDots = 80;
	#endif

	// Live timer and watchdog handling
	if ( (long)( millis() - runTimer) >= 0 ) {
		runTimer += ALIVE_TIMER;
		watchdogReset();
		#ifdef DEBUG_ALIVE
		if ((liveToogle--) < 1) {
			SerialPrintf(F("."));
			liveToogle = 3;
			liveDots++;
			if ( liveDots > 76 ) {
				printCRLF();
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
 * Description: LED blinking (to show that controller works and run
 *              in main loop)
 * ===================================================================*/
void blinkLED(void)
{
	if ( (long)( millis() - ledTimer) >= 0 ) {
		ledTimer += (TIMER)eeprom.LEDBitLenght;
		digitalWrite(STATUS_LED, bitRead(currentLEDPattern, 1) );
		currentLEDPattern >>= 1;
		currentLEDBitCount++;
		if ( currentLEDBitCount > eeprom.LEDBitCount ) {
			currentLEDBitCount = 0;
			if ( isRaining ) {
				currentLEDPattern = eeprom.LEDPatternRain;
			}
			else {
				currentLEDPattern = eeprom.LEDPatternNormal;
			}
		}
	}

	watchdogReset();
}

/* ===================================================================
 * Function:    operationHours()
 * Return:
 * Arguments:	millisSet true, if system timer was changed manually
 * Description: Remember operation hours in EEPROM
 * ===================================================================*/
void operationHours(bool millisSet)
{
	static unsigned long opHour;

	opHour = sec(NULL);
	if ( ((opHour % 3600L) == 0) && (savedOperationTime!=opHour) ) {
		if ( !millisSet ) {
			eeprom.OperatingHours++;
			eepromWriteVars();
			sendStatus(false,SYSTEM, F("OPERATION %d h"), eeprom.OperatingHours);
		}
		savedOperationTime = opHour;
	}

	watchdogReset();
}

/* ===================================================================
 * Function:    loginStatus()
 * Return:
 * Arguments:
 * Description: call sendStatus with new login value when it was change
 *              during IRQ
 * ===================================================================*/
void loginStatus(void)
{
	if ( prevUnlocked != cmdUnlocked ) {
		cmdLoginStatus(false);
		prevUnlocked = cmdUnlocked;
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
	// MPC IRQ handling outside IRQ routine
	handleMPCInt();

	// FS20-SM8 channel output handling
	ctrlSM8Status();

	/* Pushbuttons are higher priorisized than FS20-SM8 channel outputs
	 * so we handle pushbuttons after FS20-SM8 channel outputs to
	 * possibly overrule FS20-SM8 */

	// Pushbutton handling
	ctrlPushButton();

	// Motor relais output bit handling
	ctrlMotorRelais();

	// FS20-SM8 key button handling
	ctrlSM8Button();

	// Rain sensor handling
	ctrlRainSensor();

	// Output motor OFF status if it has changed during IRQ
	sendMotorOffStatus();

	// Output login status if it has changed during IRQ
	loginStatus();

	// Alive timer (watchdog trigger)
	beAlive();

	// LED blinking
	blinkLED();

	// Handle serial interface
	processSerialCommand();

	// Remember operation hours in EEPROM
	operationHours(false);
}
