/* ==========================================================================
 * Hardware definition (I/O Ports...)
 * Zeiten in ms müssen Vielfache von 10 sein
 * ========================================================================== */
// MCP23017 Adressen
#define MPC1    			0x0    // MCP23017 #1 I2C Adresse
#define MPC2    			0x1    // MCP23017 #2 I2C Adresse
#define MPC3				0x2    // MCP23017 #3 I2C Adresse
#define MPC4    			0x3    // MCP23017 #4 I2C Adresse

#define MPC_MOTORRELAIS		MPC1
#define MPC_SM8BUTTON		MPC2
#define MPC_SM8STATUS		MPC3
#define MPC_WALLBUTTON		MPC4

#define ISR_INPUT 			2			// ISR Input from MPC = D2
#define ONBOARD_LED 		LED_BUILTIN	// LED = D13

#define RAIN_INPUT 			4			// Input Signal für Regensensor
#define RAIN_INPUT_AKTIV	0

#define RAIN_ENABLE			5			// Input Signal für Regensensor aktiv
#define RAIN_ENABLE_AKTIV	0


// Serial
#define SERIAL_BAUDRATE		115200

// Anzahl der Motoren
#define MAX_MOTORS				8

// Timer2 Intervall in ms
#define TIMER_MS					10

/* ==========================================================================
 * Standardwerte
 * ========================================================================== */
// Relais Ansprechzeit in ms (Datenblatt 5 ms)
#define OPERATE_TIME				20

// Relais Rückfallzeit in ms (Datenblatt 4 ms))
#define RELEASE_TIME				20

// Motor Umschaltdelay in ms
#define MOTOR_SWITCHOVER			250

// Motor maximale Laufzeit in ms
// Fenster auf:  47s
// Fenster zu:   47s
// Jalousie auf: 15s
// Jalousie zu:  18s
#define MOTOR_WINDOW_MAXRUNTIME		50000
#define MOTOR_JALOUSIE_MAXRUNTIME	20000

// SM8 IN Schaltzeit in ms
#define FS20_SM8_IN_RESPONSE		150

// Tasten Entprellzeit in ms
#define DEBOUNCE_TIME				20

#define MTYPE_BITMASK				0b00001111	// Bitmask für Fenster Motoren (DFF=1, Jalousien=0)

// LED Blink Interval in ms
#define LED_BLINK_INTERVAL			1900
// LED Blink Length in ms
#define LED_BLINK_LEN				100


/* ==========================================================================
 * Type & Constant Definition
 * ========================================================================== */
typedef unsigned int  WORD;
typedef unsigned long DWORD;
typedef DWORD TIMER;


// EEPROM Data Adressen
#define EEPROM_ADDR_CRC32				0
#define EEPROM_ADDR_DATAVERSION			(4+EEPROM_ADDR_CRC32)
#define EEPROM_ADDR_LED_BLINK_INTERVAL	(4+EEPROM_ADDR_DATAVERSION)
#define EEPROM_ADDR_LED_BLINK_LEN		(4+EEPROM_ADDR_LED_BLINK_INTERVAL)
#define EEPROM_ADDR_MTYPE_BITMASK		(4+EEPROM_ADDR_LED_BLINK_LEN)
#define EEPROM_ADDR_MOTOR_MAXRUNTIME	(4+EEPROM_ADDR_MTYPE_BITMASK)
#define EEPROM_ADDR_FREE				((4*MAX_MOTORS)+EEPROM_ADDR_MOTOR_MAXRUNTIME)

#if MAX_MOTORS<=4
	#define IOBITS_ZERO	0x00
	typedef byte MOTORBITS;
	typedef byte IOBITS;
#elif MAX_MOTORS<=8
	#define IOBITS_ZERO	0x0000
	typedef byte MOTORBITS;
	typedef unsigned int IOBITS;
#elif MAX_MOTORS<=16
	#define IOBITS_ZERO	0x00000000
	typedef unsigned int MOTORBITS;
	typedef unsigned long IOBITS;
#else
	#assert Too many motor devices (MAX_MOTORS > 16)
#endif

#define IOBITS_CNT	(MAX_MOTORS * 2)

typedef char MOTOR_CTRL;

#if   (MOTOR_WINDOW_MAXRUNTIME/TIMER_MS)<=255
	typedef byte MOTOR_TIMEOUT;
#elif (MOTOR_WINDOW_MAXRUNTIME/TIMER_MS)<=65535
	typedef WORD MOTOR_TIMEOUT;
#else
	typedef DWORD MOTOR_TIMEOUT;
#endif


#if   (FS20_SM8_IN_RESPONSE/TIMER_MS)<=255
	typedef byte SM8_TIMEOUT;
#elif (FS20_SM8_IN_RESPONSE/TIMER_MS)<=65535
	typedef WORD SM8_TIMEOUT;
#else
	typedef DWORD SM8_TIMEOUT;
#endif
