/* ===================================================================
 * Hardware definition (I/O Ports...)
 * ===================================================================*/
// MCP23017 Adressen
#define MPC1    			0x0		// MCP23017 #1 I2C Address
#define MPC2    			0x1		// MCP23017 #2 I2C Address
#define MPC3				0x2		// MCP23017 #3 I2C Address
#define MPC4    			0x3		// MCP23017 #4 I2C Address

// MPC Funktionsnamen
#define MPC_MOTORRELAIS		MPC1	// MPC for motor relais
#define MPC_SM8BUTTON		MPC2	// MPC for FS20-SM8 key outputs
#define MPC_SM8STATUS		MPC3	// MPC for FS20-SM8 output
#define MPC_PUSHBUTTON		MPC4	// MPC for push buttons

#define MPC_INT_INPUT 		3		// Arduino port for MPC IRQ
#define STATUS_LED 			LED_BUILTIN	// LED

#define RAIN_INPUT 			4		// Arduino port for rain sensor
#define RAIN_INPUT_ACTIVE	0		// Active level

#define RAIN_ENABLE			9		// Arduino Port for rain enable/disable input
#define RAIN_ENABLE_ACTIVE	1		// Active level

// analog inputs which can be used to init random generator
// define as much as possible
#define RANDOM_SEED_ANALOG_READ1	0
#define RANDOM_SEED_ANALOG_READ2	1
#define RANDOM_SEED_ANALOG_READ3	2


/* ===================================================================
 * Program default values
 * ===================================================================*/
#define DEFAULT_MOTORTYPE			0b01010101	// WIN/JAL/WIN/JAL...
#define DEFAULT_SENDSTATUS			false
#define DEFAULT_ECHO				false
#define DEFAULT_TERM				'\r'
#define DEFAULT_LOGIN_TIMEOUT		60	// sec
#define DEFAULT_PASSWORD			PSTR("31415")

/* Rain window resume position delay */
#define DEFAULT_RAINRESUMETIME		30	// sec

// 00100000 00001000 00000010 00000000
// LED 100 30 0x20080200
#define DEFAULT_LED_NORMAL			0x20080200	// bit pattern
// 11011111 11110111 11111101 11111111
// LED 100 30 0x20080200 0xdff7fdff
// 00000101 00000000 00000011 11111111
// LED 100 30 0x20080200 0x050003FF
// 00101000 00001010 00000010 10000000
// LED 100 30 0x20080200 0x280a0280
// 11010111 11110101 11111101 01111111
// LED 100 30 0x20080200 0xd7f5fd7f
#define DEFAULT_LED_RAIN			0xd7f5fd7f	// bit pattern
// Bit counts to use from pattern
#define DEFAULT_LED_BITCOUNT		30	// #
// LED Bit Length in ms
#define DEFAULT_LED_BITLENGTH		100	// ms

/* -------------------------------------------------------------------
 * Onboard LED
 * -------------------------------------------------------------------*/
// LED Pattern
#define MAX_LEDPATTERN_BITS			32

/* -------------------------------------------------------------------
 * Timer
 * -------------------------------------------------------------------*/
// Serial interface baudrate
#define SERIAL_BAUDRATE				115200

// Timer2 intervall in ms
#define TIMER_MS					10

/* -------------------------------------------------------------------
 * Watchdog
 * -------------------------------------------------------------------*/
// Possible values (ms): 15,30,60,120,250,500,1000,2000,4000,8000
#define WATCHDOG_TIME				4000
// Watchdog Timer for REBOOT command
#define WATCHDOG_REBOOT_DELAY		250

// Alive timer period in ms
#define ALIVE_TIMER					500

/* -------------------------------------------------------------------
 * Input debounce times
 * -------------------------------------------------------------------*/
// Key debounce time in ms
// Rain sensor input
#define RAIN_DEBOUNCE_TIME			20	// ms
// Pushbuttons
#define WPB_DEBOUNCE_TIME			80	// ms
// SM8 outputs
#define SM8_DEBOUNCE_TIME			20	// ms

/* -------------------------------------------------------------------
 * Motor and motor control related
 * -------------------------------------------------------------------*/
// Maximum number of motors within system
#define MAX_MOTORS					8

// Relais closing time in ms (Datasheet 5 ms)
// Relais opening time in ms (Datasheet 4 ms))
#define RELAIS_OPERATE_TIME			20	// ms

// Motor switchover delay in ms
#define MOTOR_SWITCHOVER			250	// ms

// Motor minimum runtime in ms
#define MOTOR_MINRUNTIME		 	500	// 0.5 seconds
// Motor maximum runtime in ms
#define MOTOR_MAXRUNTIME		 	600000	// 10 minutes
// Default runtimes
#define MOTOR_WINDOW_MAXRUNTIME		60000	// 60 seconds
#define MOTOR_JALOUSIE_MAXRUNTIME	20000	// 2 seconds
// Default overtravel
#define MOTOR_WINDOW_OVERTRAVELTIME		2000	// 2 seconds
#define MOTOR_JALOUSIE_OVERTRAVELTIME	1000	// 1 second

/* -------------------------------------------------------------------
 * FS20-SM8
 * -------------------------------------------------------------------*/
// SM8 IN circuit time in ms
#define FS20_SM8_IN_RESPONSE		150	// ms
// SM8 IN circuit for programmer mode in ms
#define FS20_SM8_IN_PROGRAMMODE		6000	// 6 seconds


/* ===================================================================
 * Type & Constant Definition
 * ===================================================================*/
// Length of motor names
#define MAX_NAMELEN					21

/*  0: Motor off
 * >0: Motor opening    1=immediately, >1=delay in ms before opening)
 * <0: Motor closeing  -1=immediately, <1=abs(delay) in ms before closing)
 *                        delay values are in ms/TIMER_MS */
#define MOTOR_OPEN			1
#define MOTOR_OPEN_DELAYED	(MOTOR_SWITCHOVER/TIMER_MS)
#define MOTOR_CLOSE			-1
#define MOTOR_CLOSE_DELAYED	(-MOTOR_SWITCHOVER/TIMER_MS)
#define MOTOR_OFF			0

#define NO_POSITION 			MAX_MOTOR_TIMER
#define NO_RESUME_POSITION		UINT8_MAX
#define NO_RESUME_DELAY 		MAX_MOTOR_TIMER

// own types
typedef unsigned int  	WORD;
typedef unsigned long 	DWORD;
typedef unsigned long	TIMER;
typedef uint32_t		LEDPATTERN;
#if MAX_MOTORS<=4
	#define IOBITS_ZERO		0x00
	#define IOBITS_MASK		0xFF
	#define IOBITS_LOWMASK	0x0F
	#define IOBITS_HIGHMASK	0xF0
	typedef byte MOTORBITS;
	typedef byte IOBITS;
#elif MAX_MOTORS<=8
	#define IOBITS_ZERO		0x0000
	#define IOBITS_MASK		0xFFFF
	#define IOBITS_LOWMASK	0x00FF
	#define IOBITS_HIGHMASK	0xFF00
	typedef byte MOTORBITS;
	typedef unsigned int IOBITS;
#elif MAX_MOTORS<=16
	#define IOBITS_ZERO		0x00000000
	#define IOBITS_MASK		0xFFFFFFFF
	#define IOBITS_LOWMASK	0x0000FFFF
	#define IOBITS_HIGHMASK	0xFFFF0000
	typedef unsigned int MOTORBITS;
	typedef unsigned long IOBITS;
#else
	#assert Too many motor devices (MAX_MOTORS > 16)
#endif

#define IOBITS_CNT	(2*MAX_MOTORS)


typedef char MOTOR_CTRL;

#if   (MOTOR_MAXRUNTIME/TIMER_MS)<=UINT8_MAX
	#define MAX_MOTOR_TIMER UINT8_MAX
	typedef byte MOTOR_TIMER;
#elif (MOTOR_MAXRUNTIME/TIMER_MS)<=UINT16_MAX
	#define MAX_MOTOR_TIMER UINT16_MAX
	typedef WORD MOTOR_TIMER;
#else
	#define MAX_MOTOR_TIMER UINT32_MAX;
	typedef DWORD MOTOR_TIMER;
#endif

#if   (FS20_SM8_IN_RESPONSE/TIMER_MS)<=UINT8_MAX
	typedef byte SM8_TIMEOUT;
#elif (FS20_SM8_IN_RESPONSE/TIMER_MS)<=UINT16_MAX
	typedef WORD SM8_TIMEOUT;
#else
	typedef DWORD SM8_TIMEOUT;
#endif



/* -------------------------------------------------------------------
 * EEPROM
 * -------------------------------------------------------------------*/

/* defines the number of writes of EEPROM struct into EEPROM after that
 * a simple wear-leveling algo will move the data within EEPROM */
#define EEPROM_WEARLEVELING_WRITECNT	1000

// EEPROM data
struct MYEEPROMHEADER {
	uint64_t Magic;
	uint32_t WriteCount;
};

struct MYEEPROM {
	// Struct  CRC32 Checksum
	// Contains the crc32 sum of this structure (without CRC32 itself)
	DWORD		CRC32;

	// Version of struct
	// Should be changed via DATAVERSION whenever structure changed
	// to initalize it with defaults
	byte		DataVersion;

	/* LED-Pattern
	 * <LEDPattern> are bitwise copied to LED, starting with bit 0
	 * right-sifted <LEDBitCount> times with a delay of <LEDBitLenght>
	 * between each bit.
	 * Default pattern: 0010.0000  0000.1000  0000.0010  0000.0000
	 * using 30 bits and 100 ms delay each bit */
	// LED normal pattern
	LEDPATTERN 	LEDPatternNormal;
	// LED rain pattern
	LEDPATTERN 	LEDPatternRain;
	// LED bit count
	byte 		LEDBitCount;
	// LED bit delay
	WORD 		LEDBitLenght;

	// Motor type bitmask
	MOTORBITS	MTypeBitmask;
	// Motor runtimes
	volatile DWORD MaxRuntime[MAX_MOTORS];
	// Motor overtravel times
	volatile DWORD OvertravelTime[MAX_MOTORS];
	// Motor names
	char 		MotorName[MAX_MOTORS][MAX_NAMELEN];
	// Last motor position
	volatile MOTOR_TIMER MotorPosition[MAX_MOTORS];

	// <Rain> function bits
	#define RAIN_BIT_AUTO	(1<<0)
	#define RAIN_BIT_ENABLE	(1<<1)
	#define RAIN_BIT_RESUME	(1<<2)
	byte 		Rain;
	// Rain resume delay time
	WORD 		RainResumeTime;

	// Command interface local echo flag
	bool 		Echo;
	// Command interface terminator
	char 		Term;
	// Auto status flag
	bool 		SendStatus;
	// Opration timer
	DWORD		OperatingHours;
	// Login auto timeout (0=function disabled)
	WORD		LoginTimeout;
	// Password
	DWORD		EncryptKey[4];
	char		Password[17];
};


/* -------------------------------------------------------------------
 * enums
 * -------------------------------------------------------------------*/
// Motor type bitmask bit definition
enum MTYPE {JALOUSIE = 0, WINDOW};

// Status types
enum STATUSTYPE {SYSTEM, MOTOR, FS20OUT, FS20IN, PUSHBUTTON, RAIN};

// Encrypt/Decrypt modes
enum CRYPT_MODE{ ENCRYPT, DECRYPT };

// Help function table elements
enum PRINTCMDTYPE {PRINT_CMD = 0,PRINT_PARM,PRINT_DESC,PRINT_PDESC,PRINT_PROTECT};


/* -------------------------------------------------------------------
 * helper
 * -------------------------------------------------------------------*/
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

// external millis() timer variable
extern unsigned long timer0_millis;
