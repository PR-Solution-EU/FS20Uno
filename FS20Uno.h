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


/* Motor control values:
 *  0: Motor off
 * >0: Motor opening    1=immediately, >1=delay in ms before opening)
 * <0: Motor closeing  -1=immediately, <1=abs(delay) in ms before closing)
 *                        delay values are in ms/TIMER_MS
 */
#define MOTOR_OPEN			1
#define MOTOR_OPEN_DELAYED	(MOTOR_SWITCHOVER/TIMER_MS)
#define MOTOR_CLOSE			-1
#define MOTOR_CLOSE_DELAYED	(-MOTOR_SWITCHOVER/TIMER_MS)
#define MOTOR_OFF			0


// Serial interface baudrate
#define SERIAL_BAUDRATE				115200

// Number of motors within system
#define MAX_MOTORS					8

// Length of motor names
#define MAX_NAMELEN					21

// Timer2 intervall in ms
#define TIMER_MS					10

// Watchdog Timer
// Possible values (ms): 15,30,60,120,250,500,1000,2000,4000,8000
#define WATCHDOG_TIME				4000
// Watchdog Timer for REBOOT command
#define WATCHDOG_REBOOT_DELAY		250

// Alive timer period in ms
#define ALIVE_TIMER					500

/* ===================================================================
 * Default values
 * ===================================================================*/
// Relais closing time in ms (Datasheet 5 ms)
// Relais opening time in ms (Datasheet 4 ms))
#define RELAIS_OPERATE_TIME			20

// Motor switchover delay in ms
#define MOTOR_SWITCHOVER			250

// Motor maximum runtime in ms
#define MOTOR_MAXRUNTIME		 	655350
#define MOTOR_WINDOW_MAXRUNTIME		60000
#define MOTOR_JALOUSIE_MAXRUNTIME	20000

// SM8 IN circuit time in ms
#define FS20_SM8_IN_RESPONSE		150
// SM8 IN circuit for programmer mode in ms
#define FS20_SM8_IN_PROGRAMMODE		6000

// Key debounce time in ms
// Rain sensor input
#define RAIN_DEBOUNCE_TIME			20
// Pushbuttons
#define WPB_DEBOUNCE_TIME			80
// SM8 outputs
#define SM8_DEBOUNCE_TIME			20

// Motor type bitmask
enum mtype
{
	JALOUSIE = 0,
	WINDOW = 1
} MTYPE;
#define MTYPE_BITMASK				0b01010101


// LED Pattern
#define MAX_LEDPATTERN_BITS			32
typedef uint32_t					LEDPATTERN;

// 0010.0000  0000.1000  0000.0010  0000.0000
// LED 100 30 0x20080200
#define LED_PATTERN_NORMAL			0x20080200
// 1101.1111  1111.0111  1111.1101  1111.1111
// LED 100 30 0x20080200 0xdff7fdff
// 0000.0101  0000.0000  0000.0011  1111.1111
// LED 100 30 0x20080200 0x050003FF
// 0010.1000  0000.1010  0000.0010  1000.0000
// LED 100 30 0x20080200 0x280a0280
// 1101.0111  1111.0101  1111.1101  0111.1111
// LED 100 30 0x20080200 0xd7f5fd7f
#define LED_PATTERN_RAIN			0xd7f5fd7f
// Bit counts to use from pattern
#define LED_BIT_COUNT				30
// LED Bit Length in ms
#define LED_BIT_LENGTH				100


/* ===================================================================
 * Type & Constant Definition
 * ===================================================================*/
typedef unsigned int  	WORD;
typedef unsigned long 	DWORD;
typedef unsigned long	TIMER;



// EEPROM data addresses
#define EEPROM_ADDR_CRC32				0
#define EEPROM_ADDR_DATAVERSION			(EEPROM_ADDR_CRC32				+sizeof(unsigned long) )
#define EEPROM_ADDR_EEPROMDATA			(EEPROM_ADDR_DATAVERSION		+sizeof(byte) )

/* Rain dry window resume position delay */
#define DEFAULT_RAINRESUMETIME		30

#define DEFAULT_SendStatus		false
#define DEFAULT_Echo				false
#define DEFAULT_Term				'\r'

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

#define NO_POSITION 			MAX_MOTOR_TIMER
#define NO_RESUME_POSITION		UINT8_MAX
#define NO_RESUME_DELAY 		MAX_MOTOR_TIMER

// Status types
enum statusType
{
	SYSTEM = 0,
	MOTOR = 1,
	FS20OUT = 2,
	FS20IN = 3,
	PUSHBUTTON = 4,
	RAIN = 5
} STATUSTYPE;

// CRC types
enum crcType
{
	EEPROMCRC = 0,
	RAMCRC = 1
} CRCTYPE;

enum printCmdType
{
	 PRINT_CMD
	,PRINT_PARM
	,PRINT_DESC
	,PRINT_PDESC
} PRINTCMDTYPE;


// external millis() timer variable
extern unsigned long timer0_millis;

// helper macros
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
