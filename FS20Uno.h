/* ===================================================================
 * Hardware definition (I/O Ports...)
 * Zeiten in ms müssen Vielfache von 10 sein
 * ===================================================================*/
// MCP23017 Adressen
#define MPC1    			0x0		// MCP23017 #1 I2C Adresse
#define MPC2    			0x1		// MCP23017 #2 I2C Adresse
#define MPC3				0x2		// MCP23017 #3 I2C Adresse
#define MPC4    			0x3		// MCP23017 #4 I2C Adresse

// MPC Funktionsnamen
#define MPC_MOTORRELAIS		MPC1	// MPC für Motorrelais
#define MPC_SM8BUTTON		MPC2	// MPC für FS20-SM8 Tasten
#define MPC_SM8STATUS		MPC3	// MPC für FS20-SM8 Ausgänge
#define MPC_WALLBUTTON		MPC4	// MPC für Wandtaster

#define MPC_INT_INPUT 		3		// Arduino Port für MPC Interrupteingang
#define STATUS_LED 			LED_BUILTIN	// LED

#define RAIN_INPUT 			4		// Arduino Port für Regensensor
#define RAIN_INPUT_AKTIV	0

#define RAIN_ENABLE			9		// Arduino Port für Aktivitätsschalter
#define RAIN_ENABLE_AKTIV	1


// Motor Steuerungskommandos:
//   0: Motor AUS
//  >0: Motor Öffnen
//  <0: Motor Schliessen
// abs(Werte) <> 0 haben folgende Bedeutung:
//   1: Motor sofort schalten
//  >1: Motor Delay in (abs(Wert) - 1) * 10 ms
#define MOTOR_OPEN			1
#define MOTOR_OPEN_DELAYED	(MOTOR_SWITCHOVER/TIMER_MS)
#define MOTOR_CLOSE			-1
#define MOTOR_CLOSE_DELAYED	(-MOTOR_SWITCHOVER/TIMER_MS)
#define MOTOR_OFF			0


// Serielles Interface Baudrate
#define SERIAL_BAUDRATE				115200

// Anzahl der Motoren
#define MAX_MOTORS					8

// Timer2 Intervall in ms
#define TIMER_MS					10

// Watchdog Timer
// Mögliche Werte (ms): 15,30,60,120,250,500,1000,2000,4000,8000
#define WATCHDOG_TIME				4000

/* ===================================================================
 * Standardwerte
 * ===================================================================*/
// Relais Ansprechzeit in ms (Datenblatt 5 ms)
// Relais Rückfallzeit in ms (Datenblatt 4 ms))
#define RELAIS_OPERATE_TIME			20

// Motor Umschaltdelay in ms
#define MOTOR_SWITCHOVER			250

// Motor maximale Laufzeit in ms
// Fenster auf:  52000 ms
// Fenster zu:   50000 ms
// Jalousie auf: 15000 ms
// Jalousie zu:  18000 ms
#define MOTOR_MAXRUNTIME		 	655350
#define MOTOR_WINDOW_MAXRUNTIME		60000
#define MOTOR_JALOUSIE_MAXRUNTIME	20000

// SM8 IN Schaltzeit in ms
#define FS20_SM8_IN_RESPONSE		150
// SM8 IN Schaltzeit für Programmiermodus in ms
#define FS20_SM8_IN_PROGRAMMODE		6000

// Tasten Entprellzeit in ms
// Eingang Regensensor
#define RAIN_DEBOUNCE_TIME			20
// Wandtaster Entprellzeit
#define WPB_DEBOUNCE_TIME			80
// SM89 Ausgänge Entprellzeit
#define SM8_DEBOUNCE_TIME			20

// Bitmask für Fenster Motoren (DFF=1, Jalousien=0)
enum mtype
{
	JALOUSIE = 0,
	WINDOW = 1
} MTYPE;
#define MTYPE_BITMASK				0b01010101

// LED Blink Interval in ms
#define LED_BLINK_INTERVAL			1900
// LED Blink Length in ms
#define LED_BLINK_LEN				100



/* ===================================================================
 * Type & Constant Definition
 * ===================================================================*/
typedef unsigned int  	WORD;
typedef unsigned long 	DWORD;
typedef unsigned long	TIMER;

// EEPROM Data Adressen
#define EEPROM_ADDR_CRC32				0
#define EEPROM_ADDR_DATAVERSION			(EEPROM_ADDR_CRC32				+sizeof(unsigned long) )
#define EEPROM_ADDR_EEPROMDATA			(EEPROM_ADDR_DATAVERSION		+sizeof(byte) )

// EEPROM Data Typen
#define EEPROM_ALL						0xffff
#define EEPROM_LED_BLINK_INTERVAL		(1<<0)
#define EEPROM_LED_BLINK_LEN			(1<<1)
#define EEPROM_MTYPE_BITMASK			(1<<2)
#define EEPROM_MOTOR_MAXRUNTIME			(1<<3)
#define EEPROM_MOTOR_NAME				(1<<4)
#define EEPROM_RAIN						(1<<5)
#define EEPROM_SENDSTATUS				(1<<6)
#define EEPROM_ECHO						(1<<7)
#define EEPROM_TERM						(1<<8)

/* Fenster Position Wiederherstellungsverzögerung
 * falls RAIN RESUME aktiv */
#define DEFAULT_RAINRESUMETIME		30

#define DEFAULT_CMDSENDSTATUS		false
#define DEFAULT_CMDECHO				false
#define DEFAULT_CMDTERM				'\r'

// TODO: MASK and BITS are not correct for all MAX_MOTORS values
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


#define IOBITS_CNT	(MAX_MOTORS * 2)

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

#define NO_POSITION MAX_MOTOR_TIMER
#define NO_POSITION MAX_MOTOR_TIMER
