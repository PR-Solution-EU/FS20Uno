// MCP23017 registers (everything except direction defaults to 0)

#define IODIRA		0x00   // IO direction  (0 = output, 1 = input (Default))
#define IODIRB		0x01
#define IOPOLA		0x02   // IO polarity   (0 = normal, 1 = inverse)
#define IOPOLB		0x03
#define GPINTENA	0x04   // Interrupt on change (0 = disable, 1 = enable)
#define GPINTENB	0x05
#define DEFVALA		0x06   // Default comparison for interrupt on change (interrupts on opposite)
#define DEFVALB		0x07
#define INTCONA		0x08   // Interrupt control (0 = interrupt on change from previous, 1 = interrupt on change from DEFVAL)
#define INTCONB		0x09
#define IOCON		0x0A   // IO Configuration: bank/mirror/seqop/disslw/haen/odr/intpol/notimp
//#define IOCON		0x0B  // same as 0x0A
#define GPPUA		0x0C   // Pull-up resistor (0 = disabled, 1 = enabled)
#define GPPUB		0x0D
#define INFTFA		0x0E   // Interrupt flag (read only) : (0 = no interrupt, 1 = pin caused interrupt)
#define INFTFB		0x0F
#define INTCAPA		0x10   // Interrupt capture (read only) : value of GPIO at time of last interrupt
#define INTCAPB		0x11
#define GPIOA		0x12   // Port value. Write to change, read to obtain value
#define GPIOB		0x13
#define OLLATA		0x14   // Output latch. Write to latch output.
#define OLLATB		0x15

// for serial access
#define IODIR		0x00   // IO direction  (0 = output, 1 = input (Default))
#define IOPOL		0x02   // IO polarity   (0 = normal, 1 = inverse)
#define GPINTEN		0x04   // Interrupt on change (0 = disable, 1 = enable)
#define DEFVAL		0x06   // Default comparison for interrupt on change (interrupts on opposite)
#define INTCON		0x08   // Interrupt control (0 = interrupt on change from previous, 1 = interrupt on change from DEFVAL)
#define GPPU		0x0C   // Pull-up resistor (0 = disabled, 1 = enabled)
#define INFTF		0x0E   // Interrupt flag (read only) : (0 = no interrupt, 1 = pin caused interrupt)
#define INTCAP		0x10   // Interrupt capture (read only) : value of GPIO at time of last interrupt
#define GPIO		0x12   // Port value. Write to change, read to obtain value
#define OLLAT		0x14   // Output latch. Write to latch output.
