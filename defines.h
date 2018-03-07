#ifndef _MCP_DEFINES_H_
#define _MCP_DEFINES_H_

#define PORT_A 0
#define PORT_B 1

// MCP23017 registers (everything except direction defaults to 0)


/*
// BANK1 Mode
#define MCP_REG_IODIRA    0x00
#define MCP_REG_IOCON    0x05   // IO Configuration: bank/mirror/seqop/disslw/haen/odr/intpol/notimp
#define MCP_REG_IOPOLA    0x01   // IO polarity   (0 = normal, 1 = inverse)
#define MCP_REG_GPINTENA  0x02   // Interrupt on change (0 = disable, 1 = enable)
#define MCP_REG_DEFVALA   0x03   // Default comparison for interrupt on change (interrupts on opposite)
#define MCP_REG_INTCON    0x04   // Interrupt control (0 = change from previous, 1 = change from DEFVAL)
#define MCP_REG_GPPUA     0x06   // Pull-up resistor (0 = disabled, 1 = enabled)
#define MCP_REG_INFTFA    0x07   // Interrupt flag (read only) : (0 = no interrupt, 1 = pin caused interrupt)
#define MCP_REG_INTCAPA   0x08   // Interrupt capture (read only) : value of GPIO at time of last interrupt
#define MCP_REG_GPIOA     0x09   // Port value. Write to change, read to obtain value
#define MCP_REG_OLLATA    0x0A   // Output latch. Write to latch output. *** Read only reads latch, not port pin! ***

#define MCP_REG_IODIRB    0x10
#define MCP_REG_IOPOLB    0x11   // IO polarity   (0 = normal, 1 = inverse)
#define MCP_REG_GPINTENB  0x12   // Interrupt on change (0 = disable, 1 = enable)
#define MCP_REG_DEFVALB   0x13   // Default comparison for interrupt on change (interrupts on opposite)
#define MCP_REG_INTCONB   0x14   // Interrupt control (0 = change from previous, change from DEFVAL)
#define MCP_REG_GPPUB     0x16   // Pull-up resistor (0 = disabled, 1 = enabled)
#define MCP_REG_INFTFB    0x17   // Interrupt flag (read only) : (0 = no interrupt, 1 = pin caused interrupt)
#define MCP_REG_INTCAPB   0x18   // Interrupt capture (read only) : value of GPIO at time of last interrupt
#define MCP_REG_GPIOB     0x19   // Port value. Write to change, read to obtain value
#define MCP_REG_OLLATB    0x1A   // Output latch. Write to latch output. *** Read only reads latch, not port pin! ***
*/

// BANK0 Mode
#define MCP_REG_IOCON    0x0A   // IO Configuration: bank/mirror/seqop/disslw/haen/odr/intpol/notimp

#define MCP_REG_IODIRA    0x00
#define MCP_REG_IODIRB    0x01
#define MCP_REG_IOPOLA    0x02   // IO polarity   (0 = normal, 1 = inverse)
#define MCP_REG_IOPOLB    0x03
#define MCP_REG_GPINTENA  0x04   // Interrupt on change (0 = disable, 1 = enable)
#define MCP_REG_GPINTENB  0x05
#define MCP_REG_DEFVALA   0x06   // Default comparison for interrupt on change (interrupts on opposite)
#define MCP_REG_DEFVALB   0x07
#define MCP_REG_INTCONA   0x08   // Interrupt control (0 = interrupt on change from previous, 1 = interrupt on change from DEFVAL)
#define MCP_REG_INTCONB   0x09
#define MCP_REG_GPPUA     0x0C   // Pull-up resistor (0 = disabled, 1 = enabled)
#define MCP_REG_GPPUB     0x0D
#define MCP_REG_INTFA    0x0E   // Interrupt flag (read only) : (0 = no interrupt, 1 = pin caused interrupt)
#define MCP_REG_INTFB    0x0F
#define MCP_REG_INTCAPA   0x10   // Interrupt capture (read only) : value of GPIO at time of last interrupt
#define MCP_REG_INTCAPB   0x11
#define MCP_REG_GPIOA     0x12   // Port value. Write to change, read to obtain value
#define MCP_REG_GPIOB     0x13
#define MCP_REG_OLLATA    0x14   // Output latch. Write to latch output. *** Read only reads latch, not port pin! ***
#define MCP_REG_OLLATB    0x15

#endif
