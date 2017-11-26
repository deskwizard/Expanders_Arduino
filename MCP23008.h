#ifndef _MCP23008_h
#define _MCP23008_h

// Register definitions
#define MCP_REG_IODIR     0x00
#define MCP_REG_IOPOL     0x01   // IO polarity   (0 = normal, 1 = inverse)
#define MCP_REG_GPINTEN   0x02   // Interrupt on change (0 = disable, 1 = enable)
#define MCP_REG_DEFVAL    0x03   // Default comparison for interrupt on change (interrupts on opposite)
#define MCP_REG_INTCON    0x04   // Interrupt control (0 = change from previous, 1 = change from DEFVAL)
#define MCP_REG_IOCON     0x05   // IO Configuration: seqop/disslw/odr/intpol
#define MCP_REG_GPPU      0x06   // Pull-up resistor (0 = disabled, 1 = enabled)
#define MCP_REG_INTF      0x07   // Interrupt flag (read only) : (0 = no interrupt, 1 = pin caused interrupt)
#define MCP_REG_INTCAP    0x08   // Interrupt capture (read only) : value of GPIO at time of last interrupt
#define MCP_REG_GPIO      0x09   // Port value. Write to change, read to obtain value
#define MCP_REG_OLLAT     0x0A   // Output latch. Write to latch output. *** Read only reads latch, not port pin! ***

#define BIT_INTPOL 1
#define BIT_ODR 2


// Interrupts
#define IF_LOW 0
#define IF_HIGH 1
#define IF_CHANGED 2
#define ACTIVE_LOW 0
#define ACTIVE_HIGH 1
#define OPEN_DRAIN 2

class MCP23008 {

  public:
		MCP23008();
		
		uint8_t begin(const uint8_t address);

		uint8_t pinMode(const uint8_t pin, const uint8_t pinMode);
		uint8_t portMode(const uint8_t portMode);

		uint8_t invertInput(const uint8_t pin, const bool state);

		uint8_t portRead();
		uint8_t digitalRead(const uint8_t pin);

		uint8_t portWrite(const bool state);
		uint8_t digitalWrite(const uint8_t pin, const bool state);

		uint8_t enableInterrupt(const uint8_t pin, const uint8_t intMode);
		uint8_t disableInterrupt(const uint8_t pin);
		uint8_t intPinMode(const uint8_t intPinMode);
		uint8_t getInterruptPin();
		uint8_t getInterruptPortValue();
		uint8_t getInterruptPinValue(const uint8_t pin);
		void clearInterrupts();

#ifdef ESP8266
		uint8_t begin(const uint8_t address, const uint8_t pinSDA, const uint8_t pinSCL);
#endif

	private:
		uint8_t expanderBusAddress = 0x20;	// Default to address 0x20 (All address pins low)

		uint8_t readReg(const uint8_t reg);
		uint8_t writeReg(const uint8_t MCPregister, const uint8_t data);

		uint8_t GPIO_shadow = 0x00;

};


#endif
