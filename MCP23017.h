#ifndef _MCP23017_h
#define _MCP23017_h

#define MCP_REG_IOCON     0x0A   // IO Configuration: bank/mirror/seqop/disslw/haen/odr/intpol/notimp
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

#define PORT_A 0
#define PORT_B 1

#define BIT_INTPOL 1
#define BIT_ODR 2

// Interrupts
#define IF_LOW 0
#define IF_HIGH 1
#define IF_CHANGED 2
#define ACTIVE_LOW 0
#define ACTIVE_HIGH 1
#define OPEN_DRAIN 2


class MCP23017 {

	public:
		MCP23017();

		uint8_t begin(const uint8_t address);

#ifdef ESP8266
		uint8_t begin(const uint8_t address, const uint8_t pinSDA, const uint8_t pinSCL);
#endif

		uint8_t pinMode(uint8_t pin, const uint8_t mode);
		uint8_t pinMode(const bool port, const uint8_t pin, const uint8_t mode);
		uint8_t portMode(const bool port, const uint8_t mode);

		uint8_t enableInterrupt(const uint8_t pin, const uint8_t intMode);
		uint8_t disableInterrupt(const uint8_t pin);
		uint8_t disableInterruptPort(const uint8_t port);
		uint8_t disableInterrupt(const bool port, const uint8_t pin);
		uint8_t getInterruptPin();
		uint8_t getInterruptPinValue(const uint8_t pin);
		void clearInterrupts();

		uint8_t invertInput(const uint8_t pin, const bool state);
		uint8_t invertInput(const bool port, const uint8_t pin, const bool state);
		uint8_t invertInputPort(const bool port, const bool state);

		bool digitalRead(const uint8_t pin);
		bool digitalRead(const bool port, const uint8_t pin);
		uint8_t readPort(const bool port);

		uint8_t digitalWrite(const uint8_t pin, const bool state);
		uint8_t digitalWrite(const bool port, const uint8_t pin, const bool state);
	  uint8_t writePort(const bool port, const bool state);

		private:

		// TODO remove
 		int MCP_REG_ADDRESS = 0x20;	// Default to address 0x20 (All address pins low)

		uint8_t chipAddr = 0x20;	// Default to address 0x20 (All address pins low)

		uint8_t readRegister(const uint8_t registerAddr);
		uint8_t writeRegister(const uint8_t registerAddr, const uint8_t registerData);
		uint8_t writeBothReg(const uint8_t register, const uint8_t data); // TODO rework

		uint8_t GPIO_shadow[2] = { 0x00 };
};


#endif
