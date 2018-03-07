#include "Arduino.h"
#include "Wire.h"
#include "MCP23017.h"
#include "defines.h"

MCP23017::MCP23017() {
	Wire.begin();
}


uint8_t MCP23017::begin(const uint8_t address) {
	
	Serial.print("Initializing MCP23xxx at address 0x");
	Serial.println(address, HEX);
	Serial.println();

	writeRegister(MCP_REG_IOCON, 0x60);

  return 0;
}

uint8_t MCP23017::pinMode(const uint8_t pin, const uint8_t mode) {

	uint8_t directionRegister = MCP_REG_IODIRA;
	uint8_t pullup_register = MCP_REG_GPPUA;
	uint8_t directionRegisterData;
	uint8_t pullup_registerData;

	uint8_t pin_offset = 0;

	if (pin >= 8) {
		directionRegister = MCP_REG_IODIRB;
		pullup_register = MCP_REG_GPPUB;
		pin_offset = 8;
	}

	directionRegisterData = readRegister(directionRegister);

	switch (mode) {
		case INPUT:
			directionRegisterData |= ((1 << pin - pin_offset));
    	break;
		case OUTPUT:
			directionRegisterData &= ~(1 << (pin - pin_offset));
		break;
		case INPUT_PULLUP:
			pullup_registerData = readRegister(pullup_register);
			directionRegisterData |= (1 << (pin - pin_offset));
			pullup_registerData |= (1 << (pin - pin_offset));
			writeRegister(pullup_register, pullup_registerData);
		break;
	}

	return writeRegister(directionRegister, directionRegisterData);
}

uint8_t MCP23017::pinMode(const bool port, const uint8_t pin, const uint8_t mode) {

	uint8_t directionRegister = MCP_REG_IODIRA;
	uint8_t pullup_register = MCP_REG_GPPUA;
	uint8_t directionRegisterData;
	uint8_t pullup_registerData;

	if (port == PORT_B) {
		directionRegister = MCP_REG_IODIRB;
		pullup_register = MCP_REG_GPPUB;
	}

	directionRegisterData = readRegister(directionRegister);

	switch (mode) {
		case INPUT:
			directionRegisterData |= (1 << pin);
    	break;
		case OUTPUT:
			directionRegisterData &= ~(1 << pin);
		break;
		case INPUT_PULLUP:
			pullup_registerData = readRegister(pullup_register);
			directionRegisterData |= (1 << pin);
			pullup_registerData |= (1 << pin);
			writeRegister(pullup_register, pullup_registerData);
		break;
	}

	return writeRegister(directionRegister, directionRegisterData);
}

uint8_t MCP23017::portMode(const bool port, const uint8_t mode) {

	uint8_t directionRegister = MCP_REG_IODIRA;
	uint8_t pullup_register = MCP_REG_GPPUA;
	uint8_t directionRegisterData;
	uint8_t pullup_registerData;

	if (port == PORT_B) {
		directionRegister = MCP_REG_IODIRB;
		pullup_register = MCP_REG_GPPUB;
	}

	directionRegisterData = readRegister(directionRegister);

	switch (mode) {
		case INPUT:
      	directionRegisterData = 0xFF;
    break;
		case OUTPUT:
			directionRegisterData = 0x00;
		break;
		case INPUT_PULLUP:
			directionRegisterData = 0xFF;
			pullup_registerData = 0xFF;
			writeRegister(pullup_register, pullup_registerData);
		break;
	}

	return writeRegister(directionRegister, directionRegisterData);
}

// ************ Interrupt *******************

uint8_t MCP23017::enableInterrupt(const uint8_t pin, const uint8_t intMode) {


	uint8_t GPINTENregister = MCP_REG_GPINTENA;
	uint8_t INTCONregister = MCP_REG_INTCONA;
  uint8_t DEFVALregister = MCP_REG_DEFVALA;
	uint8_t pin_offset = 0;

  if (pin >= 8) {
    GPINTENregister = MCP_REG_GPINTENB;
		INTCONregister = MCP_REG_INTCONB;
    DEFVALregister = MCP_REG_DEFVALB;
		pin_offset = 8;
  }

	uint8_t GPINTENregisterData = readRegister(GPINTENregister);
	uint8_t INTCONregisterData = readRegister(INTCONregister);
  uint8_t DEFVALregisterData = 0; // Only need when necessary

	GPINTENregisterData |= (1 << (pin - pin_offset));	// Enable Interrupt for pin

  switch (intMode) {
    case IF_CHANGED:
			INTCONregisterData &= ~(1 << (pin - pin_offset));
      break;
    case IF_LOW:
			INTCONregisterData |= (1 << (pin - pin_offset));
			DEFVALregisterData = readRegister(DEFVALregister);
			DEFVALregisterData |= (1 << (pin - pin_offset));	// default to high
			writeRegister(DEFVALregister, DEFVALregisterData);
      break;
    case IF_HIGH:
			INTCONregisterData |= (1 << (pin - pin_offset));
			DEFVALregisterData = readRegister(DEFVALregister);
			DEFVALregisterData &= ~(1 << (pin - pin_offset));	// default to low
			writeRegister(DEFVALregister, DEFVALregisterData);
      break;
  }

	writeRegister(INTCONregister, INTCONregisterData);
	uint8_t retval = writeRegister(GPINTENregister, GPINTENregisterData); // Enable interrupts last

	clearInterrupts();

	return retval;
}


uint8_t MCP23017::disableInterrupt(const uint8_t pin) {

	uint8_t MCPregister = MCP_REG_GPINTENA;

  if (pin >= 8) {
    	MCPregister = MCP_REG_GPINTENB;
  }

  uint8_t registerData = readRegister(MCPregister);

	registerData &= ~(1 << pin);
  
	return writeRegister(MCPregister, registerData);
}

uint8_t MCP23017::disableInterrupt(const bool port, const uint8_t pin) {

	uint8_t MCPregister = MCP_REG_GPINTENA;

  if (port == PORT_B) {
    	MCPregister = MCP_REG_GPINTENB;
  }

  uint8_t registerData = readRegister(MCPregister);

	registerData &= ~(1 << pin);
  
	return writeRegister(MCPregister, registerData);
}

uint8_t MCP23017::disableInterruptPort(const uint8_t port) {

	uint8_t MCPregister = MCP_REG_GPINTENA;

  if (port == PORT_B) {
    	MCPregister = MCP_REG_GPINTENB;
  }
  
	return writeRegister(MCPregister, 0x00);
}

uint8_t MCP23017::getInterruptPin() {

	uint16_t INTFRegisterData = (readRegister(MCP_REG_INTFB) << 8) | readRegister(MCP_REG_INTFA);

	uint8_t intPin = 0;

	for (uint8_t x = 0; x <= 15; x++) {
		if (INTFRegisterData & (1 << x)) { // If bit is set this is the one
			intPin = x;
		}
	}

	return intPin;
}

uint8_t MCP23017::getInterruptPinValue(const uint8_t pin) {

	uint8_t MCPregister = MCP_REG_INTCAPA;
	uint8_t pin_offset = 0;
	bool pinValue = 0;

  if (pin >= 8) {
    	MCPregister = MCP_REG_INTCAPB;
			pin_offset = 8;
  }

	if (readRegister(MCPregister) & (1 << (pin - pin_offset) )) { // If the bit is set
		pinValue = 1;
	}

	return pinValue;
}

void MCP23017::clearInterrupts() {
	readRegister(MCP_REG_INTCAPA);
	readRegister(MCP_REG_INTCAPB);
}

// ********** Input control functions  ***********

uint8_t MCP23017::invertInput(const uint8_t pin, const bool state) {

	uint8_t MCPregister = MCP_REG_IOPOLA;
	uint8_t pin_offset = 0;

  if (pin >= 8) {
    	MCPregister = MCP_REG_IOPOLB;
			pin_offset = 8;
  }

  uint8_t registerData = readRegister(MCPregister);

  if (state == LOW) {
		registerData &= ~(1 << (pin - pin_offset) );
  }
  else {
		registerData |= (1 << (pin - pin_offset) );
  }

  return writeRegister(MCPregister, registerData);
}

uint8_t MCP23017::invertInput(const bool port, const uint8_t pin, const bool state) {

	uint8_t MCPregister = MCP_REG_IOPOLA;

  if (port == PORT_B) {
    	MCPregister = MCP_REG_IOPOLB;
  }

  uint8_t registerData = readRegister(MCPregister);

  if (state == LOW) {
		registerData &= ~(1 << pin);
  }
  else {
		registerData |= (1 << pin);
  }

  return writeRegister(MCPregister, registerData);
}

uint8_t MCP23017::invertInputPort(const bool port, const bool state) {

	uint8_t MCPregister = MCP_REG_IOPOLA;

  if (port == PORT_B) {
    	MCPregister = MCP_REG_IOPOLB;
  }
	
	uint8_t registerData;

  if (state == LOW) {
		registerData = 0x00;
  }
  else {
		registerData = 0xFF;
  }

  return writeRegister(MCPregister, registerData);

}


// *********  Reading functions ***********

bool MCP23017::digitalRead(const uint8_t pin) {

	uint8_t MCPregister = MCP_REG_GPIOA;
	uint8_t pin_offset = 0;

  if (pin >= 8) {
    	MCPregister = MCP_REG_GPIOB;
			pin_offset = 8;
  }

  return bitRead(readRegister(MCPregister), (pin - pin_offset)); // TODO ugly bitread
}

bool MCP23017::digitalRead(const bool port, const uint8_t pin) {

	uint8_t MCPregister = MCP_REG_GPIOA;

  if (port == PORT_B) {
    	MCPregister = MCP_REG_GPIOB;
  }

	return bitRead(readRegister(MCPregister), pin); // TODO ugly bitread
}

uint8_t MCP23017::readPort(const bool port) {

	uint8_t MCPregister = MCP_REG_GPIOA;

  if (port == PORT_B) {
    	MCPregister = MCP_REG_GPIOB;
  }

  return readRegister(MCPregister);
}


// ********* Writing functions **********

uint8_t MCP23017::digitalWrite(const uint8_t pin, const bool state) {

	uint8_t MCPregister = MCP_REG_GPIOA;
	bool port = PORT_A;
	uint8_t pin_offset = 0;

  if (pin >= 8) {
    	MCPregister = MCP_REG_GPIOB;
			port = PORT_B;
			pin_offset = 8;
  }

  if (state == LOW) {
		GPIO_shadow[port] &= ~(1 << (pin - (pin_offset) ));
  }
  else {
		GPIO_shadow[port] |= (1 << (pin - (pin_offset) ));
  }

  return writeRegister(MCPregister, GPIO_shadow[port]);
}


uint8_t MCP23017::digitalWrite(const bool port, const uint8_t pin, const bool state) {

	uint8_t MCPregister = MCP_REG_GPIOA;

  if (port == PORT_B) {
    	MCPregister = MCP_REG_GPIOB;
  }

  if (state == LOW) {
		GPIO_shadow[port] &= ~(1 << pin);
  }
  else {
		GPIO_shadow[port] |= (1 << pin);
  }

  return writeRegister(MCPregister, GPIO_shadow[port]);
}

uint8_t MCP23017::writePort(const bool port, const bool state) {

	uint8_t MCPregister = MCP_REG_GPIOA;

  if (port == PORT_B) {
    	MCPregister = MCP_REG_GPIOB;
  }

  if (state == LOW) {
		GPIO_shadow[port] = 0x00;
  }
  else {
		GPIO_shadow[port] = 0xFF;
  }

  return writeRegister(MCPregister, GPIO_shadow[port]);
}

/********** Private functions start ***********/

uint8_t MCP23017::readRegister(const uint8_t registerAddr) {
  
  	Wire.beginTransmission(chipAddr);
  	Wire.write(registerAddr);
  	Wire.endTransmission();
  	Wire.requestFrom(chipAddr, (uint8_t)1);
  
  	return Wire.read();
}


uint8_t MCP23017::writeRegister(const uint8_t registerAddr, const uint8_t registerData) {

  	Wire.beginTransmission(chipAddr);
  	Wire.write(registerAddr);
  	Wire.write(registerData);
  	
	return Wire.endTransmission();
}

// TODO rework for either 2x 8bit or 1x 16bit
uint8_t MCP23017::writeBothReg(const uint8_t MCPregister, const uint8_t data) {

	Wire.beginTransmission(MCP_REG_ADDRESS);
	Wire.write(MCPregister);
	Wire.write(data);  // port A
	Wire.write(data);  // port B
	
	return Wire.endTransmission();
}
