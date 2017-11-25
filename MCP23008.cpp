#include "Arduino.h"
#include "Wire.h"
#include "MCP23008.h"

MCP23008::MCP23008() {}


// Initialize MCP23008
uint8_t MCP23008::begin(const uint8_t address) {

	expanderBusAddress = address;

	Wire.begin();

  return writeReg(MCP_REG_IOCON, 0x20);		// Disable sequencial mode
}

#ifdef ESP8266
uint8_t MCP23008::begin(const uint8_t address, const uint8_t pinSDA, const uint8_t pinSCL) {

	expanderBusAddress = address;

	Wire.begin(pinSDA, pinSCL);

  return writeReg(MCP_REG_IOCON, 0x20);		// Disable sequencial mode
}
#endif

uint8_t MCP23008::pinMode(const uint8_t pin, const uint8_t pinMode) {

  uint8_t pullupRegisterData;
  uint8_t directionRegisterData = readReg(MCP_REG_IODIR);

  switch (pinMode) {
    case INPUT:
      directionRegisterData |= (1 << pin);
      break;
    case OUTPUT:
      directionRegisterData &= ~(1 << pin);
      break;
    case INPUT_PULLUP:
      pullupRegisterData = readReg(MCP_REG_GPPU);
      directionRegisterData |= (1 << pin);
      pullupRegisterData |= (1 << pin);
      writeReg(MCP_REG_GPPU, pullupRegisterData);
      break;
  }

  return writeReg(MCP_REG_IODIR, directionRegisterData);
}

uint8_t MCP23008::portMode(const uint8_t portMode) {

  uint8_t directionRegisterData = 0;

  switch (portMode) {
    case INPUT:
      directionRegisterData = 0xFF;
      break;
    case OUTPUT:
      directionRegisterData = 0x00;
      break;
    case INPUT_PULLUP:
      directionRegisterData = 0xFF;
      writeReg(MCP_REG_GPPU, 0xFF);
      break;
  }

  return writeReg(MCP_REG_IODIR, directionRegisterData);
}

uint8_t MCP23008::invertInput(const uint8_t pin, const bool state) {

  uint8_t registerData = readReg(MCP_REG_IOPOL);

  if (state == LOW) {
		registerData &= ~(1 << pin);
  }
  else {
		registerData |= (1 << pin);
  }

  return writeReg(MCP_REG_IOPOL, registerData);
}

uint8_t MCP23008::portRead() {
  return readReg(MCP_REG_GPIO);
}

uint8_t MCP23008::digitalRead(const uint8_t pin) {
  return bitRead(readReg(MCP_REG_GPIO), pin);
}

uint8_t MCP23008::portWrite(const bool state) {

  if (state == LOW) {
		return writeReg(MCP_REG_GPIO, 0x00);
  }
  else {
		return writeReg(MCP_REG_GPIO, 0xFF);
  }

}

uint8_t MCP23008::digitalWrite(const uint8_t pin, const bool state) {

  uint8_t registerData = readReg(MCP_REG_GPIO);

  if (state == LOW) {
		registerData &= ~(1 << pin);
  }
  else {
		registerData |= (1 << pin);
  }

  return writeReg(MCP_REG_GPIO, registerData);
}

uint8_t MCP23008::enableInterrupt(const uint8_t pin, const uint8_t intMode) {

	uint8_t GPINTENregisterData = readReg(MCP_REG_GPINTEN);
	uint8_t INTCONregisterData = readReg(MCP_REG_INTCON);
  uint8_t DEFVALregisterData = 0; // Only need when necessary

	GPINTENregisterData |= (1 << pin);	// Enable Interrupt for pin

  switch (intMode) {
    case IF_CHANGED:
			INTCONregisterData &= ~(1 << pin);
      break;
    case IF_LOW:
			INTCONregisterData |= (1 << pin);
			DEFVALregisterData = readReg(MCP_REG_DEFVAL);
			DEFVALregisterData |= (1 << pin);	// default to high
			writeReg(MCP_REG_DEFVAL, DEFVALregisterData);
      break;
    case IF_HIGH:
			INTCONregisterData |= (1 << pin);
			DEFVALregisterData = readReg(MCP_REG_DEFVAL);
			DEFVALregisterData &= ~(1 << pin);	// default to low
			writeReg(MCP_REG_DEFVAL, DEFVALregisterData);
      break;
  }

	writeReg(MCP_REG_INTCON, INTCONregisterData);
	return writeReg(MCP_REG_GPINTEN, GPINTENregisterData); // Enable interrupts last
}

uint8_t MCP23008::disableInterrupt(const uint8_t pin) {

	uint8_t registerData = readReg(MCP_REG_GPINTEN);

	registerData &= ~(1 << pin);
  
	return writeReg(MCP_REG_GPINTEN, registerData);
}

uint8_t MCP23008::intPinMode(const uint8_t intPinMode) {

	uint8_t registerData = readReg(MCP_REG_IOCON);


	switch (intPinMode) {

		case ACTIVE_LOW:
			registerData &= ~(1 << BIT_ODR);		// ODR cleared
			registerData &= ~(1 << BIT_INTPOL);	// INTPOL cleared
			break;
		case ACTIVE_HIGH:
			registerData &= ~(1 << BIT_ODR);		// ODR cleared
			registerData |= (1 << BIT_INTPOL);	// INTPOL set
			break;
		case OPEN_DRAIN:
			registerData |= (1 << BIT_ODR);	 		// ODR set
			break;	
	}

  return writeReg(MCP_REG_IOCON, registerData);
}


uint8_t MCP23008::getInterrupts() {
// read INTF first, reading INTCAP clears interrupts
	uint8_t INTFRegisterData = readReg(MCP_REG_INTF);
	uint8_t INTCAPRegisterData = readReg(MCP_REG_INTCAP);

	if (INTCAPRegisterData == INTFRegisterData) {
		return INTCAPRegisterData;
	}
	else {
		return INTFRegisterData;
	}
}

/********** Private functions start ***********/

uint8_t MCP23008::readReg(const uint8_t reg) {
  
  	Wire.beginTransmission(expanderBusAddress);
  	Wire.write(reg);
  	Wire.endTransmission();
  	Wire.requestFrom(expanderBusAddress, (uint8_t)1);
  
  	return Wire.read();
}


uint8_t MCP23008::writeReg(const uint8_t MCPregister, const uint8_t data) {

  	Wire.beginTransmission(expanderBusAddress);
  	Wire.write(MCPregister);
  	Wire.write(data);
  	
	return Wire.endTransmission();
}
