#include "Arduino.h"
#include "Wire.h"
#include "MCP23008.h"

MCP23008::MCP23008() {}


// Initialize MCP23008
uint8_t MCP23008::begin(const uint8_t address) {

	chipAddr = address;

	Wire.begin();

  return writeRegister(MCP_REG_IOCON, 0x20);		// Disable sequencial mode
}

#ifdef ESP8266
uint8_t MCP23008::begin(const uint8_t address, const uint8_t pinSDA, const uint8_t pinSCL) {

	chipAddr = address;

	Wire.begin(pinSDA, pinSCL);

  return writeRegister(MCP_REG_IOCON, 0x20);		// Disable sequencial mode
}
#endif

uint8_t MCP23008::pinMode(const uint8_t pin, const uint8_t pinMode) {

  uint8_t pullupRegisterData;
  uint8_t directionRegisterData = readRegister(MCP_REG_IODIR);

  switch (pinMode) {
    case INPUT:
      directionRegisterData |= (1 << pin);
      break;
    case OUTPUT:
      directionRegisterData &= ~(1 << pin);
      break;
    case INPUT_PULLUP:
      pullupRegisterData = readRegister(MCP_REG_GPPU);
      directionRegisterData |= (1 << pin);
      pullupRegisterData |= (1 << pin);
      writeRegister(MCP_REG_GPPU, pullupRegisterData);
      break;
  }

  return writeRegister(MCP_REG_IODIR, directionRegisterData);
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
      writeRegister(MCP_REG_GPPU, 0xFF);
      break;
  }

  return writeRegister(MCP_REG_IODIR, directionRegisterData);
}

uint8_t MCP23008::invertInput(const uint8_t pin, const bool state) {

  uint8_t registerData = readRegister(MCP_REG_IOPOL);

  if (state == LOW) {
		registerData &= ~(1 << pin);
  }
  else {
		registerData |= (1 << pin);
  }

  return writeRegister(MCP_REG_IOPOL, registerData);
}

uint8_t MCP23008::readPort() {
  return readRegister(MCP_REG_GPIO);
}

uint8_t MCP23008::digitalRead(const uint8_t pin) {
  return bitRead(readRegister(MCP_REG_GPIO), pin);
}

uint8_t MCP23008::writePort(const bool state) {

  if (state == LOW) {
		GPIO_shadow = 0x00;
  }
  else {
		GPIO_shadow = 0xFF;
  }

	return writeRegister(MCP_REG_GPIO, GPIO_shadow);
}

uint8_t MCP23008::digitalWrite(const uint8_t pin, const bool state) {

  if (state == LOW) {
		GPIO_shadow &= ~(1 << pin);
  }
  else {
		GPIO_shadow |= (1 << pin);
  }

  return writeRegister(MCP_REG_GPIO, GPIO_shadow);
}

uint8_t MCP23008::enableInterrupt(const uint8_t pin, const uint8_t intMode) {

	uint8_t GPINTENregisterData = readRegister(MCP_REG_GPINTEN);
	uint8_t INTCONregisterData = readRegister(MCP_REG_INTCON);
  uint8_t DEFVALregisterData = 0; // Only need when necessary

	GPINTENregisterData |= (1 << pin);	// Enable Interrupt for pin

  switch (intMode) {
    case IF_CHANGED:
			INTCONregisterData &= ~(1 << pin);
      break;
    case IF_LOW:
			INTCONregisterData |= (1 << pin);
			DEFVALregisterData = readRegister(MCP_REG_DEFVAL);
			DEFVALregisterData |= (1 << pin);	// default to high
			writeRegister(MCP_REG_DEFVAL, DEFVALregisterData);
      break;
    case IF_HIGH:
			INTCONregisterData |= (1 << pin);
			DEFVALregisterData = readRegister(MCP_REG_DEFVAL);
			DEFVALregisterData &= ~(1 << pin);	// default to low
			writeRegister(MCP_REG_DEFVAL, DEFVALregisterData);
      break;
  }

	writeRegister(MCP_REG_INTCON, INTCONregisterData);
	uint8_t retval = writeRegister(MCP_REG_GPINTEN, GPINTENregisterData); // Enable interrupts last

	clearInterrupts();

	return retval;
}

uint8_t MCP23008::disableInterrupt(const uint8_t pin) {

	uint8_t registerData = readRegister(MCP_REG_GPINTEN);

	registerData &= ~(1 << pin);
  
	return writeRegister(MCP_REG_GPINTEN, registerData);
}

uint8_t MCP23008::intPinMode(const uint8_t intPinMode) {

	uint8_t registerData = readRegister(MCP_REG_IOCON);

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

  return writeRegister(MCP_REG_IOCON, registerData);
}

uint8_t MCP23008::getInterruptPin() {

	uint8_t INTFRegisterData = readRegister(MCP_REG_INTF);
	uint8_t intPin = 0;

	for (uint8_t x = 0; x <= 7; x++) {
		if (INTFRegisterData & (1 << x)) { // If bit is set this is the one
			intPin = x;
		}
	}

	return intPin;
}

uint8_t MCP23008::getInterruptPortValue() {
	return readRegister(MCP_REG_INTCAP);
}

uint8_t MCP23008::getInterruptPinValue(const uint8_t pin) {

	bool pinValue = 0;

	if (readRegister(MCP_REG_INTCAP) & (1 << pin)) { // If the bit is set
		pinValue = 1;
	}

	return pinValue;
}

void MCP23008::clearInterrupts() {
	readRegister(MCP_REG_INTCAP);
}

/********** Private functions start ***********/

uint8_t MCP23008::readRegister(const uint8_t registerAddr) {
  
  	Wire.beginTransmission(chipAddr);
  	Wire.write(registerAddr);
  	Wire.endTransmission();
  	Wire.requestFrom(chipAddr, (uint8_t)1);
  
  	return Wire.read();
}


uint8_t MCP23008::writeRegister(const uint8_t registerAddr, const uint8_t registerData) {

  	Wire.beginTransmission(chipAddr);
  	Wire.write(registerAddr);
  	Wire.write(registerData);
  	
	return Wire.endTransmission();
}
