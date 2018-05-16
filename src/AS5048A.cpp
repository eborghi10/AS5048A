#include "Arduino.h"

#include <AS5048A.h>

static const uint16_t AS5048A_CLEAR_ERROR_FLAG              = 0x0001;
static const uint16_t AS5048A_PROGRAMMING_CONTROL           = 0x0003;
static const uint16_t AS5048A_OTP_REGISTER_ZERO_POS_HIGH    = 0x0016;
static const uint16_t AS5048A_OTP_REGISTER_ZERO_POS_LOW     = 0x0017;
static const uint16_t AS5048A_DIAG_AGC                      = 0x3FFD;
static const uint16_t AS5048A_MAGNITUDE                     = 0x3FFE;
static const uint16_t AS5048A_ANGLE                         = 0x3FFF;

static const double 	  AS5048A_MAX_VALUE 					= 8191.0;

/**
 * Constructor
 */
AS5048A::AS5048A(byte cs, bool debug /*=false*/)
: _cs(cs)
, errorFlag(false)
, position(0)
, debug(debug)
{
}

/**
 * Initialiser
 * Sets up the SPI interface
 */
void AS5048A::begin(){
	setDelay();

	// 1MHz clock (AMS should be able to accept up to 10MHz)
	this->settings = SPISettings(3000000, MSBFIRST, SPI_MODE1);

	//setup pins
	pinMode(this->_cs, OUTPUT);

	//SPI has an internal SPI-device counter, it is possible to call "begin()" from different devices
	SPI.begin();
}

/**
 * Closes the SPI connection
 * SPI has an internal SPI-device counter, for each begin()-call the close() function must be called exactly 1 time
 */
void AS5048A::close(){
	SPI.end();
}

/**
 * Utility function used to calculate even parity of an unigned 16 bit integer
 */
uint8_t AS5048A::spiCalcEvenParity(uint16_t value){
	uint8_t cnt = 0;

	for (uint8_t i = 0; i < 16; i++)
	{
		if (value & 0x1)
		{
			cnt++;
		}
		value >>= 1;
	}
	return cnt & 0x1;
}



/**
 * Get the rotation of the sensor relative to the zero position.
 *
 * @return {int16_t} between -2^13 and 2^13
 */
int16_t AS5048A::getRotation(){
	uint16_t data;
	int16_t rotation;

	data = AS5048A::getRawRotation();
	rotation = static_cast<int16_t>(data) - static_cast<int16_t>(this->position);
	if(rotation > AS5048A_MAX_VALUE) rotation = -((0x3FFF)-rotation); //more than -180

	return rotation;
}

/**
 * Returns the raw angle directly from the sensor
 */
int16_t AS5048A::getRawRotation(){
	return AS5048A::read(AS5048A_ANGLE);
}

/**
  * Get the rotation of the sensor relative to the zero position in degrees.
  *
  * @return {double} between 0 and 360
  */

double AS5048A::getRotationInDegrees(){
	int16_t rotation = getRotation();
	double degrees = 360.0 * (rotation + AS5048A_MAX_VALUE) / (AS5048A_MAX_VALUE * 2.0);
	return degrees;
}

/**
  * Get the rotation of the sensor relative to the zero position in radians.
  *
  * @return {double} between 0 and 2 * PI
  */

double AS5048A::getRotationInRadians(){
	int16_t rotation = getRotation();
	double radians = PI * (rotation + AS5048A_MAX_VALUE) / AS5048A_MAX_VALUE;
	return radians;
}

/**
 * returns the value of the state register
 * @return unsigned 16 bit integer containing flags
 */
uint16_t AS5048A::getState(){
	return AS5048A::read(AS5048A_DIAG_AGC);
}

/**
 * Print the diagnostic register of the sensor
 */
void AS5048A::printState(){
	if (this->debug)
	{
		uint16_t data = AS5048A::getState();
		if(AS5048A::error()){
			Serial.print("Error bit was set!");
		}
		Serial.println(data, BIN);
	}
}

/**
 * Returns the value used for Automatic Gain Control (Part of diagnostic
 * register)
 */
uint8_t AS5048A::getGain(){
	uint16_t data = AS5048A::getState();
	return static_cast<uint8_t>(data & 0xFF);
}

/*
 * Get and clear the error register by reading it
 */
uint16_t AS5048A::getErrors(){
	return AS5048A::read(AS5048A_CLEAR_ERROR_FLAG);
}

/*
 * Set the zero position
 */
void AS5048A::setZeroPosition(uint16_t position){
	this->position = position % 0x3FFF;
}

/*
 * Returns the current zero position
 */
uint16_t AS5048A::getZeroPosition(){
	return this->position;
}

/*
 * Check if an error has been encountered.
 */
bool AS5048A::error(){
	return this->errorFlag;
}

/*
 * Read a register from the sensor
 * Takes the address of the register as an unsigned 16 bit
 * Returns the value of the register
 */
uint16_t AS5048A::read(uint16_t registerAddress){
	uint16_t command = 0b0100000000000000; // PAR=0 R/W=R
	command = command | registerAddress;

	//Add a parity bit on the the MSB
	command |= static_cast<uint16_t>(spiCalcEvenParity(command)<<0xF);

	if (this->debug)
	{
		Serial.print("Read (0x");
		Serial.print(registerAddress, HEX);
		Serial.print(") with command: 0b");
		Serial.println(command, BIN);
	}

	//SPI - begin transaction
	SPI.beginTransaction(this->settings);

	//Send the command
	digitalWrite(this->_cs, LOW);
	SPI.transfer16(command);
	digitalWrite(this->_cs,HIGH);

	delay(this->esp32_delay);

	//Now read the response
	digitalWrite(this->_cs, LOW);
	uint16_t response = SPI.transfer16(0x00);
	digitalWrite(this->_cs, HIGH);

	//SPI - end transaction
	SPI.endTransaction();

	if (this->debug)
	{
		Serial.print("Read returned: ");
		Serial.print(command, BIN);
	}

	//Check if the error bit is set
	if (response & 0x4000) {
		if (this->debug)
		{
			Serial.println("Setting error bit");
		}
		this->errorFlag = true;
	}
	else {
		this->errorFlag = false;
	}

	//Return the data, stripping the parity and error bits
	return response & ~0xC000;
}


/*
 * Write to a register
 * Takes the 16-bit  address of the target register and the unsigned 16 bit of data
 * to be written to that register
 * Returns the value of the register after the write has been performed. This
 * is read back from the sensor to ensure a sucessful write.
 */
uint16_t AS5048A::write(uint16_t registerAddress, uint16_t data) {

	uint16_t command = 0b0000000000000000; // PAR=0 R/W=W
	command |= registerAddress;

	//Add a parity bit on the the MSB
	command |= static_cast<uint16_t>(spiCalcEvenParity(command)<<0xF);

	if (this->debug)
	{
		Serial.print("Write (0x");
		Serial.print(registerAddress, HEX);
		Serial.print(") with command: 0b");
		Serial.println(command, BIN);
	}

	//SPI - begin transaction
	SPI.beginTransaction(this->settings);

	//Start the write command with the target address
	digitalWrite(this->_cs, LOW);
	SPI.transfer16(command);
	digitalWrite(this->_cs,HIGH);

	uint16_t dataToSend = 0b0000000000000000;
	dataToSend |= data;

	//Craft another packet including the data and parity
	dataToSend |= static_cast<uint16_t>(spiCalcEvenParity(dataToSend)<<0xF);

	if (this->debug)
	{
		Serial.print("Sending data to write: ");
		Serial.println(dataToSend, BIN);
	}

	//Now send the data packet
	digitalWrite(this->_cs,LOW);
	SPI.transfer16(dataToSend);
	digitalWrite(this->_cs,HIGH);

	delay(this->esp32_delay);

	digitalWrite(this->_cs, LOW);
	uint16_t response = SPI.transfer16(0x00);
	digitalWrite(this->_cs, HIGH);

	//SPI - end transaction
	SPI.endTransaction();

	//Return the data, stripping the parity and error bits
	return response & ~0xC000;
}

/**
 * Set the delay acording to the microcontroller architecture
 */
void AS5048A::setDelay()
{
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
	this->esp32_delay = 50;
	if (this->debug)
	{
		Serial.println("AS5048A working with ESP32");
	}
#elif __AVR__
	this->esp32_delay = 0;
	if (this->debug)
	{
		Serial.println("AS5048A working with AVR");
	}
#else
	this->esp32_delay = 0;
	if (this->debug)
	{
		Serial.println("Device not detected");
	}
#endif
}
