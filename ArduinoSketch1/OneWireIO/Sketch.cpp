/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */



#include <PID_v1.h>
#include <Encoder.h>
#include <EnableInterrupt.h>
#include <OneWireSlave.h>
#include "Arduino.h"
//Beginning of Auto generated function prototypes by Atmel Studio
//End of Auto generated function prototypes by Atmel Studio



// This is the pin that will be used for one-wire data (depending on your arduino model, you are limited to a few choices, because some pins don't have complete interrupt support)
// On ATTiny85 you can use pin 2
Pin oneWireData(2);

// This is the ROM the arduino will respond to, make sure it doesn't conflict with another device
const byte owROM[7] = { 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02 };

// define the available commands

//The number of bytes read or written can be between 1 and 8.
const byte READ_SCRATCHPAD = 0xBE;
const byte WRITE_SCRATCHPAD = 0x4E;

//The scratchpad should contain the following content before the above command is sent to the device:
//Byte 0: The start address of where the data bytes should be retrieved from in the user EEPROM.
//Byte 1: The number of bytes to be retrieved from the user EEPROM (max of 8 bytes).
const byte COPY_EEPROM_TO_SCRATCHPAD = 0x37;
//The scratchpad should contain the following content before the above command is sent to the device:
//Byte 0: The start address of where the data bytes should be stored in the user EEPROM.
//Byte 1: The number of bytes to be stored to the user EEPROM (max of 8 bytes).
const byte COPY_SCRATCHPAD_TO_EEPROM = 0x39;

//The scratchpad should contain the following content before the above command is sent to the device:
//Byte 0: The start address of where the data bytes should be retrieved from in the user RAM.
//Byte 1: The number of bytes to be retrieved from the user RAM (max of 8 bytes).
const byte COPY_RAM_TO_SCRATCHPAD = 0x11;
//The scratchpad should contain the following content before the above command is sent to the device:
//Byte 0: The start address of where the data bytes should be stored in the user RAM.
//Byte 1: The number of bytes to be stored to the user RAM (max of 8 bytes).
const byte COPY_SCRATCHPAD_TO_RAM = 0x13;

//Re-initialize the servo PID values to their default
const byte SERVO_INITIALIZE = 0x03;
//Home the servo by reversing until it stops by stalling in the fully up position
const byte SERVO_HOME = 0x05;
//Copy the scratchpad to the servo program memory, then execute the program
const byte SERVO_RUN_SCRATCHPAD = 0x07;
//Poll the status of the servo
const byte SERVO_STATUS = 0x09;


// TODO:
// - handle configuration (resolution, alarms)

enum DeviceState
{
	DS_WaitingReset,
	DS_WaitingCommand,
	DS_WaitingByte,
	DS_ConvertingTemperature,
	DS_TemperatureConverted,
};
volatile DeviceState state = DS_WaitingReset;

// 8-byte scratchpad, with the CRC byte at the end
volatile byte scratchpad[9];
volatile byte scratchpad_index;

volatile unsigned long conversionStartTime = 0;

// This function will be called each time the OneWire library has an event to notify (reset, error, byte received)
void owReceive(OneWireSlave::ReceiveEvent evt, byte data);

//Define PID Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//Initialize the rotary encoder library
Encoder myEnc(3, 4);


void setup()
{
	// Setup the OneWire library
	OWSlave.setReceiveCallback(&owReceive);
	OWSlave.begin(owROM, oneWireData.getPinNumber());

	//Setup the PID library
	//turn the PID on
	myPID.SetMode(AUTOMATIC);
}

void loop()
{
	delay(10);
#ifdef INCLUDE_PID
	myPID.Compute();
#endif

#ifdef INCLUDE_ENCODER
	myEnc.read();
#endif

	cli();//disable interrupts
	// Be sure to not block interrupts for too long, OneWire timing is very tight for some operations. 1 or 2 microseconds (yes, microseconds, not milliseconds) can be too much depending on your master controller, but then it's equally unlikely that you block exactly at the moment where it matters.
	// This can be mitigated by using error checking and retry in your high-level communication protocol. A good thing to do anyway.
	DeviceState localState = state;
	unsigned long localConversionStartTime = conversionStartTime;
	sei();//enable interrupts

	if (localState == DS_ConvertingTemperature && millis() > localConversionStartTime + 750)
	{
		float temperature = 42.0f; // here you could plug any logic you want to return the emulated temperature
		int16_t raw = (int16_t)(temperature * 16.0f + 0.5f);

		byte data[9];
		data[0] = (byte)raw;
		data[1] = (byte)(raw >> 8);
		for (int i = 2; i < 8; ++i)
			data[i] = 0;
		data[8] = OWSlave.crc8(data, 8);

		cli();
		memcpy((void*)scratchpad, data, 9);
		state = DS_TemperatureConverted;
		OWSlave.writeBit(1, true); // now that conversion is finished, start sending ones until reset
		sei();
	}
}

// Start pulse has been detected; begin a 1-wire packet.
void owReceive(OneWireSlave::ReceiveEvent evt, byte data)
{
	switch (evt)
	{
	case OneWireSlave::RE_Byte:
		switch (state)
		{
		case DS_WaitingCommand:
			switch (data)
			{
			case DS18B20_START_CONVERSION:
				state = DS_ConvertingTemperature;
				conversionStartTime = millis();
				OWSlave.writeBit(0, true); // send zeros as long as the conversion is not finished
				break;

			case READ_SCRATCHPAD:
				state = DS_WaitingReset;
				OWSlave.write((const byte*)scratchpad, 17, 0);
				break;

			case WRITE_SCRATCHPAD:
				state = DS_WaitingByte;
				scratchpad_index = 0;
				break;
			}
			break;
		}
		case DS_WaitingByte:  //should be into a temporary buffer so the CRC can be checked before destroying the scratchpad.
			scratchpad[scratchpad_index++] = data;
			//Read up to 8 bytes into the scratchpad memory + CRC
			//then wait for reset.
			if (scratchpad_index<9)
				break;
			else
			{
				scratchpad_index = 0;
				state = DS_WaitingReset;
				break;
			}

				

		break;

	case OneWireSlave::RE_Reset:
		state = DS_WaitingCommand;
		break;

	case OneWireSlave::RE_Error:
		state = DS_WaitingReset;
		break;
	}
}
