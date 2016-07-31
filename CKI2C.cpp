/*
	CKI2C.cpp - Chipkit I2C master library
	Copyright (c) 2016 P Joyce.  All right reserved.

	This is a modified version of the i2c master library by Wayne Truchsess, 
	which in turn was a modified version of the Arduino Wire/TWI 
	library.  Functions were rewritten for Chipkit compatibility.

	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 2.1 of the License, or (at your option) any later version.

	This library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public
	License along with this library; if not, write to the Free Software
	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include <WProgram.h>
#include "CKI2C.h"

uint8_t CKI2C::bytesAvailable = 0;
uint8_t CKI2C::bufferIndex = 0;
uint8_t CKI2C::totalBytes = 0;
uint16_t CKI2C::timeOutDelay = 2;

CKI2C::CKI2C()
{
}

////////////// Public Methods ////////////////////////////////////////

void CKI2C::begin()
{
	I2C1BRG = 0x0c2; //initialize baud clock 100 KHz.  Todo: make this a calculation
	I2C1CONSET = (1<<15); // turn on
	delayMicroseconds(50); // probably not necessary
	I2C1STAT = 0; //Clear the flags	 
}

void CKI2C::end()
{
	I2C1CON = 0; //turn off module
}

void CKI2C::timeOut(uint16_t _timeOut)
{
  timeOutDelay = _timeOut;
}



//beginTransmission sends start and device address (shifted left x1, LSB = 0)
uint8_t CKI2C::beginTransmission(uint8_t deviceAddress)
{
	resultCode = start(SEN);
	if(resultCode)
	{
	  return(resultCode); //fail if not 0
	}
	resultCode = sendAddress(SLA_W(deviceAddress));
	return(resultCode);
}
uint8_t CKI2C::beginTransmission(int deviceAddress)
{
	return(beginTransmission((uint8_t) deviceAddress));
}

uint8_t CKI2C::available()
{
	return(bytesAvailable);
}

uint8_t CKI2C::receive()
{
	bufferIndex = totalBytes - bytesAvailable;
	if(!bytesAvailable)
	{
		bufferIndex = 0;
		return(0);
	}
	bytesAvailable--;
	return(data[bufferIndex]);
}

  
/*return values for new functions that use the timeOut feature 
  will now return at what point in the transmission the timeout
  occurred. Looking at a full communication sequence between a 
  master and slave (transmit data and then readback data) there
  a total of 7 points in the sequence where a timeout can occur.
  These are listed below and correspond to the returned value:
  1 - Waiting for successful completion of a Start bit
  2 - Waiting for ACK/NACK while addressing slave in transmit mode (MT)
  3 - Waiting for ACK/NACK while sending data to the slave
  4 - Waiting for successful completion of a Repeated Start
  5 - Waiting for ACK/NACK while addressing slave in receiver mode (MR)
  6 - Waiting for ACK/NACK while receiving data from the slave
  7 - Waiting for successful completion of the Stop bit

  All possible return values:
  0           Function executed with no errors
  1 - 7       Timeout occurred, see above list
  8 - 0xFF    See datasheet for exact meaning */ 


/////////////////////////////////////////////////////
uint8_t CKI2C::write(int data)
{
	return(write((uint8_t) data));
}	

uint8_t CKI2C::write(uint8_t data)
{
	resultCode = sendByte(data);
	return(resultCode);
}

//this is not tested
uint8_t CKI2C::write(uint8_t *data, uint8_t numberBytes)
{
	resultCode = 0;
	for (uint8_t i = 0; i < numberBytes; i++)
	{
		resultCode = sendByte(data[i]);
		if(resultCode)
		{
			return(resultCode);
		}
	}
	// resultCode = stop();
	return(resultCode);
}
uint8_t CKI2C::endTransmission(void)
{
	resultCode = stop();
	return(resultCode);
}

uint8_t CKI2C::requestFrom(int address, int numberBytes)
{
	return(requestFrom((uint8_t) address, (uint8_t) numberBytes));
}

uint8_t CKI2C::requestFrom(uint8_t address, uint8_t numberBytes)
{
	bytesAvailable = 0;
	bufferIndex = 0;
	if(numberBytes == 0)
	{
		numberBytes++;
	}
	nack = numberBytes - 1;
	resultCode = 0;
	resultCode = start(SEN);
	if(resultCode)
	{
		return(resultCode);
	}
	resultCode = sendAddress(SLA_R(address));
	if(resultCode)
	{
		return(resultCode);
	}
	for(uint8_t i = 0; i < numberBytes; i++)
	{
		if( i == nack )
		{
			resultCode = receiveByte(1); //signal NACK
			if(resultCode)
			{
				return(resultCode);
			}
		}
		else
		{
			resultCode = receiveByte(0); //signal ACK
			if(resultCode)
			{
				return(resultCode);
			}
		}
		data[i] = I2C1RCV;
		bytesAvailable = i+1;
		totalBytes = i+1;
	}
	resultCode = stop();
	return(resultCode);
}


uint8_t CKI2C::requestFrom(uint8_t address, uint8_t numberBytes, uint8_t *dataBuffer)
{
	bytesAvailable = 0;
	bufferIndex = 0;
	if(numberBytes == 0)
	{
		numberBytes++;
	}
	nack = numberBytes - 1;
	resultCode = 0;
	resultCode = start(SEN);
	if(resultCode)
	{
		return(resultCode);
	}
	resultCode = sendAddress(SLA_R(address));
	if(resultCode)
	{
		return(resultCode);
	}
	for(uint8_t i = 0; i < numberBytes; i++)
	{
		if( i == nack )
		{
			resultCode = receiveByte(1);
			if(resultCode)
			{
				return(resultCode);
			}
		}
		else
		{
			resultCode = receiveByte(0);
			if(resultCode)
			{
				return(resultCode);
			}
		}
		dataBuffer[i] = I2C1RCV;
		bytesAvailable = i+1;
		totalBytes = i+1;
	}
	resultCode = stop();
	return(resultCode);
}



/////////////// Private Methods ////////////////////////////////////////


uint8_t CKI2C::start(uint16_t StartType)
{
	unsigned long startingTime = millis();
	//todo: for multi master mode check if another start is in process, if so reset and try again later
	I2C1CONSET = (StartType); // Sent Start or Repeated Start (.0 or .1)
	while  (!INTERRUPT)
	{
		if(!timeOutDelay){continue;}
		if((millis() - startingTime) >= timeOutDelay)
		{
		  resetBus();
		  return(1);
		}
	}	
	ClearInterrupt();

	if (STARTFLAG) //start bit detected
	{
		ShowStart();
		return(0);
	}
	if (COLLISION) //Was there a collision?
	{
		ShowCollision();
		resetBus();
		return(2);
	}
	ShowCollision();
	resetBus();	
	return(3); //returning for some other reason	

}

uint8_t CKI2C::sendAddress(uint8_t i2cAddress)
{
//returns 0 if ACK received (OK)
//returns 1 if NAC received and STOP sent
//Returns 2 if there was a bus collision and the bus was reset or lockup
	I2C1TRN = i2cAddress;
	unsigned long startingTime = millis();
	while  (!INTERRUPT)
	{
		if(!timeOutDelay){continue;}
		if((millis() - startingTime) >= timeOutDelay)
		{
			resetBus();
			return(2);
		}
	}
	ClearInterrupt();	
	if (!(I2C1STAT & bit(15))) //if ACK received
	{
		return(0);
	}
// it wasn't an ACK, was it a collision?
	if (COLLISION) //Was there a collision?
	{
		ShowCollision(); //
		resetBus();
		return(2);
	}	
//Not an ACK or a collision, must be  NAC, send stop and return	
	stop();
	return(1); //flag that a NAC was received and that a stop was issued
}

uint8_t CKI2C::sendByte(uint8_t i2cData)
{
	I2C1TRN = i2cData;
	unsigned long startingTime = millis();
	while  (!INTERRUPT) //wait for interrupt
	{
		if(!timeOutDelay){continue;}
		if((millis() - startingTime) >= timeOutDelay)
		{
			resetBus();
			return(1);
		}
	}
	ClearInterrupt();	
	if (!(I2C1STAT & bit(15))) //If ACK bit is low return normal
	{
		return(0);
	}
// it wasn't an ACK, was it a collision?
	if (COLLISION) //Was there a collision?
	{
		ShowCollision();
		resetBus();
		return(3);
	}
	{
		resetBus();
		return(4); //some other failure`
	} 
}

uint8_t CKI2C::receiveByte(uint8_t ack)
{
	// digitalWrite(J1P20, LOW);
	I2C1CONSET = bit(3); // enable receives RCEN 12	

	unsigned long startingTime = millis();
	while (!INTERRUPT) //Wait for interrupt (rbf)
	{
		if(!timeOutDelay){continue;}
		if((millis() - startingTime) >= timeOutDelay)
		{
		  resetBus();
		  return(10);
		}
	}
	ClearInterrupt();		
	if(ack == 0)
	{
		I2C1CONCLR = bit(5); // CLEAR ACKDT to generate a ACK 13
	}
	else
	{
		I2C1CONSET = bit(5); // Set ACKDT to generate a NAC 13

	}	
	I2C1CONSET = bit(4); // Send ACKEN to initiate the NAC event 13		
	while (!INTERRUPT) //Wait for interrupt
	{
		if(!timeOutDelay){continue;}
		if((millis() - startingTime) >= timeOutDelay)
		{
		  resetBus();
		  return(11);
		}
	}
	ClearInterrupt();		
	if (COLLISION) //Was there a collision?
	{
		ShowCollision();
		resetBus();
		return(12);
	}
	return(0); 
}


uint8_t CKI2C::stop()
{
	I2C1CONSET = bit(2); // send stop bit 14	
	unsigned long startingTime = millis();
	while  (!INTERRUPT)
	{
		if(!timeOutDelay){continue;}
		if((millis() - startingTime) >= timeOutDelay)
		{
		  resetBus();
		  return(1);
		}
	}	
	ClearInterrupt();
	if (STOPFLAG) //stop bit detected
	{
		Showstop();
		return(0);
	}
	if (COLLISION) //Was there a collision?
	{
		ShowCollision();
		resetBus();
		return(2);
	}
	ShowCollision();
	resetBus();	
	return(1); //returning for some other reason	
}


void CKI2C::resetBus()
{
	// digitalWrite(J1P3, HIGH);	
	I2C1CON = 0; //Clear all bits
	I2C1STAT = 0; 
	pinMode(SCL1, OPEN);	
// Add additional clocks in case slave is waiting for more clocks to release bus.
	for (int i=0; i<30; i++)  
	{
		digitalWrite(SCL1,LOW);
		delayMicroseconds(20);
		digitalWrite(SCL1,HIGH);
		delayMicroseconds(20);
	}
	ClearInterrupt();
	delayMicroseconds(10);	
	I2C1CONSET = bit(15); 
	// digitalWrite(J1P3, LOW);		
}
void CKI2C::ClearInterrupt(void)
{
	ShowInterrupt();	
	IFS1 &= ~bit(12); //Clear the interrupt
	ShowInterrupt();
}
// delete the following when done

void CKI2C::ShowInterrupt(void)
{
	if INTERRUPT 
	{
		// digitalWrite(J1P6, HIGH);	
	}
	else
	{
		// digitalWrite(J1P6, LOW);
	}
}

void CKI2C::ShowStart(void)
{
	// digitalWrite(J1P19, HIGH);		
	// digitalWrite(J1P19, LOW);	
}	
void CKI2C::Showstop(void)
{
	// digitalWrite(J1P14, HIGH);		
	// digitalWrite(J1P14, LOW);	
}	
void CKI2C::ShowCollision(void)
{
	// digitalWrite(J1P20, HIGH);		
	// digitalWrite(J1P20, LOW);	
}
CKI2C Wire = CKI2C();


