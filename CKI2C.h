/*
  CKI2C.h   - Chipkit I2C Master library(polled)
  Copyright (c) 2016 P. Joyce.  All right reserved.
  
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
//#include <inttypes.h>

#ifndef CKI2C_h
#define CKI2C_h

//****new  & re-used defines****************
#define RSEN bit(1)
#define SEN  bit(0)
#define SLA_W(address)  (address << 1)
#define SLA_R(address)  ((address << 1) + 0x01)
#define MAX_BUFFER_SIZE 32
#define INTERRUPT (IFS1 & bit(12))
#define COLLISION (I2C1STAT & bit(10))
#define STARTFLAG I2C1STAT & bit(3)
#define STOPFLAG I2C1STAT & bit(4)

class CKI2C
{
  public:
    CKI2C();
    void begin();
    uint8_t beginTransmission(uint8_t);	
    uint8_t beginTransmission(int);
    uint8_t endTransmission(void);
    // uint8_t endTransmission(uint8_t);
    uint8_t write(int);
    uint8_t write(uint8_t);
    uint8_t write(uint8_t*, uint8_t);
    // uint8_t read(int*, int);
    // uint8_t read(uint8_t, uint8_t, uint8_t);
    // uint8_t read(int, int, int);
    // uint8_t read(uint8_t, uint8_t, uint8_t*);
    // uint8_t read(uint8_t, uint8_t, uint8_t, uint8_t*);
	
    void end();
    void timeOut(uint16_t);
    uint8_t available();
    uint8_t receive();

	
    uint8_t requestFrom(uint8_t, uint8_t); //device address, data_received
    uint8_t requestFrom(int, int); //device address, data_received
    uint8_t requestFrom(uint8_t, uint8_t, uint8_t*); //device address, numbytes, *databuffer

  private:
    uint8_t start(uint16_t);
    uint8_t sendAddress(uint8_t);
    uint8_t sendByte(uint8_t);
    uint8_t receiveByte(uint8_t);
    uint8_t stop();
    void resetBus();
    uint8_t resultCode;
    uint8_t nack;
    uint8_t data[MAX_BUFFER_SIZE];
    static uint8_t bytesAvailable;
    static uint8_t bufferIndex;
    static uint8_t totalBytes;
    static uint16_t timeOutDelay;
	void ClearInterrupt();
	const int SCL1 = 38; //FIXME
	const int SDA1 = 4; //FIXME
};

extern CKI2C Wire;

#endif
