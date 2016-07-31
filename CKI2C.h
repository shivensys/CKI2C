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
	void ShowInterrupt();
	void ClearInterrupt();
	void ShowStart();
	void Showstop();
	void ShowCollision();
	const int SCL1 = 38;
	const int SDA1 = 4;
	// const int PGD = 10;
	// const int PGC = 11;
	// const int J1P3 = 33; 
	// const int J1P5 = 27; 
	// const int J1P6 = 31; 
	// const int J1P7 = 27; 
	// const int J1P8 = 32; 
	// const int J1P9 = 30; 
	// const int J1P10 = 15;
	// const int J1P11 = 14;
	// const int J1P12 = 15;
	// const int J1P13 = 7;
	// const int J1P14 = 6;
	// const int J1P15 = 24;
	// const int J1P17 = 16;
	// const int J1P18 = 29;
	// const int J1P19 = 35;
	// const int J1P20 = 13;	

};

extern CKI2C Wire;

#endif
