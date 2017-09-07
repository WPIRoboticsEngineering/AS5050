//////////////////////////////////////////////////////////////////////////
//                     libAS5050
// This library aims to provide easy and convenient
// communication with the AS5050 magnetic rotary encoder IC.
//////////////////////////////////////////////////////////////////////////
// Written and maintained by Dan Sheadel (tekdemo@gmail.com)
// Code available at https://github.com/tekdemo/AS5050
//////////////////////////////////////////////////////////////////////////
//
// This program is free software; you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will
// be useful, but WITHOUT ANY WARRANTY; without even the
// implied warranty of MERCHANTABILITY or FITNESS FOR A
// PARTICULAR PURPOSE.  See the GNU General Public
// License for more details.
//
// You should have received a copy of the GNU General
// Public License along with this program; if not, write
// to the Free Software Foundation, Inc.,
// 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
//
// Licence can be viewed at
// http://www.fsf.org/licenses/gpl.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
//
//////////////////////////////////////////////////////////////////////////

#ifndef _AS5050_H_INCLUDED
#define _AS5050_H_INCLUDED

//Must include standard libs!
#include "mbed.h"
#include <SPI.h>



//Various library options for tweaking behaviors
#ifndef AS5050_AUTO_ERROR_HANDLING
  #define AS5050_AUTO_ERROR_HANDLING 0
#endif

#ifndef AS5050_RESET_ON_ERRORS
  #define AS5050_RESET_ON_ERRORS 0
#endif

//Command values for reading and writing
#define AS_WRITE  (0x0000)
#define AS_READ   (1<<15)

typedef uint8_t byte;

/*AS5050 Register values
Shifted 1 to make room for parity bit
OR with AS_READ/AS_WRITE before sending command
*/
#define REG_POWER_ON_RESET (0x3F22)
#define REG_SOFTWARE_RESET (0x3C00)
#define REG_MASTER_RESET   (0x33A5)
#define REG_CLEAR_ERROR    (0x3380)/*Read to reset errors*/
#define REG_NOP            (0x0000)
#define REG_GAIN_CONTROL   (0x3FF8)
#define REG_ANGLE          (0x3FFF)
#define REG_ERROR_STATUS   (0x33A5)
#define REG_CHIP_STATUS    (0x3F20)

//Data packets when writing the software Reset regarding clearing SPI or not
#define DATA_SWRESET_SPI    (0x11)
#define DATA_SWRESET_NO_SPI (0x00)

//Ways to check the responses from the AS5050 when
#define RES_PARITY         (0x1)
#define RES_ERROR_FLAG     (0x2)
#define RES_ALARM_HIGH     (0x8000)   /*Magnetic Field too high*/
#define RES_ALARM_LOW      (0x4000)   /*Magnetic Field too low*/
#define RES_ERROR          (RES_PARITY|RES_ERROR_FLAG)  //standard error frame
//error from reading the angle
#define RES_ERROR_ANGLE    (RES_PARITY| RES_ERROR_FLAG|RES_ALARM_HIGH|RES_ALARM_LOW)

//Error Status Register
/*
Set to high when the transmitted parity bit does not match to calculated
parity bit.
*/
#define ERR_PARITY    (1<<0)
/*
Set to high when the amount of clock cycles is not correct.
*/
#define ERR_CLKMON    (1<<1)
//Set to high when non existing address is used
#define ERR_ADDMON    (1<<2)
/*
When a READ ANGLE command is in progress, the WOW flag is set to 1. At the
end of the measurement the WOW flag is cleared to 0. Only in case of
deadlock the WOW flag is stuck high; in which case a MASTER RESET must be
sent to clear the deadlock.
*/
#define ERR_WOW       (1<<4)
/*
The ADCOV bit occurs if the magnetic input field strength is too large for at
least one Hall element. This can be the case if the magnet is displaced. Second
reason could be that the offset compensation after power up is not finished
yet. If this happens some dummy READ ANGLE commands may be sent to
settle the offset loop.
*/
#define ERR_ADCOV     (1<<8) ///#define RES_ERR_RESERVED6  (1<<7)
//#define RES_ERR_RESERVED7  (1<<6)
/*
The CORDIC calculates the angle. An error occurs when the input signals of the
CORDIC are too large. The internal algorithm fails.
*/
#define ERR_CORDICOV  (1<<9)
/*
The RANGE flag signals that the Hall bias circuit has reached the head room
limit. This might occur at the combination of low supply voltage, high
temperature and low magnetic field. In this case, manually reducing the AGC
setting (Figure 28) can be used to recover a valid Hall biasing condition.
*/
#define ERR_RANERR    (1<<10)	//Range error
#define ERR_DSPALO    (1<<2)	//AGC level is equal or even higher than the maximum level. Magnetic field is too weak.
#define ERR_DSPAHI    (1<<1)	//AGC level is equal or even lower than the minimum level. Magnetic field is too strong
//#define RES_ERR_RESERVED13  (1<<0)

//Set various limits on some values
#define AS5050_ANGULAR_RESOLUTION ((1<<12)-1)
#define AS5050_MAX_GAIN 31 //5 bits of storage
#define AS5050_MIN_GAIN 0
#define  AS5050_ALARM_BITMASK ((1<<13)|(1<<12))

//Implementation specific value of tau
#define AS5050_TAU 6.283185307179586232

//use unions to quickly parse multi-byte data from SPI
union spi_data{
  unsigned int value;
  struct{
    byte lsb; //lowest byte in struct
    byte msb; //next up
  } bytes;
};



class AS5050{
  public:
    AS5050(PinName mosi_pin, PinName miso_pin, PinName clk_pin, PinName ss_pin);
    AS5050(SPI* commonSpi,  PinName ss_pin);

    void begin(SPI* commonSpi,DigitalOut*);

    unsigned int send(unsigned int);
    unsigned int status();
    unsigned int read(unsigned int);
    unsigned int write(unsigned int,unsigned int);
    unsigned int handleErrors();
    //These functions return the physical angle on the chip
    int angle();
    float angleDegrees();
    float angleRad();
    //These functions return the absolute angle from initial startup
    long int totalAngle();
    float totalAngleDegrees();
    float totalAngleRad();
    //These functions return the deltas from initial startup, so it changes on start position
    long int deltaAngle();
    float deltaAngleDegrees();
    float deltaAngleRad();
    //Reset the home the deltas/totals will use
    void setHome();

    //


    //Make errors accessible from outside the class if needed
    struct error_struct{
	byte parity;		//will be non-zero if there's a parity error
	unsigned int transaction;      //holds the error data from angular reads
	unsigned int status;           //holds the AS5050 error data from other sources
    }error;

    //Store the current gain value, so it can be recorded
    byte gain;

    //Keep track of how many full rotations we've gone through
    int rotations;
	bool mirrored;
  unsigned int data;
  unsigned int angleData;

    private:
        void loadError();

        PinName _ss_pin;
        SPI *_spi;
        DigitalOut *_cs;
        int _last_angle;
        int _init_angle;
};

#endif
