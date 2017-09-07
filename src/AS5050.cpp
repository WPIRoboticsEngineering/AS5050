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



#include "mbed.h"
#include "AS5050.h"

AS5050::AS5050(SPI* commonSpi, PinName ss_pin){
	  /*CONSTRUCTOR
	  * Sets up the required values for the function, and configures the
	  * hardware SPI to operate correctly
	  */
	  _ss_pin = ss_pin;

	  begin(commonSpi, new DigitalOut(_ss_pin));

	  //Prepare the chip
	  write(REG_MASTER_RESET,0x0); //do a full reset in case the chip glitched in the last power cycle
	  //Read angle twice to initialize chip and get to a known good state
	  //Reading once won't work, as _last_angle will be set incorrectly
	  angle();
	  _init_angle=angle();
	  //angle() will glitch on startup if it's >768, reset it
	  rotations=0;

	  //By default, we don't want the angle to be reversed
	  mirrored=true;
}


AS5050::AS5050(PinName mosi_pin, PinName miso_pin, PinName clk_pin, PinName ss_pin){
  /*CONSTRUCTOR
  * Sets up the required values for the function, and configures the
  * hardware SPI to operate correctly
  */
//  _mosi_pin = mosi_pin;
//  _miso_pin = miso_pin;
//  _clk_pin = clk_pin;
  _ss_pin = ss_pin;

  begin(new SPI(mosi_pin, miso_pin, clk_pin), new DigitalOut(_ss_pin));

  //Prepare the chip
  write(REG_MASTER_RESET,0x0); //do a full reset in case the chip glitched in the last power cycle
  //Read angle twice to initialize chip and get to a known good state
  //Reading once won't work, as _last_angle will be set incorrectly
  angle();
  _init_angle=angle();
  //angle() will glitch on startup if it's >768, reset it
  rotations=0;

  //By default, we don't want the angle to be reversed
  mirrored=true;
};

void AS5050::begin(SPI *spi, DigitalOut *cs) {
  //Prepare the SPI interface
  this->_spi = spi;
  this->_cs = cs;

  // Deselect the chip
  this->_cs->write(1);
  this->_spi->format(8,1);
  this->_spi->frequency(1000000);
}

unsigned int AS5050::send(unsigned int reg_a){
  spi_data response,reg;
  reg.value=reg_a;
  //This function does not take care of parity stuff,
  //due to peculiarities with it.

  this->_spi->lock();
  this->_cs->write(0);  //Start Transaction
  //Send data in MSB order
  //response.value=this->_spi->write(reg.value);
  response.bytes.msb=this->_spi->write(reg.bytes.msb);
  response.bytes.lsb=this->_spi->write(reg.bytes.lsb);
  this->_cs->write(1);	//End Transaction
  this->_spi->unlock();

  return response.value;
};
unsigned int _my_parity(unsigned int reg){
  unsigned int total =0;
  //start right shifted by one
  for(int i=1;i<16;i++){
    if(reg&(1<<i)){
      total++;

    }
  }
}
unsigned int AS5050::read(unsigned int reg){
  /* Data packet looks like this:
  MSB |14 .......... 2|   1      | LSB
  R/W | ADRESS <13:0> | ERR_FLAG | |Parity
  */

  //Prepare data command for sending across the wire
  reg= (reg<<1) |(AS_READ); //make room for parity and set RW bi
  reg|= __builtin_parity(reg);  //set in the parity bit

  send(reg);              //send data
  reg = send(REG_NOP);              //send data


  return reg; //remove error and parity bits
}

unsigned int AS5050::write(unsigned int reg,unsigned int data){

  //Prepare register data
  reg=(reg<<1)|(AS_WRITE);      //add parity bit place and set RW bit
  reg|=__builtin_parity(reg);   //Set the parity bit

  //prepare data for transmit
  data=data<<2;                  //Don't care and parity placeholders
  data|=__builtin_parity(data);  //Set the parity bit on the data

  send(reg);          //send the register we wish to write to
  send(data);         //set the data we want in there
  data=send(REG_NOP); //Get confirmation from chip

  //save error and parity data
  error.parity|=__builtin_parity(data&(~RES_PARITY)) != (data&RES_PARITY); //save parity errors
  error.transaction=error.parity; //save error data
  error.transaction|=reg&RES_ERROR_FLAG;

  return data;      //remove parity and EF bits and return data.
};

// TODO: Check if works
unsigned int AS5050::status(){
  unsigned int data = read(REG_CHIP_STATUS);
  return data;
}

int AS5050::angle(){
  //This function strips out the error and parity
  //data in the data frame, and handles the errors
  data=read(REG_ANGLE);

  /* Response from chip is this:
   14 | 13 | 12 ... 2                       | 1  | 0
   AH | AL |  <data 10bit >                 | EF | PAR
  */

  //Automatically handle errors if we've enabled it
  #if AS5050_AUTO_ERROR_HANDLING==1
  if(data & AS5050_ALARM_BITMASK){
    loadError();
    //handleErrors(); // handle by user in non timer thread
    //If there's a parity error, the angle might be invalid so prevent glitching by swapping in the last angle
    if(error.transaction&RES_PARITY) return _last_angle;
  }
  #endif
  angleData=( (data>>2)// Shift the data over to wipe the dont car bit and parity
          &(AS5050_ALARM_BITMASK^0xffff)//Xor the two bits that are the alarm bits with 0xffff into all bits except those
        )
        &AS5050_ANGULAR_RESOLUTION; //strip away alarm bits, then parity and error flags

  //Allow the user to reverse the logical rotation
  //if(mirrored){angle=(AS5050_ANGULAR_RESOLUTION-1)-angle;}

  //track rollovers for continous angle monitoring
  double boundForWrap = AS5050_ANGULAR_RESOLUTION/6;
  double maxForWrap =(AS5050_ANGULAR_RESOLUTION-boundForWrap);
  if(_last_angle>maxForWrap && angleData<=boundForWrap)
    rotations+=1;
  else if(_last_angle<boundForWrap && angleData>=maxForWrap)
    rotations-=1;
  _last_angle=angleData;

  return angleData;

}
void AS5050::loadError(){
  error.status=read(REG_ERROR_STATUS);

}

float AS5050::angleDegrees(){
    //Rewrite of arduino's map function, to make sure we don't lose resolution
    //return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    return angle()*360/(float)AS5050_ANGULAR_RESOLUTION;
}
float AS5050::angleRad(){
  //return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return angle()*(AS5050_TAU)/(float)AS5050_ANGULAR_RESOLUTION;
}



long int AS5050::totalAngle(){
    return angle()+rotations*AS5050_ANGULAR_RESOLUTION ;
}
float AS5050::totalAngleDegrees(){
    return angleDegrees()+360*rotations;
}
float AS5050::totalAngleRad(){
    return angleDegrees()+AS5050_TAU*rotations;
}


long int AS5050::deltaAngle(){
    return (angle()-_init_angle)+rotations*AS5050_ANGULAR_RESOLUTION;
}
float AS5050::deltaAngleDegrees(){
    return (deltaAngle())*360/(float)AS5050_ANGULAR_RESOLUTION;
}
float AS5050::deltaAngleRad(){
     return (deltaAngle())*AS5050_TAU/(float)AS5050_ANGULAR_RESOLUTION;
}

void AS5050::setHome(){
    //Reset the home for the deltas/total functions
    rotations=0;
    _init_angle=0;
}

unsigned int AS5050::handleErrors(){  //now, handle errors:


	//If we don't have any standing errors, then quickly bypass all the checks
	if(error.status){
    //printf("\n\n----------------\nError Value %h \n\n",error.status);
		if(error.status & ERR_PARITY){
			//set high if the parity is wrong
			//Avoid doing something insane and assume we'll come back to
			//this function and try again with correct data
      //printf("\nERR_PARITY");

			//return error.status;
		}

		/*
		* Gain problems, automatically adjust
		*/
		if(error.status & ERR_DSPAHI){
      int gain=read(REG_GAIN_CONTROL);	//get information about current gain
			write(REG_GAIN_CONTROL,--gain); 	//increment gain and send it back
      //printf("\nERR_DSPAHI");

		}
		else if(error.status & ERR_DSPALO){
			int gain=read(REG_GAIN_CONTROL); 	//get information about current gain
			write(REG_GAIN_CONTROL,++gain); 	//increment gain and send it back
      //printf("\nERR_DSPALO");

		}

		/*
		* Chip Failures, can be fixed with a reset
		*/
		if(error.status & ERR_WOW){
			//After a read, this gets set low. If it's high, there's an internal
			//deadlock, and the chip must be reset
      //printf("\nERR_WOW");
			//write(REG_MASTER_RESET,0x0);
      //return error.status;


		}


		/*
		* Hardware issues. These need to warn the user somehow
		*/
		//TODO figure out some sane warning! This is not a good thing to have happen
		if(error.status & ERR_CORDICOV){
			//The CORDIC calculates the angle. An error occurs when the input signals of the
      //CORDIC are too large. The internal algorithm fails.
      //write(REG_SOFTWARE_RESET,DATA_SWRESET_SPI);
      //printf("\nERR_CORDICOV");
		}
		if(error.status & ERR_RANERR){
			//Accuracy is decreasing due to increased tempurature affecting internal current source
      //printf("\nERR_RANERR");

		}

		/*
		* Reasonably harmless errors that can be fixed without reset
		*/
		if(error.status & ERR_ADCOV){
			//V
// The ADCOV bit occurs if the magnetic input field strength is too large for at
// least one Hall element. This can be the case if the magnet is displaced. Second
// reason could be that the offset compensation after power up is not finished
// yet. If this happens some dummy READ ANGLE commands may be sent to
// settle the offset loop
      //printf("\nERR_ADCOV");
      //write(REG_MASTER_RESET,0x0);
      //return error.status;
		}
		if(error.status & ERR_CLKMON){
			//The clock cycles are not correct
      //printf("\nERR_CLKMON");

		}
		if(error.status & ERR_ADDMON){
			//set high when an address is incorrect for the last operation
      //printf("\nERR_ADDMON");

		}


	//If the error is still there, reset the AS5050 to attempt to fix it
	#if AS5050_RESET_ON_ERRORS==1
	//if(error.status)write(REG_SOFTWARE_RESET,DATA_SWRESET_SPI);
	if(error.status){
    write(REG_MASTER_RESET,0x0);
    //printf("\n\n Encoder System Reset \n\n");

  }
  #else
  	//This command returns 0 on successful clear
  	//otherwise, this command can handle it later
  	error.status=read(REG_CLEAR_ERROR) ;
	#endif
	}

	return error.status;
};
