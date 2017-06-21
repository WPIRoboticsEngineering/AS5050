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

AS5050::AS5050(PinName mosi_pin, PinName miso_pin, PinName clk_pin, PinName ss_pin){
  /*CONSTRUCTOR
  * Sets up the required values for the function, and configures the
  * hardware SPI to operate correctly
  */
  _mosi_pin = mosi_pin;
  _miso_pin = miso_pin;
  _clk_pin = clk_pin;
  _ss_pin = ss_pin;

  begin(new SPI(_mosi_pin, _miso_pin, _clk_pin), new DigitalOut(_ss_pin));

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

  this->_cs->write(0);  //Start Transaction

  //Send data in MSB order
  response.bytes.msb=this->_spi->write(reg.bytes.msb);
  response.bytes.lsb=this->_spi->write(reg.bytes.lsb);

  this->_cs->write(1);	//End Transaction

  // printf("Recieved \n", response.bytes.msb);

  return response.value;
};

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

  //Save the parity error for analysis
  error.parity=__builtin_parity(reg&(~RES_PARITY)) != (reg&RES_PARITY);
  error.transaction|=error.parity;
  error.transaction|=reg&RES_ERROR_FLAG;

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

  //save error data to the error register
  //Check parity of transaction
  //Save the parity error for analysis
  error.parity=__builtin_parity(data&(~RES_PARITY)) != (data&RES_PARITY);
  error.transaction|=error.parity;
  //Save angular errors
  error.transaction=(data|RES_ALARM_HIGH|RES_ALARM_LOW);



  //Automatically handle errors if we've enabled it
  #if AS5050_AUTO_ERROR_HANDLING==1
  if(error.transaction){
    error.status=read(REG_ERROR_STATUS);
    //handleErrors();
    //If there's a parity error, the angle might be invalid so prevent glitching by swapping in the last angle
    if(error.transaction&RES_PARITY) return _last_angle;
  }
  #endif

  //TODO this needs some work to avoid magic numbers
  unsigned int angle=((data)&0x3FF); //strip away alarm bits, then parity and error flags

  //Allow the user to reverse the logical rotation
  //if(mirrored){angle=(AS5050_ANGULAR_RESOLUTION-1)-angle;}

  //track rollovers for continous angle monitoring
  double boundForWrap = AS5050_ANGULAR_RESOLUTION/4;
  double maxForWrap =(AS5050_ANGULAR_RESOLUTION-boundForWrap);
  if(_last_angle>maxForWrap && angle<=boundForWrap)
    rotations+=1;
  else if(_last_angle<boundForWrap && angle>=maxForWrap)
    rotations-=1;
  _last_angle=angle;

  return angle;

}
int AS5050::angle(byte nsamples){
	int sum=0;
	for(byte i=0;i<nsamples;i++){
		sum+=angle();
	}
	//this performs fair rounding on arbitrary integers
	return (sum+nsamples/2)/nsamples;
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
    return (angle()-_init_angle)+rotations*1024;
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
		if(error.status & ERR_PARITY){
			//set high if the parity is wrong
			//Avoid doing something insane and assume we'll come back to
			//this function and try again with correct data
      printf("\n\n ERR_PARITY \n\n");

			return error.status;
		}

		/*
		* Gain problems, automatically adjust
		*/
		if(error.status & ERR_DSPAHI){
      int gain=read(REG_GAIN_CONTROL);	//get information about current gain
			write(REG_GAIN_CONTROL,--gain); 	//increment gain and send it back
      printf("\n\n ERR_DSPAHI \n\n");

		}
		else if(error.status & ERR_DSPALO){
			int gain=read(REG_GAIN_CONTROL); 	//get information about current gain
			write(REG_GAIN_CONTROL,++gain); 	//increment gain and send it back
      printf("\n\n ERR_DSPALO \n\n");

		}

		/*
		* Chip Failures, can be fixed with a reset
		*/
		if(error.status & ERR_WOW){
			//After a read, this gets set low. If it's high, there's an internal
			//deadlock, and the chip must be reset
			write(REG_SOFTWARE_RESET,DATA_SWRESET_SPI);
      printf("\n\n ERR_WOW \n\n");

		}
		if(error.status & ERR_DSPOV){
			//CORDIC overflow, meaning input signals are too large.
			//Gain adjustments should take care of this
			write(REG_SOFTWARE_RESET,DATA_SWRESET_SPI);
      printf("\n\nERR_DSPOV  \n\n");

		}

		/*
		* Hardware issues. These need to warn the user somehow
		*/
		//TODO figure out some sane warning! This is not a good thing to have happen
		if(error.status & ERR_DACOV){
			//This indicates a Hall effect sensor is being saturated by too large of
			//a magnetic field. This usually indicates a hardware failure such as a magnet
			//being displaced
      printf("\n\nERR_DACOV  \n\n");
		}
		if(error.status & ERR_RANERR){
			//Accuracy is decreasing due to increased tempurature affecting internal current source
      printf("\n\n ERR_RANERR \n\n");

		}

		/*
		* Reasonably harmless errors that can be fixed without reset
		*/
		if(error.status & ERR_MODE){
			//set high if the chip is measuring an angle, otherwise low
      printf("\n\nERR_MODE  \n\n");

		}
		if(error.status & ERR_CLKMON){
			//The clock cycles are not correct
      printf("\n\n ERR_CLKMON \n\n");

		}
		if(error.status & ERR_ADDMON){
			//set high when an address is incorrect for the last operation
      printf("\n\n ERR_ADDMON \n\n");

		}

	//This command returns 0 on successful clear
	//otherwise, this command can handle it later
	error.status=read(REG_CLEAR_ERROR) ;

	//If the error is still there, reset the AS5050 to attempt to fix it
	#if AS5050_RESET_ON_ERRORS==1
	//if(error.status)write(REG_SOFTWARE_RESET,DATA_SWRESET_SPI);
	if(error.status)write(REG_MASTER_RESET,0x0);
	#endif
	}

	return error.status;
};
