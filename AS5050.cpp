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



#include "Arduino.h"
#include "AS5050.h"

#ifndef _SPI_H_INCLUDED
#warning +============================================+
#warning | Please include SPI.h from the main sketch. |
#warning | This issue is a limitation of the Arduino  |
#warning | libraries and cannot be resolved here.     |
#warning +============================================+
#endif

#include "SPI.h"


AS5050::AS5050(byte pin, byte spi_speed){
  /*CONSTRUCTOR
  * Sets up the required values for the function, and configures the 
  * hardware SPI to operate correctly
  */
  _pin=pin;

  //Prepare the SPI interface
  pinMode(_pin,OUTPUT); 
  SPI.setClockDivider(spi_speed);       //this board supports speedy! :D
  SPI.setBitOrder(MSBFIRST);            //Match the expected bit order
  SPI.setDataMode(SPI_MODE1);           //falling edge (CPOL 1), low idle (CPHA 0);
  //pull pin mode low to assert slave
  //digitalWrite(_pin,LOW);

  //Reboot the chip, in case there were issues during various transmissions
  write(REG_MASTER_RESET,0x0);
  
  
  //Read angle twice to initialize chip and get to a known good state
  //Reading once won't work, as _last_angle will be set incorrectly
  angle();
  _init_angle=angle();
  //angle() will glitch on startup if it's >768, reset it
  rotations=0;
  
  //Set to True to reverse the angle returned by the library
  mirror=false;
};

unsigned int AS5050::send(unsigned int reg_a){
  spi_data response,reg;
  reg.value=reg_a;
  //This function does not take care of parity stuff,
  //due to peculiarities with it.
  
  SPI.begin();
  digitalWrite(_pin,LOW);  //Start Transaction
  
  //Send data in MSB order
  response.bytes.msb=SPI.transfer(reg.bytes.msb);
  response.bytes.lsb=SPI.transfer(reg.bytes.lsb);

  digitalWrite(_pin,HIGH);	//End Transaction
  SPI.end();
  
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
  //delayMicroseconds(1);       //hold time between transactions:50ns
  reg=send(REG_NOP); //receive response from chip
  
  //Save the parity error for analysis
  error.parity=__builtin_parity(reg&(~RES_PARITY)) != (reg&RES_PARITY);
  error.transaction|=error.parity;
  error.transaction|=reg&RES_ERROR_FLAG;

  return reg; //remove error and parity bits  
}


//FIXME: Make the Write return and verify the response.
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
  error.transaction=data & (RES_ERROR_FLAG); //save error data
  error.parity=__builtin_parity(data&(~RES_PARITY)) != (data&RES_PARITY);
  error.transaction|=error.parity;
  error.transaction|=data&RES_ERROR_FLAG;

 
  return data;      //remove parity and EF bits and return data. 
};  

int AS5050::raw_angle(){
	//This function strips out the error and parity 
	//data in the data frame, and handles the errors
	unsigned int data;
	int angle;
	
	data=read(REG_ANGLE);
	/* Response from chip is this:
	14 | 13 | 12 ... 2                       | 1  | 0
	AH | AL |  <data 10bit >                 | EF | PAR
	*/

	//Parity data is generated in read() function
	
	//TODO this needs some work to avoid magic numbers
	angle=(data&0x3FFE)>>2; //strip away alarm bits, then parity and error flags
	
	//Allow for the user to mirror the rotation, in case the angle seems backwards
	if(mirror)angle=AS5050_ANGULAR_RESOLUTION-angle;
	
	
	#if AS5050_AUTO_ERROR_HANDLING
	switch(data&(RES_ALARM_HIGH|RES_ALARM_LOW)){
		case RES_ALARM_HIGH: //gain too high, decrease gain
			gain=read(REG_GAIN_CONTROL);		//get information about current gain
			write(REG_GAIN_CONTROL,--gain); 	//increment gain and send it back
		break;
		case RES_ALARM_LOW: //gain too low, increase gain
			gain=read(REG_GAIN_CONTROL); 		//get information about current gain
			write(REG_GAIN_CONTROL,++gain); 	//increment gain and send it back
		break;
			default:handleErrors();
		break;
	}
	#endif		
		
	return angle;
}


int AS5050::angle(){
	unsigned int anglesum=0;
	int angle;
	int last_sample=_last_angle;
	int _rotations=rotations; //work with a local copy to make function re-entrant when called from interrupts
	
	
	/*BUG : near the zerog crossing, one of the samples will jump to 1024 
	The resulting data looks like this:
	1024
	1024
	5
	1020
	which obviously has a REALLY bad average of 768. Need to scan for 
	differences of 512 or more and then promote or demote the value, or 
	take care of rollovers within each scan
	*/
	//TODO until the above bug is fixed, remove loop
	for(byte i=0;i<NUM_ANGLE_SAMPLES; i++){
	
		angle=raw_angle();
		
		//We need to make sure that there's no parity errors, or else our angle will be corrupted.
		if( error.parity ){
			//undo the latest read, and use the last known good data
			angle=last_sample;
		}
		
		//keep up our running sum
		anglesum+=angle;


		//track rollovers for continous angle monitoring
		if(last_sample>768 && angle<=256)_rotations+=1;
		else if(last_sample<256 && angle>=768)_rotations-=1;
		
		//preserve the good data for future possible errors
		last_sample=angle;

		
		//If we did encounter errors, we need to correct them before the next sample.
		if(error.transaction){	handleErrors();	}

		//TODO make sure that this can run continuously without exceeding chip hold times
		//delayMicroseconds(10);
	}

	//average samples and find true angle
	angle=(anglesum+NUM_ANGLE_SAMPLES/2)/NUM_ANGLE_SAMPLES; //the anglesum+numsamples/2 performs fair rounding

	//Update the globals
	rotations=_rotations;
	_last_angle=angle;
	return angle;
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
    return angle()+rotations*1024 ;
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
	error.status=read(REG_ERROR_STATUS);
	
	//If we don't have any standing errors, then quickly bypass all the checks
	if(error.status){
		if(REG_ERROR_STATUS & ERR_PARITY){
			//set high if the parity is wrong
			//Avoid doing something insane and assume we'll come back to 
			//this function and try again with correct data
			return error.status;
		}
		
		/*
		* Gain problems, automatically adjust
		*/
		if(REG_ERROR_STATUS & ERR_DSPAHI){										int gain=read(REG_GAIN_CONTROL);	//get information about current gain
			write(REG_GAIN_CONTROL,--gain); 	//increment gain and send it back
		}
		else if(REG_ERROR_STATUS & ERR_DSPALO){
			int gain=read(REG_GAIN_CONTROL); 	//get information about current gain
			write(REG_GAIN_CONTROL,++gain); 	//increment gain and send it back
		}
		
		/*
		* Chip Failures, can be fixed with a reset
		*/
		if(REG_ERROR_STATUS & ERR_WOW){
			//After a read, this gets set low. If it's high, there's an internal
			//deadlock, and the chip must be reset
			write(REG_SOFTWARE_RESET,DATA_SWRESET_SPI);
		}
		if(REG_ERROR_STATUS & ERR_DSPOV){
			//CORDIC overflow, meaning input signals are too large. 
			//Gain adjustments should take care of this
			write(REG_SOFTWARE_RESET,DATA_SWRESET_SPI);
		}
			
		/*
		* Hardware issues. These need to warn the user somehow
		*/
		//TODO figure out some sane warning! This is not a good thing to have happen
		if(REG_ERROR_STATUS & ERR_DACOV){
			//This indicates a Hall effect sensor is being saturated by too large of 
			//a magnetic field. This usually indicates a hardware failure such as a magnet 
			//being displaced
		}
		if(REG_ERROR_STATUS & ERR_RANERR){
			//Accuracy is decreasing due to increased tempurature affecting internal current source 
		}
			
		/*
		* Reasonably harmless errors that can be fixed without reset
		*/
		if(REG_ERROR_STATUS & ERR_MODE){
			//set high if the chip is measuring an angle, otherwise low
		}
		if(REG_ERROR_STATUS & ERR_CLKMON){
			//The clock cycles are not correct			
		}
		if(REG_ERROR_STATUS & ERR_ADDMON){
			//set high when an address is incorrect for the last operation
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

