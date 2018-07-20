/*******************************************
* Author: Brian Adams
* Date: 3/19/2018
* Revised: 4/3/2018
* Description: Driver that allows i2c read 
* and write commands for the reaction wheel.
*
* Note: i2c-1 core may be disabled on start.
* Enter the following command in the terminal
* to enable the clock:
*      devmem2 0xf800012c w 0xfc0c48
********************************************/

#include "i2c-dev.h"	//i2c-tools version
#include <fcntl.h>
#include <errno.h>
#include <stdlib.h>			//for system();
#include <stdio.h>

void enableI2cBus1Core();	//used to enable the i2c-1 core
int i2cInit();			//File should be opened once; Slave Address is configured once.
int i2cClose();			//Closes open file
int i2cWrite();			//Writes byte data
int i2cRead();			//Reads byte data
	
static int file;
int coreEnabled=0;		//core enable status

void enableI2cBus1Core(){
	if(coreEnabled==0){
		char cmd[50];
		snprintf(cmd,49,"devmem2 0xf800012c w 0xfc0c48");		
		int rv = system(cmd);
		coreEnabled=1;
	}
}

int i2cInit(){
	enableI2cBus1Core();	//enables the core once before command in sent.

	char RW_ADDR=0x33;	//The default RW address is 0x33
		file = open("/dev/i2c-1",O_RDWR);
		if(file<0){						
    			printf("Error opening file: %s\n", strerror(errno));
    			printf("file = %d\n",file);
    			return 0;
  		}
		if (ioctl(file, I2C_SLAVE, RW_ADDR) < 0){			
    			printf("ioctl error: %s\n", strerror(errno));		
    			printf("The address entered was %p.\n", RW_ADDR);
    			return 0;
		}
}


int i2cClose(){
	close(file);
}


int i2cWrite(){
	i2cInit();
	if (write(file, NewCmd, Byte) != Byte) {		
		printf("i2c communication write failed. Byte size=%i\n",Byte);
		i2cClose();
		return 0;
	}else{
		printf("i2c write successful.\n");	
	}	
	i2cClose();
}


int i2cRead(){
	i2cInit();
	if (read(file, NewCmd, Byte) != Byte){
		printf("Error Handling: %s\n", strerror(errno));
		i2cClose();
		return 0;
	}else{
		printf("i2c read successful.\n");
	}
	i2cClose();
}
