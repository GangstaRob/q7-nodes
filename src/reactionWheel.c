/*************************************************
* Author: Brian Adams
* Date: 3/23/2018
* Revised: 3/30/2018
* Description: Driver that sends commands via i2c. 
* i2cRW.c and slip.c are needed for this to work.
* The RW command is stored into an array and the crc 
* is calculated and added for the last two bytes. 
* The array is then sent to be slip encoded. 
* The RW will decode the slip message, check the 
* crc, and do the command. When reading the 
* message received must be slip decoded.  
**************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "reactionWheel.h"	//contains function declarations and union RW_value_t
#include "slip.c"		//messages must be within a slip frame
#include "i2cRW.c"		//includes read and write commands using smbus commands

char RW_ADDR=0x33;	
char NSP_SOURCE_ADDR=0x11;
char RW_MSG_CONTROL__POLL=0x80;	// 0b10000000
char RW_MSG_CONTROL__B = 0x40;	// 0b01000000
char RW_MSG_CONTROL__A = 0x20;	// 0b00100000

enum RW_telecommands
{
    RW_PING = 0x00,
    RW_INIT = 0x01,
    RW_PEEK = 0x02,
    RW_POKE = 0x03,
    RW_TELEMETRY = 0x04,
    RW_CRC_TEST = 0x06,
    RW_APP_TELEMETRY = 0x07,
    RW_APP_CMD = 0x08
};

enum RW_mode
{
    RW_IDLE = 0x00,
    RW_SPEED = 0x05,
    RW_MOMENTUM = 0x15,
    RW_TORQUE = 0x16,
};


static int RW_ping()
{
    uint8_t cmd[10];
    uint16_t crc;
    int i = 0;

    cmd[i++] = RW_ADDR;
    cmd[i++] = NSP_SOURCE_ADDR;
    cmd[i++] = RW_MSG_CONTROL__POLL | RW_PING;
    cmd[i++] = 0x00;
    cmd[i++] = 0x10;
    cmd[i++] = 0x00;
    cmd[i++] = 0x00;
    crc = NSP_calc_crc(cmd, i);
    cmd[i++] = crc & 0xFF;
    cmd[i++] = crc >> 8;
   
    SLIPenc(cmd,i);
    i2cWrite();

    return 0;
}

int ProgramStarted=0;	//Status


static void RW_telecommand_init()
{
    uint8_t cmd[10];
    uint16_t crc;
    int i = 0;
    if(ProgramStarted==0){
    cmd[i++] = RW_ADDR;
    cmd[i++] = NSP_SOURCE_ADDR;
    cmd[i++] = RW_INIT;
    cmd[i++] = 0x00;
    cmd[i++] = 0x10;
    cmd[i++] = 0x00;
    cmd[i++] = 0x00;
    crc = NSP_calc_crc(cmd, i);
    cmd[i++] = crc & 0xFF;
    cmd[i++] = crc >> 8;
    
    SLIPenc(cmd,i);
    i2cWrite();

    ProgramStarted=1;
    printf("Program Started. \n");
    }else{
	cmd[i++] = RW_ADDR;
    	cmd[i++] = NSP_SOURCE_ADDR;
    	cmd[i++] = RW_INIT;
    	crc = NSP_calc_crc(cmd, i);
    	cmd[i++] = crc & 0xFF;
    	cmd[i++] = crc >> 8;
    
    	SLIPenc(cmd,i);
    	i2cWrite();
 	ProgramStarted=0;
        printf("Program Stopped. \n");
    }
}


static void _self_test(void)
{
    uint8_t cmd[16];
    uint16_t crc;
    int i = 0;

    cmd[i++] = RW_ADDR;
    cmd[i++] = NSP_SOURCE_ADDR;
    cmd[i++] = RW_APP_CMD;
    cmd[i++] = 0x00;
    cmd[i++] = 0x18;
    cmd[i++] = 0x00;
    cmd[i++] = 0x00;
    cmd[i++] = 0x00;
    cmd[i++] = 0x00;
    crc = NSP_calc_crc(cmd, i);
    cmd[i++] = crc & 0xFF;
    cmd[i++] = crc >> 8;

    SLIPenc(cmd,i);
    i2cWrite();
    
}

void _set_speed(RW_value_t speed){
    uint8_t cmd[16];
    uint16_t crc;
    int i = 0;

    cmd[i++] = RW_ADDR;
    cmd[i++] = NSP_SOURCE_ADDR;
    cmd[i++] = RW_APP_CMD;
    cmd[i++] = 0x00;
    cmd[i++] = RW_SPEED;
    cmd[i++] = speed.byte[0];
    cmd[i++] = speed.byte[1];
    cmd[i++] = speed.byte[2];
    cmd[i++] = speed.byte[3];
    crc = NSP_calc_crc(cmd, i);
    cmd[i++] = crc & 0xFF;
    cmd[i++] = crc >> 8;

    SLIPenc(cmd,i);
    i2cWrite();
}


void _get_speed(RW_value_t* speed){
    uint8_t cmd[16];
    uint16_t crc;
    int i=0;
    Redo:i=0;

/* Reset Array */
    int j=0;
    for (j = 0; j < 16; j++) // Using for loop we are initializing
    {
    	NewCmd[j] = 0;
    }
/***************/

    cmd[i++] = RW_ADDR;
    cmd[i++] = NSP_SOURCE_ADDR;
    cmd[i++] = RW_MSG_CONTROL__POLL | RW_APP_TELEMETRY;
    cmd[i++] = RW_SPEED;
    crc = NSP_calc_crc(cmd, i);
    cmd[i++] = crc & 0xFF;
    cmd[i++] = crc >> 8;

    SLIPenc(cmd,i);
    i2cWrite();
    i2cRead();
  
    SLIPdec(NewCmd,Numel(NewCmd));

    RW_value_t x;
    x.byte[0] = NewCmd[2];
    x.byte[1] = NewCmd[3];
    x.byte[2] = NewCmd[4];
    x.byte[3] = NewCmd[5];
	
	//printf("Speed Bytes: %x %x %x %x\n",x.byte[0],x.byte[1],x.byte[2],x.byte[3]);
    if(x.value > 681 | x.value < -681){	//gather a new value if not within the maximum range
	printf("speed: %p\n",speed);
	
	goto Redo;
    }else{				//store values within range
    	speed->byte[0] = NewCmd[2];
    	speed->byte[1] = NewCmd[3];
    	speed->byte[2] = NewCmd[4];
    	speed->byte[3] = NewCmd[5];
    }
   
    
}

void _get_torque(RW_value_t* torque){
    uint8_t cmd[16];
    uint16_t crc;
    int i=0;
    //Redo:i=0;

/* Reset Array */
    int j=0;
    for (j = 0; j < 16; j++) // reseting array
    {
    	NewCmd[j] = 0;
    }
/***************/

    cmd[i++] = RW_ADDR;
    cmd[i++] = NSP_SOURCE_ADDR;
    cmd[i++] = RW_MSG_CONTROL__POLL | RW_APP_TELEMETRY;
    cmd[i++] = RW_TORQUE;
    crc = NSP_calc_crc(cmd, i);
    cmd[i++] = crc & 0xFF;
    cmd[i++] = crc >> 8;

    SLIPenc(cmd,i);
    i2cWrite();
    i2cRead();
  
    SLIPdec(NewCmd,Numel(NewCmd));
/*
    RW_value_t x;
    x.byte[0] = NewCmd[2];
    x.byte[1] = NewCmd[3];
    x.byte[2] = NewCmd[4];
    x.byte[3] = NewCmd[5];
*/	

	torque->byte[0] = NewCmd[2];
    	torque->byte[1] = NewCmd[3];
    	torque->byte[2] = NewCmd[4];
    	torque->byte[3] = NewCmd[5];
    
   
    
}
void _set_torque(RW_value_t torque){
    uint8_t cmd[16];
    uint16_t crc;
    int i = 0;

    cmd[i++] = RW_ADDR;
    cmd[i++] = NSP_SOURCE_ADDR;
    cmd[i++] = RW_APP_CMD;
    cmd[i++] = 0x00;
    cmd[i++] = RW_TORQUE;
    cmd[i++] = torque.byte[0];
    cmd[i++] = torque.byte[1];
    cmd[i++] = torque.byte[2];
    cmd[i++] = torque.byte[3];
    crc = NSP_calc_crc(cmd, i);
    cmd[i++] = crc & 0xFF;
    cmd[i++] = crc >> 8;

    SLIPenc(cmd,i);
    i2cWrite();
}


static void _stop_wheel(){
    RW_value_t speed;
    speed.value = 0.0;
    _set_speed(speed);
}


uint16_t NSP_calc_crc(uint8_t * bufp, unsigned int len)
{
#define POLY 0x8408 /* bits reversed for LSB-first */
    uint16_t crc = 0xffff;
    int i;

    while (len-- > 0) {
        uint8_t ch = *bufp++;
        for (i = 0; i < 8; i++) {
            crc = (crc >> 1) ^ ( ((ch ^ crc) & 0x01) ? POLY : 0 );
            ch >>= 1;
        }
    }

    return crc;

}


