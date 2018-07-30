/*******************************************
* Author: Brian Adams, Robinson Czajkowski
* Date: 7/30/18
* Revised: 
* Description: 
********************************************/

#include <linux/spi/spidev.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>

#define Numel(x) sizeof(x)/sizeof(x[0])
static int file;
static const char *device = "/dev/spidev1.0";
static __u8 mode = 3;
static __u8 bits = 8;
static __u32 speed = 15*1000*1000;
static __u16 delay;
static __u8 tx[4] = {0x00, 0x00, 0x00, 0x00};

void enableSPImodule();
int spiInit();
int spiReadWrite();
int spiClose();
static int setTX(__u8 newtx[]);
static int transfer();
static void acc(__u8 * bytes);

void enableSPImodule(){
        char cmd[50];
	snprintf(cmd,49,"modprobe spi-cadence");
	int rv = system(cmd);
}


int spiInit(){ //Opens file and prints default values
	enableSPImodule();
	file = open(device,O_RDWR);
	if(file<0){								
    		printf("Error opening file: %s\n", strerror(errno));
    		printf("file = %d\n",file);
    		return 0;
	}
	
	//printf("spi device: %s\n",device);
	//printf("spi mode: %d\n", mode);
	//printf("bits per word: %d\n", bits);
	//printf("max speed: %d Hz (%d kHz)\n\n", speed, speed/1000);
	//printf("File opened.\n");

}


int spiReadWrite(__u8 newtx[]){
	int ret=0;

	setTX(newtx);
	spiInit();
	
	ret = ioctl(file, SPI_IOC_WR_MODE, &mode);	
	if(ret==-1){
		printf("SPI Write setup failed.\n");
		spiClose(); 
		return 0;
	}	
	
	ret =ioctl(file,SPI_IOC_WR_BITS_PER_WORD,&bits);
	if(ret==-1){
		printf("SPI Write failed.\n");
		spiClose(); 
		return 0;
	}

    ret = ioctl(file, SPI_IOC_RD_MODE, &mode);
    if(ret==-1){
        printf("SPI Read setup failed.\n");
        spiClose();
        return 0;
    }

    ret =ioctl(file,SPI_IOC_RD_BITS_PER_WORD,&bits);
    if(ret==-1){
        printf("SPI Read failed.\n");
        spiClose();
        return 0;
    }

	//printf("SPI Write.\n");
	transfer();
	spiClose();
}

static int setTX(__u8 newtx[]){
    tx[0] = newtx[0];
    tx[1] = newtx[1];
    tx[2] = newtx[2];
    tx[3] = newtx[3];
	int i=0;
	printf("tx: ");
	for(i;i<Numel(tx);i++){
		printf("%.2p ",tx[i]);
	}
	printf("\n");
	return *tx;
}

static int transfer(){
	int ret;
	
	__u8 rx[Numel(tx)]={0};

	struct spi_ioc_transfer tr= {
		.tx_buf=(unsigned long)tx,
		.rx_buf=(unsigned long)rx,
		.len = Numel(tx),
		.delay_usecs= delay,
		.speed_hz = speed,
		.bits_per_word=bits
	};
	ret = ioctl(file, SPI_IOC_MESSAGE(1), &tr);
	if(ret<1){
		printf("Message send failed. \n\n");
		return 0;
	}
	

	int i=0;
	printf("rx: ");
	for(i;i<Numel(rx);i++){
		printf("%.2p ",rx[i]);

	}
	printf("\n");
	acc(rx);
	return ret;
}

int spiClose(){
	close(file);
	//printf("File closed.\n\n");

}

typedef union{
	float value;
	__u8 bits[sizeof(float)];
}x;

static void acc(__u8 * bytes){
/*
	x lsb;
	lsb.bits[0] = bytes[0]*.8/1000;
	lsb.bits[1] = bytes[1]*.8/1000;
	printf("value: %.3f\n",&lsb.value);
*/
	float value=0;
	//printf("Bytes: %.2p %.2p\n",bytes[0],bytes[1]);
	//bytes[0]=~bytes[0];
	//bytes[1]=~bytes[1];
	//printf("~Bytes: %.2p %.2p\n",bytes[0],bytes[1]);
	//bytes[0]=bytes[0]+0x01;
	//bytes[1]=bytes[1]+0x01;
	//printf("~Bytes+1: %.2p %.2p\n",bytes[0],bytes[1]);
	value=bytes[0] +bytes[1];
	printf("Value: %f\n",value);
	//value=bytes[0]*(float)(.8/1000) +bytes[1]*(float)(.8/1000);
	//printf("Value g: %.3f\n",value);
	//return value;


}
