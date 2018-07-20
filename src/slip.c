/*******************************************
* Author: Brian Adams
* Date: 3/19/2018
* Revised: 3/30/2018
* Description: Encodes and Decodes Slip bytes
* and returns the array of bytes.
********************************************/

#define Numel(x) (sizeof(x)/sizeof(uint8_t))	//Used to calculate size of array

void* SLIPenc(uint8_t cmd[],int elems);		//Encodes bytes into slip frame
void* SLIPdec(uint8_t cmd[],int elems);		//Decodes bytes from slip frame
int printSlip(int x);				//prints decoded or encoded slip bytes
						//1=encode, 0 = decode, No value = no printing
/* Slip framming commands */
static const uint8_t FEND = 0xC0;		//represents start and end byte
static const uint8_t FESC = 0xDB;
static const uint8_t TFEND = 0xDC;
static const uint8_t TFESC = 0xDD;

static int Byte=0;			//Holds number of bytes processed
static uint8_t NewCmd[16]={0};		//Holds encoded and decoded slip bytes into array


void* SLIPenc(uint8_t cmd[],int elems){
	int i=1;			//default = 1; first byte is not needed for i2c implementation
	int j=0;
	for(i;i<elems;i++){
		if(cmd[i]==FEND){
			NewCmd[j++]=FESC;
			NewCmd[j++]=TFEND;
		}else if(cmd[i]==FESC){
			NewCmd[j++]=FESC;
			NewCmd[j++]=TFESC;
		}else{
			NewCmd[j++]=cmd[i];
		}
	}//end for loop
	NewCmd[j++]=FEND;
	Byte=j;
	// debug ***********
	printSlip(1);			//enter 1 to print encoded slip
	/******************/
	return NewCmd;
}


void* SLIPdec(uint8_t cmd[],int elems){
	int i=0;
	int j=0;
	int e=0;			//used to flag error status. 1=True, 0=False
	for(i;i<elems;i++){
		if(i>0){
			if(cmd[i-1]==FESC){
				if(cmd[i]==TFEND){
					NewCmd[j++]=FEND;
					i++;
				}else if(cmd[i]==TFESC){
					NewCmd[j++]=FESC;
					i++;
				}else{
					//printf("\nError: FESC must be followed by TFESC or TFEND.\n");
					e=1;	// flag error
					break;
				}
			}else if(cmd[i-1]==FEND){
				
			}else{
				NewCmd[j++]=cmd[i-1];
			}
		}
	}//end for loop
	Byte=j;
	// debug ***********
	if (e==0){
		printSlip(0);		//enter 0 to print decoded slip
	}
	/******************/
	return NewCmd;	
}


int printSlip(int x){
	int i=0;
	if(x==1){
			
		printf("Number of Bytes encoded: %i\n",Byte);
		for(i=0;i<Byte;i++){
			printf(" %.2p ",NewCmd[i]);
		}
		printf("\n\n");
	}else if(x==0){
		
		printf("Number of Bytes decoded: %i\n",Byte);
	
		for(i=0;i<Byte;i++){
			printf(" %.2p ",NewCmd[i]);
		}
		printf("\n\n");
		
	}else{
		return 0;		
	}
	
	
}
