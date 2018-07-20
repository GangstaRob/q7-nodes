#include "reactionWheel.c"

int main(int argc, char *argv[]){

/******************************/

RW_ping();

/******************************

uint8_t s[7]={0x01,0xc0,0xff,0xaa,0xdb,0xdd,0xcc};

SLIPenc(s,7);

int i;
for(i=0;i<Numel(NewCmd);i++){
	printf(" %.2p ",NewCmd[i]);
}
printf("\nj=%i\n",Byte);

	
/*******************************

RW_ping();
printf("\nNewCmd2: \n");
for(i=0;i<10;i++){
	printf(" %p ",NewCmd[i]);
}
printf("\nJ=%i\n",j);

uint8_t t[11]={0x01,0xdb,0xdc,0xff,0xdb,0xdd,0xdb,0xdd,0xdd,0xc0,0xc0};
SLIPdec(t,11);

*********************************/
}
