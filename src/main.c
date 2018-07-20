/*************************************************
* Author: Brian Adams
* Date: 3/23/2018
* Revised: 3/30/2018
* Description: This program tests Reaction Wheel (RW) 
* commands. Before wheel commands are executed, 
* an init(1) command must be sent.   
**************************************************/

#include "reactionWheel.c"

void ShellUI();				//Text GUI for testing RW functions
void ShellClr();			//Clear shell window
void speedConv(float speed);		//Converts from rad/s to Hz, degrees/sec, and rpm
void torqueConv(float torque);		//Converts torque from Nm to lb*ft
void detectAddr();			//Runs i2c-tools i2cdetect to find available addresses
void wait(int microsec);		//waits x microseconds before next task
void exitSeq();				//Text Gui for exiting program

char ShellCmd;				//Comands 1-9

static RW_value_t speed;
//static RW_value_t previous_speed;
static RW_value_t torque;

int main(int argc, char *argv[]){
int i=0;
speed.value=0.0;
torque.value=0.0;

ShellClr();

    while(i<1){
	
	ShellUI();
	switch(ShellCmd)
	{
		
		case '1':
			ShellClr();
			RW_telecommand_init();
			printf("Init initiated.\n\n");
			break;
		case '2':
			ShellClr();
			RW_ping();
			printf("Ping initiated.\n\n");
			break;
		case '3':	
 			ShellClr();
			_self_test();
			printf("Self Test initiated.\n\n");
			break;
		case '4': 
			ShellClr();
			printf("Current speed: %.2f rad/s\n",speed.value);//use i2c read function
			printf("Range: +/-680 \nEnter Speed(rad/s): ");
			scanf("%f",&(speed.value));
			speedConv(speed.value);
			
			_set_speed(speed);
			/*
			while(speed.value!=sp){
				printf("Current speed: %.2f rad/s\n",speed.value);//use i2c read function
				_get_speed(&speed);
			}*/
			printf("Speed Set. \n\n");
			break;
		case '5': 
			ShellClr();
			_get_speed(&speed);
			printf("Current speed: %.2f rad/s\n",speed.value);//use i2c read function
			speedConv(speed.value);
			
			printf("Speed Captured. \n\n");
			break;

		case '6': 
			ShellClr();
			/**/_get_torque(&torque);
			printf("Current Torque: %.2f N*m\n",torque.value);//use i2c read function
			printf("Enter Torque(N*m): ");
			scanf("%f",&(torque.value));
			torqueConv(torque.value);
			
			_set_torque(torque);

			printf("Torque Set. \n\n");
			break;
		case '7':
			ShellClr();
			detectAddr();
			break;
		case '8':
			ShellClr();
			_stop_wheel();
			/**************************/
			if(speed.value!=0){
				wait(3000000);
			}
			/**************************/
			printf("Reaction Wheel stopped.\n\n");
			break;
		case '9':
			ShellClr();
			_stop_wheel();
			exitSeq();
			printf("Reaction Wheel stopped. \n\n");
			RW_telecommand_init();
			i++;
			break;
		default:
			ShellClr();
			printf("The value entered is not valid.\n\n");
			break;
	}

    }
}


void speedConv(float speed){
	float x;
	x=speed*.159155;
	printf("%.2f Hz\n",x);
	x=speed* 57.29578;
	printf("%.2f deg/sec\n",x);
	x=speed* 9.5493 ;
	printf("%.2f rpm\n\n",x);
}

void torqueConv(float torque){
	float x;
	x=torque*1.3558179;
	printf("%.2f lb*ft \n\n",x);
}

void ShellUI(){
	
	printf("Enter reaction wheel command: \n"
	"1. Init \n2. Ping \n3. Self Test \n4. Set Speed \n"
	"5. Get Speed\n6. Set Torque \n7. Available I2c Addresses"
	"\n8. Stop\n9. Exit\n\n");
	scanf(" %c",&ShellCmd);
}

void wait(int microsec){					//microseconds 1s = 10^-6 us
	char cmd[25];
	snprintf(cmd,24,"usleep %i",microsec);	
  	int rv = system(cmd);
}

void ShellClr(){
	char cmd[25];
	snprintf(cmd,24,"clear");	
  	int rv = system(cmd);
}

void detectAddr(){					//detect i2c devices on bus 1
	char cmd[25];
	printf("Available addresses: \n");
	snprintf(cmd,24,"i2cdetect -y -r 1");		//Use i2c bus 1
  	int rv = system(cmd);
	printf("\n\n");
}


void exitSeq(int seq){
	if(seq==1){
		int count=0;
		while(count<10){
			ShellClr();
			printf("Exiting Program... / \n\n");
			wait(100000);// wait .5 second
			ShellClr();
			printf("Exiting Program... - \n\n");
			wait(100000);// wait .5 second
			ShellClr();
			printf("Exiting Program... \\ \n\n");
			wait(100000);// wait .5 second
			ShellClr();
			printf("Exiting Program... | \n\n");
			wait(100000);// wait .5 second
			count++;
		}
	}else if(seq==2){
		int count=0;
		int t=1;
		float previous_speed=0;
		if(speed.value != 0){
			do{
		    		ShellClr();
		   		_get_speed(&speed);
	    	   		previous_speed=speed.value;
		   		printf("Current speed: %.2f rad/s |\n",previous_speed);		    
		  		ShellClr();
		  		_get_speed(&speed);
		    		if(speed.value == 0 && previous_speed == 0){
		    			t=0;
		    		}
		    		printf("Current speed: %.2f rad/s -\n",speed.value);
				    
			}while(t==1);
	
		}
		
	}else{
		int count=0;
		while(count<10){
			ShellClr();
			printf("Exiting Program... / \n\n");
			wait(100000);// wait .1 second
			ShellClr();
			printf("Exiting Program... - \n\n");
			wait(100000);// wait .1 second
			ShellClr();
			printf("Exiting Program... \\ \n\n");
			wait(100000);// wait .1 second
			ShellClr();
			printf("Exiting Program... | \n\n");
			wait(100000);// wait .1 second
			count++;
		}
	}

}



	




