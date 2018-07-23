/*Allows the computer to connect and control the q7*/
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <iostream>
#include <fstream>
using namespace std;

string response = "02 in  on  |\n04 in  on  |\n06 in  on  |\n07 in  on  |\n08 in  on  |\n09 in  on  |\n10 in  on  |\n12 in  on  |\n13 in  on  |\n14 in  on  |\n16 in  on  |\n18 in  on  |\n20 in  on  |\n23 in  on  |\n25 in  on  |\n30 in  on  |\n32 in  on  |";
int numOfPins = 22;
string pins[22] = {"00", "01", "02", "03", "04", "05", "06", "07", "08", "09", "10", "11", "12", "13", "14", "15", "16", "17", "18", "19", "20", "21"};

void readCommand(char[]);
string getState();
void setIn(char[]);
void setOut(char[]);
void setOn(char[]);
void setOff(char[]);
void doStuff(int);
string readValue(char[]);
string readDirection(char[]);
void resetPins();
void loadPins(char[]);
void error(const char *msg) {
    perror(msg);
    exit(0);
}


int main(int argc, char **argv) {
	
    ros::init(argc, argv, "hello_ros");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("hello, ROS!");

    int sockfd, portno, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    char buffer[256];
    
	cout << "Assuming port is 3440\n";
	portno = 3440;
    
    while(1) {
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");
    server = gethostbyname("138.115.225.244");
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
        error("ERROR connecting");

    printf("Prompting server for a command\n");
    doStuff(sockfd);
    close(sockfd);
	
    }
    return 0;

}

void doStuff(int sock) {
    int n;
    char buffer[256];
    string message;

    bzero(buffer,256);
    message = response;
    strcpy(buffer, message.c_str());
    n = write(sock,buffer,strlen(buffer));
    if (n < 0) 
         error("ERROR writing to socket\n");
    bzero(buffer,256);
    n = read(sock,buffer,255);
    if (n < 0) 
         error("ERROR reading from socket\n");
    //printf("Command: %s\n",buffer);
    readCommand(buffer);
}

void readCommand(char command[]) {
    char pin[2];
    pin[0] = command[1];
    pin[1] = command[2];
    if(command[0] == 'g') {
	printf("Reading all pins\n");
	response = getState();
    } else if(command[0] == 'i') {
	cout << "Setting pin " << pin[0] << pin[1] << " to in \n";
	setIn(pin);
        response = getState();
    } else if(command[0] == 'o') {
	cout << "Setting pin " << pin[0] << pin[1] << " to out \n";
	setOut(pin);
        response = getState();
    } else if(command[0] == 'n') {
	cout << "Setting pin " << pin[0] << pin[1] << " to on \n";
	setOn(pin);
        response = getState();
    } else if(command[0] == 'f') {
	cout << "Setting pin " << pin[0] << pin[1] << " to off \n";
	setOff(pin);
        response = getState();
    } else if(command[0] == 'v') {
	//cout << "Reading value of pin " << pin[0] << pin[1] << "\n";
	response = readValue(pin);
    } else if(command[0] == 'd') {
	//cout << "Reading direction of pin " << pin[0] << pin[1] << "\n";
	response = readDirection(pin);
    } else if(command[0] == 'r') {
	cout << "Resetting all pins\n";
	resetPins();
        response = getState();
    } else if(command[0] == 'l') {
	cout << "Loading State\n";
	loadPins(command);
        response = getState();
    } else {
	printf("Could not understand command\n");
	response = "q7: Could not understand command";
    }
}

string getState() {
    char pin[2];
    string chart;
    chart = "";
    for(int i = 0; i < numOfPins; i++) {
	strcpy(pin, pins[i].c_str());
	chart += pins[i];
	chart += " ";
	chart += readDirection(pin);
	chart += " ";
	chart += readValue(pin);
	chart += "\n";
    }
    return chart;
}

void setOut(char pin[]) {
    ofstream myFile;
    string directory;
    if(pin[0] == '0') {
	directory = "/dev/gpios/q7_3v3/q7_gpio_3v3_" + string(1,pin[1]) + "/direction";
    } else {
	directory = "/dev/gpios/q7_3v3/q7_gpio_3v3_" + string(1,pin[0]) + string(1,pin[1]) + "/direction";
    }
    myFile.open(directory.c_str());
    myFile << "out";
    myFile.close();
    //cout << "The direction of pin " << pin[0] << pin[1] << " was set to out\n";
}

void setIn(char pin[]) {
    ofstream myFile;
    string directory;
    if(pin[0] == '0') {
	directory = "/dev/gpios/q7_3v3/q7_gpio_3v3_" + string(1,pin[1]) + "/direction";
    } else {
	directory = "/dev/gpios/q7_3v3/q7_gpio_3v3_" + string(1,pin[0]) + string(1,pin[1]) + "/direction";
    }
    myFile.open(directory.c_str());
    myFile << "in";
    myFile.close();
    //cout << "The direction of pin " << pin[0] << pin[1] << " was set to in\n";
}

void setOn(char pin[]) {
    ofstream myFile;
    string directory;
    if(pin[0] == '0') {
	directory = "/dev/gpios/q7_3v3/q7_gpio_3v3_" + string(1,pin[1]) + "/value";
    } else {
	directory = "/dev/gpios/q7_3v3/q7_gpio_3v3_" + string(1,pin[0]) + string(1,pin[1]) + "/value";
    }
    myFile.open(directory.c_str());
    myFile << 1;
    myFile.close();
    //cout << "The value of pin " << pin[0] << pin[1] << " was set to on\n";
}

void setOff(char pin[]) {
    ofstream myFile;
    string directory;
    if(pin[0] == '0') {
	directory = "/dev/gpios/q7_3v3/q7_gpio_3v3_" + string(1,pin[1]) + "/value";
    } else {
	directory = "/dev/gpios/q7_3v3/q7_gpio_3v3_" + string(1,pin[0]) + string(1,pin[1]) + "/value";
    }
    myFile.open(directory.c_str());
    myFile << 0;
    myFile.close();
    //cout << "The value of pin " << pin[0] << pin[1] << " was set to off\n";
}

string readValue(char pin[]) {
    string answer;
    string directory;
    if(pin[0] == '0') {
	directory = "/dev/gpios/q7_3v3/q7_gpio_3v3_" + string(1,pin[1]) + "/value";
    } else {
	directory = "/dev/gpios/q7_3v3/q7_gpio_3v3_" + string(1,pin[0]) + string(1,pin[1]) + "/value";
    }
    ifstream myFile;
    myFile.open(directory.c_str());
    if(myFile.is_open()) {
	getline(myFile, answer);
	if(answer.compare("1") == 0) answer = "n |";
	else answer = "f |";
	return answer;
    } else {
	cout << "There was an error reading the file :(\n";
    }
    myFile.close();
    return "0";
}

string readDirection(char pin[]) {
    string answer;
    string directory;
    if(pin[0] == '0') {
	directory = "/dev/gpios/q7_3v3/q7_gpio_3v3_" + string(1,pin[1]) + "/direction";
    } else {
	directory = "/dev/gpios/q7_3v3/q7_gpio_3v3_" + string(1,pin[0]) + string(1,pin[1]) + "/direction";
    }
    ifstream myFile;
    myFile.open(directory.c_str());
    if(myFile.is_open()) {
	getline(myFile, answer);
	if(answer.compare("out") == 0) answer = "o";
	else answer = "i";
	return answer;
    } else {
	cout << "There was an error reading the file :(\n";
    }
    myFile.close();
    return "0";
}

void resetPins() {
    for(int i = 0; i < numOfPins; i++) {
	char pin[2];
	strcpy(pin, pins[i].c_str());
	setOn(pin);
	setIn(pin);
    }
}

void loadPins(char command[]) {
    for(int i = 1; i < 45; i++) {
	if(command[i] == 'o') {
	    string p = pins[i - 1];
	    char pin[2];
	    strcpy(pin, p.c_str());
	    setOut(pin);
	} else if(command[i] == 'i') {
	    string p = pins[i - 1];
	    char pin[2];
	    strcpy(pin, p.c_str());
	    setIn(pin);
	} else if(command[i] == 'n') {
	    string p = pins[i - 23];
	    char pin[2];
	    strcpy(pin, p.c_str());
	    setOn(pin);
	} else if(command[i] == 'f') {
	    string p = pins[i - 23];
	    char pin[2];
	    strcpy(pin, p.c_str());
	    setOff(pin);
	}
    }
}






