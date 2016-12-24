#include "imu.h"
#include <termios.h> // terminal io (serial port) interface
#include <fcntl.h>   // File control definitions
#include <errno.h>   // Error number definitions
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <iostream>
#include <string>

// Clears the com port's read and write buffers
int Purge(ComPortHandle comPortHandle){
    if (tcflush(comPortHandle,TCIOFLUSH)==-1){
        printf("flush failed\n");
        return 0;
    }
    return 1;
}

// Opens a com port with the correct settings for communicating with a MicroStrain 3DM-GX3-25 sensor
ComPortHandle OpenComPort(const char* comPortPath){
    ComPortHandle comPort = open(comPortPath, O_RDWR | O_NOCTTY);
    if (comPort== -1){ //Opening of port failed
				printf("failed to open port!\n");
        throw 1;
    }
    //Get the current options for the port...
    struct termios options;
    tcgetattr(comPort, &options);
    //set the baud rate to 115200
    int baudRate = B115200;
    cfsetospeed(&options, baudRate);
    cfsetispeed(&options, baudRate);
    //set the number of data bits.
    options.c_cflag &= ~CSIZE;    // Mask the character size bits
    options.c_cflag |= CS8;
    //set the number of stop bits to 1
    options.c_cflag &= ~CSTOPB;
    //Set parity to None
    options.c_cflag &=~PARENB;
    //set for non-canonical (raw processing, no echo, etc.)
    options.c_iflag = IGNPAR; // ignore parity check close_port(int
    options.c_oflag = 0; // raw output
    options.c_lflag = 0; // raw input
    //Time-Outs -- won't work with NDELAY option in the call to open
    options.c_cc[VMIN]  = 0;      // block reading until RX x characers. If x = 0, it is non-blocking.
    options.c_cc[VTIME] = 1 ;//changed from 1 to 10     // Inter-Character Timer -- i.e. timeout= x*.1 s
    //Set local mode and enable the receiver
    options.c_cflag |= (CLOCAL | CREAD);
    //Purge serial port buffers
    Purge(comPort);
    //Set the new options for the port...
    int status=tcsetattr(comPort, TCSANOW, &options);
    if (status != 0){ //For error message
        printf("Configuring comport failed\n");
        return status;
    }
    //Purge serial port buffers
    Purge(comPort);
    return comPort;
}

// Closes a port that was previously opened with OpenComPort
void CloseComPort(ComPortHandle comPort){
    close(comPort);
}

// readComPort
// read the specified number of bytes from the com port
int Imu::readComPort(unsigned char* bytes, int size){
    int bytesRead = read(comPort, bytes, size);
    int temp;
    while (bytesRead<size) {
        temp = read(comPort, &bytes[bytesRead], size - bytesRead);
        bytesRead += temp;
        if (bytesRead < size && temp == 0) {
					printf("failed to read data, got: %u, expected: %u\n", bytesRead, size);
				 	//throw 1;
					break;
				}
    }
    return bytesRead;     
}

// writeComPort
// send bytes to the com port
int Imu::writeComPort(unsigned char* bytes, int size){
    int temp = write(comPort, bytes, size);
    if (temp <= 0) {
			printf("failed to write to com port\n");
			throw 1;
		}
    else return temp;
}

//scandev
//finds attached microstrain devices
char* scandev(){
    FILE *instream;
    char devnames[255][255];//allows for up to 256 devices with path links up to 255 characters long each
    int devct=0; //counter for number of devices
    int userchoice=0;
    char* device = NULL;
    std::string str = "find /dev/serial -print | grep -i ftdi";
    const char *command = str.c_str(); // search /dev/serial for microstrain devices
    instream=popen(command, "r"); // execute piped command in read mode
    if(!instream){//SOMETHING WRONG WITH THE SYSTEM COMMAND PIPE...EXITING
        printf("ERROR BROKEN PIPELINE %s\n", command);
        throw 1;
        return device;
    }
    for(int i=0;i<255&&(fgets(devnames[i],sizeof(devnames[i]), instream));i++){//load char array of device addresses
        ++devct;
    }
    for(int i=0;i<devct;i++){
        for(unsigned int j=0;j<sizeof(devnames[i]);j++){
            if(devnames[i][j]=='\n'){
                devnames[i][j]='\0';//replaces newline inserted by pipe reader with char array terminator character 
                break;//breaks loop after replacement
            }
        }
    }

    if(devct>0){
        device=devnames[userchoice];
        return device;
    }
    else{
				printf("error finding device\n");
        throw 1;
        return device;
    }
}

//constructor for the IMU class. Finds and establishes a connection
//with the IMU.
Imu::Imu(){
    char* dev;
    char a;
    a='\0';
    dev=&a;
        dev=scandev();
        if(strcmp(dev,"")!=0){
            comPort = OpenComPort(dev);
        }
        else{
            throw 1;
        }
    if(comPort < 0) throw 1;
		sleep(1);
		char buffer [30];
		read(comPort, buffer, 30);
		return;
		Byte c1 = (Byte) 0xDB;
		Byte c2 = (Byte) 0xA8;
		Byte c3 = (Byte) 0xB9;
		Byte c4 = 0;
		//Byte c5 = 0;
//		while (true) {
		sleep(1);
		printf("writing Command:%x\n", c1);
		writeComPort(&c1, 1);
		writeComPort(&c2, 1);
		writeComPort(&c3, 1);
		writeComPort(&c4, 1);
		for (int i = 0; i < 16; i++) {
			writeComPort(&c4, 1);
		}
		//writeComPort(&c4, 1);
		//writeComPort(&c4, 1);
		//writeComPort(&c4, 1);
		//writeComPort(&c5, 1);
		Byte data[18];
		int size = readComPort(data, 18);
		if (data[0] == 0xDB) {
		printf("echo: %x\n", data[0]);
//		printf("smalltime: %x\n", data[4]);
		//Byte reversed[4];
		//reversed[0] = data[4];
		//reversed[1] = data[3];
		//reversed[2] = data[2];
		//reversed[3] = data[1];
		//printf("1: %x\n", reversed[0]);
		//printf("2: %x\n", reversed[1]);
		//printf("3: %x\n", reversed[2]);
		////printf("4: %x\n", reversed[3]);
		//unsigned int time = *(unsigned int *) (&reversed);
		//float time2 = time / 62500.00;
		//printf("current time: %u\n", time);
		//printf("current time2: %f\n", time2);
		printf("Data cond1: %x\n", data[6]);
		printf("Data cond2: %x\n", data[7]);
		} else {
			printf("failed :(\n");
		}
//}
}

//////////////////////////////////////////////////////////////////////
//Returns a packet of data from the IMU according the settings.     //
//To learn more about the settings and the stucture of the data, see//
//"3DM-GX3-25 Single Byte Data Communications Protocol"             //
//////////////////////////////////////////////////////////////////////
void Imu::getdata(Byte (&data)[43]) {
    int size;
    do {
        Purge(comPort); // flush port
        writeComPort(&dataCommand, 1); // write command to port
        size = readComPort(&data[0], 43);
    } while (size == 0);
}

////////////////////////////////////////////////////////////////////
//Returns the Quaternion data from the IMU.  In order to read this//
//data, you must change the settings in the IMU.  By default, the //
//necessary settings are turned off.  To learn how to change the  //
//settings and which settings need to be changed, read            //
//"3DM-GX3-25 Single Byte Data Communications Protocol"           //
////////////////////////////////////////////////////////////////////
void Imu::getQuaternion(Byte (&data)[23]) {
    int size;
    do {
        Purge(comPort); // flush port
        writeComPort(&quatCommand, 1); // write command to port
        size = readComPort(&data[0], 23);
    } while (size == 0);
}

//Destructor for the IMU class.
Imu::~Imu(){
    CloseComPort(comPort);
}

//Prints the Hexadecimal data in the correct format.
void Imu::printHexByte(char byte) {
    if (byte < 16) {
        printf("0");
    }
    printf("%x ", byte);
}

//When the IMU sends a packet of data, it sends a Checksum at the end
//in order to catch corrupted data.  This function checks the Checksum.
void Imu::calcChecksum(unsigned char* data, int size, uint16_t* checksum) {
    uint16_t checksum_byte1 = 0;
    uint16_t checksum_byte2 = 0;
    for(int i=0; i<size; i++) {
        checksum_byte1 += data[i];
        checksum_byte2 += checksum_byte1;
    }
    *checksum = (checksum_byte1 << 8) + checksum_byte2;
}
