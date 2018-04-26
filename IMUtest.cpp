#include "src/imu/imu.h"
#include <cstdio>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <stdlib.h>

//reverses bytes and converts to IEEE-754 Floating Point
float convertToFloat (Byte* value) {
	Byte reversed[4];
	reversed[0] = value[0];
	reversed[1] = value[1];
	reversed[2] = value[2];
	reversed[3] = value[3];
	return *(float*)(&reversed);
}

struct ImuData {
	//acceleration in Gs (includes gravity)
	float accelX;
	float accelY;
	float accelZ;
	//angular something
	float angRateX;
	float angRateY;
	float angRateZ;
	//magnetometer data (probably don't need this)
	//currently commented out
	// float magX;
	// float magY;
	// float magZ;
	//time in seconds since the IMU was powered on.
	float timer;
	bool valid = false;
};

//helper funtion that gets data and converts it to floats
//could probably be merged with imu.getData
ImuData getData(Imu* imu) {
	//create buffer to hold data
	unsigned char imuData[43];
	//create struct to hold converted data
	ImuData convertedData;
	//tell IMU driver to get the data from the IMU
	imu->getdata(imuData);
	/* This buffer receives the following data:
	 *  Byte  1     0xCB     Command Echo
	 *  Bytes 2-5   AccelX   (IEEE-754 Floating Point)
	 *  Bytes 6-9   AccelY   (IEEE-754 Floating Point)
	 *  Bytes 10-13 AccelZ   (IEEE-754 Floating Point)
	 *  Bytes 14-17 AngRateX (IEEE-754 Floating Point)
	 *  Bytes 18-21 AngRateY (IEEE-754 Floating Point)
	 *  Bytes 22-25 AngRateZ (IEEE-754 Floating Point)
	 *  Bytes 26-29 MagX     (IEEE-754 Floating Point)
	 *  Bytes 30-33 MagY     (IEEE-754 Floating Point)
	 *  Bytes 34-37 MagZ     (IEEE-754 Floating Point)
	 *  Bytes 38-41 Timer    32-bit Unsigned Integer
	 *  Bytes 42-43 Checksum
		 */

	 //find the actual checksum of received data
	 // by calculating sum of every Byte
	 uint16_t actualChecksum = 0;
	 for (int i = 0; i < 43 - 2; i++) {
		 actualChecksum += imuData[i];
	 }
	 //checksum send with the data
	 //this is the expected value with valid data
	 uint16_t expectedChecksum = 0;
	 expectedChecksum = imuData[43 - 2] << 8; //upper Byte
	 expectedChecksum += imuData[43 - 1];			//lower Byte
	 printf("checksum expected : %u\n", expectedChecksum);
	 printf("actual checksum   : %u\n", actualChecksum);
	 if (expectedChecksum != actualChecksum) {
	 	printf("*****************bad data ******************\n");
		// 	failures++;
		// 	usleep(100);
	 	//Byte buffer [10];
	 	//imu -> readComPort(buffer, 10);
		convertedData.valid = false;
	 } else {
	 	printf("good checksum!\n");
		convertedData.valid = true;
		unsigned char command = imuData[0];
		printf("command: %x\n", command);
		printf("byte1: %x\n", imuData[1]);
		convertedData.accelX = convertToFloat(imuData + 1);
		convertedData.accelY = convertToFloat(imuData + 1 + 8);
		convertedData.accelZ = convertToFloat(imuData +1 + 16);
		printf("ax: %4.2f \n", convertedData.accelX);
		printf("ay: %4.2f \n", convertedData.accelY);
		printf("az: %4.2f \n", convertedData.accelZ);
		convertedData.angRateX = convertToFloat(imuData + 1 + 16);
		convertedData.angRateX = convertToFloat(imuData + 1 + 16 + 8);
		convertedData.angRateX = convertToFloat(imuData +1 + 16 + 16);
		Byte reversed[4];
		reversed[0] = imuData[40];
		reversed[1] = imuData[39];
		reversed[2] = imuData[38];
		reversed[3] = imuData[37];
		unsigned int timer = *(unsigned int *)(&reversed);
		float timerSeconds = timer/62500.00;
		convertedData.timer = timerSeconds;
		printf("timer: %f \n", timerSeconds);
	}
}

//quaternion structure for returning
// struct Quaternion	{
// 	float q0;
// 	float q1;
// 	float q2;
// 	float q3;
// 	bool valid = false;
// };
//
// Quaternion getQuat(Imu* imu) {
//
// }

int main() {
	//IMU object
	Imu* imu = nullptr;

	// log file (currently unused)
	// std::ofstream myfile;

	// IMU driver will throw an error if it can't connect to the imu
	try {
		imu = new Imu();
		printf("Successfully connected to the IMU\n");
		sleep(2);
		//break;
	} catch (int e) {
		printf("FAILED to connect to the IMU\n");
		sleep(10);
	}

	// myfile.open("log.txt");
	// myfile << "Start\n";
	try {
		//keep track of how many bad packets we get
		int badPackets = 0;
		for (int j = 0; j < 100; j ++) {
			ImuData accelData;
			accelData = getData(imu);
			if (accelData.valid) {
				//do something???
			} else {
				//data invalid
				badPackets ++;
				//wait for 100 microseconds
				//this will usually give the connection time to
				//reset or something so the next packet will be valid
				usleep(100);
			}
			//quaternion stuff, TODO:should be moved into helper function
		    // unsigned char imuQuat[23];
				// imu->getQuaternion(imuQuat);
		    //     /* This buffer receives the following quaternion data:
		    //      *  Byte  1     0xDF    Command Echo
		    //      *  Bytes 2-5   q0      (IEEE-754 Floating Point)
		    //      *  Bytes 6-9   q1      (IEEE-754 Floating Point)
		    //      *  Bytes 10-13 q2      (IEEE-754 Floating Point)
		    //      *  Bytes 14-17 q3      (IEEE-754 Floating Point)
		    //      *  Bytes 18-21 Timer   32-bit Unsigned Integer
		    //      *  Bytes 22-23 Checksum
		    //      * NOTE: this data will be parsed by COSMOS when it arrives
		    //      */
				// 	command = imuQuat[0];
				// 	if (command == 0xDF) {
				// 	float q0 = convertToFloat(imuQuat + 1);
				// 	printf("command: %x\n", command);
				// 	printf("q0: %4.2f \n", q0);
				//
				//
				// 	printf("\n");
				// 	}
			//wait between getting data
			//this number is completely arbitrary and should probably be lower
			usleep(20000);
		}
		printf("\n failed: %u out of 100 times\n", badPackets);
	} catch (int e) {
		//we got an error in the driver
		//this usually doesn't actually mean we lost connection.
		//Just something went wrong in receiving data
		//TODO: reconnect instead of giving up entirely
		printf("lost connection with the IMU%u\n", e);
		// myfile.close();
	}
	// myfile.close();

	delete imu;
	return 0;
}