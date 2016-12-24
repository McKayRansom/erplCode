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

int main() {
	Imu* imu = nullptr;
	std::ofstream myfile;
	try {
		imu = new Imu();
		printf("Successfully connected to the IMU\n");
		sleep(2);
		//break;
	} catch (int e) {
		printf("FAILED to connect to the IMU\n");
		sleep(10);
	}
	myfile.open("log.txt");
	myfile << "Start\n";
	try {
		int failures = 0;
		for (int j = 0; j < 100; j ++) {
		//printf("start\n");
		unsigned char imuData[43];
		//Byte * dataPointer = &imuData;
		//printf("preCommand: %x\n", imuData[0]);
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
         * NOTE: this data will be parsed by COSMOS when it arrives
         */
		unsigned char command = imuData[0];
		//if (command == (char) 0xCB) {
			printf("command: %x\n", command);
			printf("byte1: %x\n", imuData[1]);
			//int otherTimer = *(unsigned int *)(&imuData + 37);
			float ax = convertToFloat(imuData + 1);
			float ay = convertToFloat(imuData + 1 + 8);
			float az = convertToFloat(imuData +1 + 16);
			printf("ax: %4.2f \n", ax);
			printf("ay: %4.2f \n", ay);
			printf("az: %4.2f \n", az);
			//printf("time: %u \n", otherTimer);
			Byte reversed[4];
			reversed[0] = imuData[40];
			reversed[1] = imuData[39];
			reversed[2] = imuData[38];
			reversed[3] = imuData[37];
			unsigned int timer = *(unsigned int *)(&reversed);
			printf("timer: %f \n", timer/62500.00);
			uint16_t checksum = 0;
			for (int i = 0; i < 43 - 2; i++) {
				checksum += imuData[i];
			}
			reversed[0] = imuData[42];
			reversed[1] = imuData[41];
			reversed[2] = 0;
			reversed[3] = 0;
			uint16_t expected = 0;
			expected = imuData[43 - 2] << 8;
			expected += imuData[43 - 1];
			printf("checksum expected : %u\n", expected);
			printf("actual checksum   : %u\n", checksum);
			if (expected != checksum) {
				printf("*****************bad data ******************\n");
				failures++;
				usleep(100);
				//Byte buffer [10];
				//imu -> readComPort(buffer, 10);
			} else {
				printf("good checksum!\n");
			}
		//} else {
			//printf("bad data**************************************8\n");
		//}
		//myfile << std::hex;
		//for (int i =0; i < 43; i++) {
			//printf("%x\n", imuData[i]);
			//char buffer[5];
			//std::itoa(imuData[i], buffer, 16);
			//myfile << buffer << std::endl;
		//}
		//myfile << std::endl;
		//std::cout << "mid" << std::endl;
    unsigned char imuQuat[23];
		imu->getQuaternion(imuQuat);
        /* This buffer receives the following quaternion data:
         *  Byte  1     0xDF    Command Echo
         *  Bytes 2-5   q0      (IEEE-754 Floating Point)
         *  Bytes 6-9   q1      (IEEE-754 Floating Point)
         *  Bytes 10-13 q2      (IEEE-754 Floating Point)
         *  Bytes 14-17 q3      (IEEE-754 Floating Point)
         *  Bytes 18-21 Timer   32-bit Unsigned Integer
         *  Bytes 22-23 Checksum
         * NOTE: this data will be parsed by COSMOS when it arrives
         */
			command = imuQuat[0];
			if (command == 0xDF) {
			float q0 = convertToFloat(imuQuat + 1);
			printf("command: %x\n", command);
			printf("q0: %4.2f \n", q0);

			
			printf("\n");
			}
			usleep(20000);
		}
		printf("\n failed: %u out of 100 times\n", failures);
	} catch (int e) {
		printf("lost connection with the IMU%u\n", e);
		myfile.close();
	}
	myfile.close();

	delete imu;
	return 0;
}
