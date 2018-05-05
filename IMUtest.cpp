#include "src/imu/imu.h"
#include <cstdio>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <stdlib.h>
#include <chrono>

//reverses bytes and converts to IEEE-754 Floating Point
float convertToFloat (Byte* value) {
	Byte reversed[4];
	reversed[0] = value[3];
	reversed[1] = value[2];
	reversed[2] = value[1];
	reversed[3] = value[0];
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
	//magnetometer data
	float magX;
	float magY;
	float magZ;
	//time in seconds since the IMU was powered on.
	float timer;
	bool valid;
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
		convertedData.accelY = convertToFloat(imuData + 5);
		convertedData.accelZ = convertToFloat(imuData + 9);
		printf("ax: %4.2f \n", convertedData.accelX);
		printf("ay: %4.2f \n", convertedData.accelY);
		printf("az: %4.2f \n", convertedData.accelZ);
		convertedData.angRateX = convertToFloat(imuData + 13);
		convertedData.angRateY = convertToFloat(imuData + 17);
		convertedData.angRateZ = convertToFloat(imuData + 21);
		printf("angx: %4.2f \n", convertedData.angRateX);
		printf("angy: %4.2f \n", convertedData.angRateY);
		printf("angz: %4.2f \n", convertedData.angRateZ);
		convertedData.magX = convertToFloat(imuData + 25);
		convertedData.magY = convertToFloat(imuData + 29);
		convertedData.magZ = convertToFloat(imuData + 33);
		printf("mx: %4.2f \n", convertedData.magX);
		printf("my: %4.2f \n", convertedData.magY);
		printf("mz: %4.2f \n", convertedData.magZ);
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
	return convertedData;
}

//quaternion structure for returning
struct Quaternion	{
	float q0;
	float q1;
	float q2;
	float q3;
	float timer;
	bool valid = false;
};
//
Quaternion getQuat(Imu* imu) {

	unsigned char imuQuat[23];
	Quaternion convertedData;
	imu->getQuaternion(imuQuat);
    /* This buffer receives the following quaternion data:
     *  Byte  1     0xDF    Command Echo
     *  Bytes 2-5   q0      (IEEE-754 Floating Point)
     *  Bytes 6-9   q1      (IEEE-754 Floating Point)
     *  Bytes 10-13 q2      (IEEE-754 Floating Point)
     *  Bytes 14-17 q3      (IEEE-754 Floating Point)
     *  Bytes 18-21 Timer   32-bit Unsigned Integer
     *  Bytes 22-23 Checksum
     */
	 //find the actual checksum of received data
	 // by calculating sum of every Byte
	 uint16_t actualChecksum = 0;
	 for (int i = 0; i < 23 - 2; i++) {
		 actualChecksum += imuQuat[i];
	 }
	 //checksum sent with the data
	 //this is the expected value with valid data
	 uint16_t expectedChecksum = 0;
	 expectedChecksum = imuQuat[43 - 2] << 8; //upper Byte
	 expectedChecksum += imuQuat[43 - 1];			//lower Byte
	 printf("checksum expected : %u\n", expectedChecksum);
	 printf("actual checksum   : %u\n", actualChecksum);
	 if (expectedChecksum != actualChecksum) {
	 	printf("*****************bad data ******************\n");
		convertedData.valid = false;
	 } else {
	 	printf("good checksum!\n");
		convertedData.valid = true;
		unsigned char command = imuData[0];
		printf("command: %x\n", command);
		printf("byte1: %x\n", imuQuat[1]);
		convertedData.q0 = convertToFloat(imuQuat + 1);
		convertedData.q1 = convertToFloat(imuQuat + 5);
		convertedData.q2 = convertToFloat(imuQuat + 9);
		convertedData.q3 = convertToFloat(imuQuat + 13);
		printf("q0: %4.2f \n", convertedData.q0);
		printf("q1: %4.2f \n", convertedData.q1);
		printf("q2: %4.2f \n", convertedData.q2);
		printf("q3: %4.2f \n", convertedData.q3);
		Byte reversed[4];
		reversed[0] = imuQuat[20];
		reversed[1] = imuQuat[19];
		reversed[2] = imuQuat[18];
		reversed[3] = imuQuat[17];
		unsigned int timer = *(unsigned int *)(&reversed);
		float timerSeconds = timer/62500.00;
		convertedData.timer = timerSeconds;
		printf("timer: %f \n", timerSeconds);
	}
	return convertedData;
}

//function to get date and time from:
//https://stackoverflow.com/questions/34963738/c11-get-current-date-and-time-as-string
std::string get_date_string(std::chrono::time_point t) {
  auto as_time_t = std::chrono::system_clock::to_time_t(t);
  struct tm tm;
  if (::gmtime_r(&as_time_t, &tm))
    if (std::strftime(some_buffer, sizeof(some_buffer), "%F", &tm))
      return std::string{some_buffer};
  throw std::runtime_error("Failed to get current date as string");
}

int main() {
	//IMU object
	Imu* imu = nullptr;

	// log file
	std::ofstream logFile;

	// IMU driver will throw an error if it can't connect to the imu
	try {
		imu = new Imu();
		printf("Successfully connected to the IMU\n");
		sleep(2);
		//break;
	} catch (int e) {
		printf("FAILED to connect to the IMU\n");
		return 1;
	}
	logFile.open(get_date_string(std::system_clock::now()););
	logFile << "Time, AccelX, AccelY, AccelZ, AngRateX, AngRateY, AngRateZ, MagX, MagY, MagZ, Time2, q0,q1,q2,q3\n";
	try {
		//keep track of how many bad packets we get
		int badPackets = 0;
		int totalAttempts = 0;
		auto startTime = std::chrono::steady_clock::now();
		do {
			ImuData accelData;
			accelData = getData(imu);
			totalAttempts++;
			if (accelData.valid) {
				logFile << accelData.timer << ", ";
				logFile << accelData.accelX << ", ";
				logFile << accelData.accelY << ", ";
				logFile << accelData.accelZ << ", ";
				logFile << accelData.angRateX << ", ";
				logFile << accelData.angRateY << ", ";
				logFile << accelData.angRateZ << ", ";
				logFile << accelData.magX << ", ";
				logFile << accelData.magY << ", ";
				logFile << accelData.magZ << ", ";
			} else {
				//data invalid
				badPackets ++;
				for (int i =0; i<10;i++) {
					logFile << ", ";
				}
			}
			Quaternion quatData;
			quatData = getQuaternion(imu);
			totalAttempts++;
			if (quatData.valid) {
				logFile << quatData.timer << ", ";
				logFile << quatData.q0 << ", ";
				logFile << quatData.q1 << ", ";
				logFile << quatData.q2 << ", ";
				logFile << quatData.q3;
			} else {
				//data invalid
				badPackets ++;
				for (int i =0; i<5;i++) {
					logFile << ", ";
				}
			}
			logFile << std::endl;
			logFile.flush();

			//wait between getting data
			//this number is completely arbitrary and should probably be lower
			//usleep(20000);
		} while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - startTime).count()
< 1000);
		printf("\n failed: %u out of %u times\n", badPackets, totalAttempts);
	} catch (int e) {
		//we got an error in the driver
		//this usually doesn't actually mean we lost connection.
		//Just something went wrong in receiving data
		//TODO: reconnect instead of giving up entirely
		printf("lost connection with the IMU%u\n", e);
		myfile.close();
	}
	myfile.close();

	delete imu;
	return 0;
}
