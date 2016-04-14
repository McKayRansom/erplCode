#include "dcmotor.h"
#include "pid.h"
#include <sys/time.h>
#include <unistd.h>
#include <thread>
#include <cstdlib>
#include <cstdio>
#include <mutex>

std::mutex speed_mutex;
std::mutex pos_mutex;

DCMotor::DCMotor(int channel, int addr, int freq) : pwm(addr), decoder() {
    if (channel == 0) {
        pwmPin = 8;
        in2Pin = 9;
        in1Pin = 10;
    } else if (channel == 1) {
        pwmPin = 13;
        in2Pin = 12;
        in1Pin = 11;
    } else if (channel == 2) {
        pwmPin = 2;
        in2Pin = 3;
        in1Pin = 4;
    } else if (channel == 3) {
        pwmPin = 7;
        in2Pin = 6;
        in1Pin = 5;
    } else printf("ERROR: incorrect motor channel, must be between 0-3\n");

	i2cAddr = addr;
    runningPID.store(false);
    pwmSpeed = 0;
    pwmSpeedOld = 0;
    pwm.setPWMFreq(freq); // default @1600Hz PWM freq
    run(RELEASE);
}

DCMotor::~DCMotor() {
    run(RELEASE);
}

void DCMotor::run(int command) {
	if (command == FORWARD) {
		setPin(in1Pin, 1);
        setPin(in2Pin, 0);
	}
	if (command == BACKWARD) {
		setPin(in1Pin, 0);
		setPin(in2Pin, 1);
	}
	if (command == RELEASE) {
		setPin(in1Pin, 0);
		setPin(in2Pin, 0);
	}
}

void DCMotor::setSpeed(int speed) {
	if (speed < -255) speed = -255;
	if (speed > 255) speed = 255;
    if (speed > 0) {
        run(BACKWARD);
        pwm.setPWM(pwmPin, 0, speed * 16);
    } else if (speed < 0) {
        run(FORWARD);
        pwm.setPWM(pwmPin, 0, abs(speed) * 16);
    }
    else if (speed == 0) {
        run(RELEASE);
    }
    pwmSpeed = speed;
}

void DCMotor::setGradSpeed(int speed) {
    if (speed > 255 ) {
        speed = 255;
    } else if (speed < -255) {
        speed = -255;
    }
    //printf("speed changing from %d to %d\n", pwmSpeedOld, speed);

    if (speed < pwmSpeedOld) inc = -1;
    else inc = 1;

    for (int i=pwmSpeedOld; i != speed ; i += inc) {
        //printf("old speed: %d, current speed: %d, new speed: %d\n", pwmSpeedOld, i, speed);
        setSpeed(i);
        usleep(6000);
    }
    pwmSpeedOld = speed;
}

void DCMotor::setPin(int pin, int value) {
	if (pin < 0 || pin > 15) {
        printf("ERROR: pin must be between 0 and 15\n");
        return;
    }
	if(value != 0 && value != 1) {
        printf("ERROR: value must be 0 or 1\n");
        return;
    }
	if (value == 0) pwm.setPWM(pin, 0, 4096);
	if (value == 1) pwm.setPWM(pin, 4096, 0);
}

void DCMotor::setHome() {
    decoder.clearCntr();
}

double DCMotor::getSpeed() {
    static bool initialized = false;
    static struct timeval timeVal;
    static struct timezone timeZone;
    static unsigned int i;
    static double times[4];
    static int32_t ticks[4];
    if (!initialized) { // the first time, populate these arrays
        for (int j=0; j<4; j++) {
            gettimeofday(&timeVal, &timeZone);
            times[j] =  timeVal.tv_sec + (timeVal.tv_usec/1000000.0);
            ticks[j] = decoder.readCntr();
            usleep(5000);
        }
        initialized = true;
    }
    speed_mutex.lock();
    gettimeofday(&timeVal, &timeZone);
    times[i] =  timeVal.tv_sec + (timeVal.tv_usec/1000000.0);
    ticks[i] = decoder.readCntr();

    degSpeed = ( (ticks[i] - ticks[(i+1)%4]) /
                 (times[i] - times[(i+1)%4]) ) * DEG_PER_CNT;

    if (++i > 3) i = 0;
    speed_mutex.unlock();

    return degSpeed;
}

// return the current position in degrees from home
double DCMotor::getPosition() {
    pos_mutex.lock();
    int count = decoder.readCntr();
    degPosition = (count % (int) CNT_PER_REV) * DEG_PER_CNT;
    if (count < 0) degPosition += 360;
    pos_mutex.unlock();
    return degPosition;
}

void DCMotor::posPID() {
    printf("starting pid for position %f\n", setPos);
    PID pid(0.02, 2, 0.8, 0);
    //pid.setLimits(-30, 30);
    pid.setDampening(-0.08, 0.08);
    double output;
    while (runningPID.load()) {
        pid.update(setPos, getPosition());
        output = pid.getOutput();
        setSpeed((int) output);
        //setSpeed((int) pid.getOutput());
        printf("setPoint: %-.4f, proccessValue: %-.4f, output: %-.4f\n", setPos, getPosition(), output);
        usleep(20000);
    }
}

void DCMotor::speedPID() {
    printf("starting pid for speed %f\n", setSpd);
    PID pid(0.02, 0.08, 0, 0);
    //pid.setLimits(-30, 30);
    //pid.setDampening(-0.08, 0.08);
    double output;
    while (runningPID.load()) {
        pid.update(setSpd, getSpeed());
        output = pid.getOutput();
        setSpeed((int) output + pwmSpeed);
        //setSpeed((int) pid.getOutput() + pwmSpeed);
        printf("setPoint: %-.4f, proccessValue: %-.4f, output: %-.4f\n", setSpd, getSpeed(), output);
        usleep(20000);
    }
}

void DCMotor::stopPID() {
    runningPID.store(false);
    usleep(50000);
}

void DCMotor::pidPosition(double position) {
    setPos = position;
    runningPID.store(true);
    std::thread thr(&DCMotor::posPID, this);
    thr.detach();
}

void DCMotor::pidSpeed(double speed) {
    setSpd = speed;
    runningPID.store(true);
    std::thread thr(&DCMotor::speedPID, this);
    thr.detach();
}
