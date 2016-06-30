#include "cosmos/cosmosQueue.h"
#include "gps/gps.h"
//#include "camera/camera.h"
#include "tam/tam.h"
#include "imu/imu.h"
#include <raspicam/raspicam.h>
#include <wiringPi.h>
#include <sys/time.h>
#include <unistd.h>
#include <sstream>
#include <cstdio>
#include <mutex>
#include <condition_variable>
#ifdef USE_FVM400
#include "fvm400/fvm400.h"
#endif

using namespace raspicam;

// gather data from various sensors
void imu(SensorPacket* p);
void tam(SensorPacket* p);
void systemTimestamp(uint32_t &stime, uint32_t &ustime);
//void camera(CameraPacket* p);

// set up condition variables to wake the TAM and GPS threads
std::mutex tam_m;
std::condition_variable tam_cv;
bool tamStart = false;

// initialize packets
TimePacket* tPacket = nullptr;
SensorPacket* sPacket = nullptr;

// make the CosmosQueue global (so that all threads can access it)
CosmosQueue queue(4810, 512, 8);

std::atomic<bool> camera_state;
std::condition_variable camera_cv;
std::mutex camera_mutex;
PI_THREAD (cameraControl) {
    camera_state.store(false);
    CameraPacket* cPacket = nullptr;
    raspicam::RaspiCam camera;
    camera.setWidth(320);
    camera.setHeight(240);
    camera.setFormat(RASPICAM_FORMAT_GRAY);
    if (!camera.open()) printf("ERROR: Camera not opened\n");
    else {
        printf("Camera opened\n");
        usleep(3000000);
        while (true) {
            if (camera_state.load()) {
                cPacket = new CameraPacket();
                camera.grab();
                systemTimestamp(cPacket->sysTimeSeconds, cPacket->sysTimeuSeconds);
                camera.retrieve(cPacket->pBuffer);
                queue.push_tlm(cPacket);
                usleep(5000);
            } else {
                std::unique_lock<std::mutex> lk(camera_mutex);
                camera_cv.wait(lk, []{return camera_state.load();});
            }
        }
    }
    return nullptr;
}

PI_THREAD (cmdThread) {
    Packet* cmdPacket = nullptr;
    while (true) {
        if (queue.pop_cmd(cmdPacket)) {
            switch (cmdPacket->id) {
                case CAM_CMD_ID:
                    {
                        CameraPowerCmd* camCmd = static_cast<CameraPowerCmd*>(cmdPacket);
                        camCmd->CameraPowerCmd::convert();
                        if (camCmd->state) {
                            std::lock_guard<std::mutex> lk(camera_mutex);
                            camera_state.store(true);
                            camera_cv.notify_one();
                            printf("Camera enabled\n");
                        } else {
                            camera_state.store(false);
                            printf("Camera disabled\n");
                        }
                    }
                    break;
                default:
                    printf("unknown command received\n");
            }
            delete cmdPacket;
            cmdPacket = nullptr;
        }
        usleep(200000);
    }
}

PI_THREAD (tamControl) {
    std::unique_lock<std::mutex> lk(tam_m);
    lk.unlock();
    while (true) {
        lk.lock();
        tam_cv.wait(lk, []{return tamStart;});
        tam(sPacket);
        tamStart = false;
        lk.unlock();
        tam_cv.notify_one();
    }
}

#ifdef USE_FVM400
std::mutex fvm400_m;
std::condition_variable fvm400_cv;
bool fvm400Start = false;
PI_THREAD (fvm400Control) {
    FVM400 mag("/dev/ttyUSB1");
    FVM400_data data;
    std::unique_lock<std::mutex> lk(fvm400_m);
    lk.unlock();
    while (true) {
        lk.lock();
        fvm400_cv.wait(lk, []{return fvm400Start;});
        data = mag.getData();
        sPacket->fvm400x = data.comp1;
        sPacket->fvm400y = data.comp2;
        sPacket->fvm400z = data.comp3;
        fvm400Start = false;
        lk.unlock();
        fvm400_cv.notify_one();
    }
}
#endif

int main() {

    // initialize devices
    Gps gps(1, "/dev/ttyAMA0", 9600);
    imu(sPacket);
    tam(sPacket);

    // start threads
    if (piThreadCreate(cameraControl) != 0) {
        perror("Camera control thread didn't start");
    }
    if (piThreadCreate(cmdThread) != 0) {
        perror("Camera control thread didn't start");
    }

    // set high priority for this thread
    pid_t pid = getpid();
    std::stringstream cmd;
    cmd << "renice -n -1 -p " << pid;
    if (pid > 0) system(cmd.str().c_str());

    // initialize the TAM with the higher priority
    if (piThreadCreate(tamControl) != 0) {
        perror("TAM control thread didn't start");
    }
#ifdef USE_FVM400
    // initialize the FVM400 with the higher priority
    if (piThreadCreate(fvm400Control) != 0) {
        perror("FVM400 control thread didn't start");
    }
#endif
    std::stringstream cmd2;
    cmd2 << "renice -n -2 -p " << pid;
    if (pid > 0) system(cmd2.str().c_str());

    // these values help time the packets
    long timer = 0, difference = 0;
    struct timeval start, next;
    usleep(1000000);
    while (true) {

        // get timestamps and send time packet
        tPacket = new TimePacket();
        gps.timestampPPS(tPacket->sysTimeSeconds, tPacket->sysTimeuSeconds);
        start.tv_sec = tPacket->sysTimeSeconds;
        start.tv_usec = tPacket->sysTimeuSeconds;
        tPacket->gpsTime = gps.getTime();
        //printf("pushed TIME packet with gpstime = %f\n", tPacket->gpsTime);
        queue.push_tlm(tPacket);
        difference = 0;
        timer = 0;
        // every second, do this 15 times
        for (int j=0; j<15; j++) {

            sPacket = new SensorPacket();
            do { // delay a bit (20ms per packet)
                gettimeofday(&next, nullptr);
                difference = next.tv_usec - start.tv_usec + (next.tv_sec - start.tv_sec) * 1000000;
                if (difference < timer) usleep(20);
            } while (difference < timer);
            if (difference > 940000) break;
            //printf("started cycle at %li/%li\n", difference, timer);

            { // tell the TAM to collect data
                std::lock_guard<std::mutex> lk(tam_m);
                tamStart = true;
                systemTimestamp(sPacket->sysTimeSeconds, sPacket->sysTimeuSeconds);
            } tam_cv.notify_one();
#ifdef USE_FVM400
            { // tell the FVM400 to collect data
                std::lock_guard<std::mutex> lk(fvm400_m);
                fvm400Start = true;
            } fvm400_cv.notify_one();
#endif
            imu(sPacket); // get IMU data
            { // wait for TAM to finish
                std::unique_lock<std::mutex> lk(tam_m);
                tam_cv.wait(lk, []{return !tamStart;});
            }
#ifdef USE_FVM400
            { // wait for FVM400 to finish
                std::unique_lock<std::mutex> lk(fvm400_m);
                fvm400_cv.wait(lk, []{return !fvm400Start;});
            }
#endif
            queue.push_tlm(sPacket);

            timer += 66666;
        }
    }
    return 0;
}

void imu(SensorPacket* p) {
    static Imu imuSensor;
    if (p == nullptr) return;
    imuSensor.getdata(p->imuData);
    imuSensor.getQuaternion(p->imuQuat);
}

void tam(SensorPacket* p) {
    static Tam tamSensor;
    if (p == nullptr) return;
    p->tamA = tamSensor.getData(0);
    p->tamB = tamSensor.getData(1);
    p->tamC = tamSensor.getData(2);
}

/*
void camera(CameraPacket* p) {
    static Camera cam;
    if (p == nullptr) return;
    cam.getFrame(p->pBuffer);
}
*/

void systemTimestamp(uint32_t &stime, uint32_t &ustime) {
    static struct timeval timeVal;
    gettimeofday(&timeVal, nullptr);
    stime =  timeVal.tv_sec;
    ustime = timeVal.tv_usec;
}
