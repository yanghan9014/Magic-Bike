#include "PeripheralNames.h"
#include "PinNames.h"
#include "PwmOut.h"
#include "mbed.h"
#include "mbed_wait_api.h"
#include <cstdio>



PwmOut pedal(D3);
PwmOut direction(D4);
PwmOut brake(D5);

float nowSpeed;
const float speedDelta = 0.02;
const int speedWait = 100000;

const float servoLow = 0.02;   // 2%
const float servoHigh = 0.128; // 12.8%
const float servoDelta = 0.001; //0.1%

const float leftDir = servoHigh;//servoLow + (servoHigh - servoLow) * 0.25;
const float rightDir = servoLow;//servoLow + (servoHigh - servoLow) * 0.75;
const float midDir = (servoLow + servoHigh) / 2;
float nowDir;

bool nowStop;

void setup() {
    pedal.period(0.00002); // 50 kHz
    direction.period(0.02f); // 50 Hz
    brake.period(0.02f);

    nowSpeed = 0;
    nowDir = midDir;
    nowStop = false;
}
float calculateDuty(float speed) {
    float min = 0.3;  // 20%
    float max = 1;
    return min + (max-min) * speed;
}
void run(float speed) { //speed 0~1
    if (speed > 1)
        speed = 1;
    if (speed < 0)
        speed = 0;
    if (speed > nowSpeed) {
        while (speed > nowSpeed) {
            nowSpeed += speedDelta;
            pedal.write(calculateDuty(nowSpeed));
            wait_us(speedWait);
        }
        nowSpeed = speed;
        pedal.write(calculateDuty(nowSpeed));
    }
    else {
        while (speed < nowSpeed) {
            nowSpeed -= speedDelta;
            pedal.write(calculateDuty(nowSpeed));
            wait_us(speedWait);
        }
        nowSpeed = speed;
        pedal.write(calculateDuty(nowSpeed));
    }
}

float turnTrans(int t) {
    return (t/90.0f) * 0.054f;
}

void turnLeftDelta(float delta) {
    int count = delta / servoDelta;
    for (int i = 0; i < count; ++i) {
        nowDir += servoDelta;
        direction.write(nowDir);
        wait_us(20000);
        if (nowDir > leftDir) {
            nowDir = leftDir;
            direction.write(nowDir);
            break;
        }
    }
}


void turnRightDelta(float delta) {
    int count = delta / servoDelta;
    for (int i = 0; i < count; ++i) {
        nowDir -= servoDelta;
        direction.write(nowDir);
        wait_us(20000);
        if (nowDir < rightDir) {
            nowDir = rightDir;
            direction.write(nowDir);
            break;
        }
    }
}


void turnRight() {
    while (nowDir > rightDir) {
        nowDir -= servoDelta;
        direction.write(nowDir);
        wait_us(20000); // wait 50 ms
    }
    nowDir = rightDir;
    direction.write(nowDir);
}

void turnLeft() {
    while (nowDir < leftDir) {
        nowDir += servoDelta;
        direction.write(nowDir);
        wait_us(20000); // wait 50 ms
    }
    nowDir = leftDir;
    direction.write(nowDir);
}

void turnFront() {
    if (nowDir < midDir) {
        while (nowDir < midDir) {
            nowDir += servoDelta;
            direction.write(nowDir);
            wait_us(20000);
        }
    }
    else {
        while (nowDir > midDir) {
            nowDir -= servoDelta;
            direction.write(nowDir);
            wait_us(20000);
        }
    }
    nowDir = midDir;
    direction.write(nowDir);
}

void holdBrake() {
    if (nowStop)
        return;
    float output = servoHigh;
    while (output > servoLow) {
        brake.write(output);
        output -= servoDelta;
        wait_us(10000); //wait 100ms
    }
    nowStop = true;
}

void releaseBrake() {
    if (!nowStop)
        return;
    float output = servoLow;
    while (output < servoHigh) {
        brake.write(output);
        output += servoDelta;
        wait_us(10000); //wait 100ms
    }
    nowStop = false;
}
void waitSec(int us) {
    wait_us(1000000 * us);
}

// main() runs in its own thread in the OS
int main()
{
    DigitalOut led(LED1);
    led = false;
    setup();
    printf("set up finish\n");
    led = true;
    holdBrake();
    printf("DDDDD\n");
    turnFront();
    run(0.39);
    char c[50];
    while (1) {
        scanf("%s", c);
        int t = 0;
        for (int i = 0; c[i] != '\0'; ++i)
            t++;
        printf("I get message: %s\n", c);
        printf("msg len(expect 4, other will be ignore): %d\n", t);
        if (t != 4)
            continue;
        led = !led;
        if (c[0] == 'w') {
            run(0.39);
        }
        else if (c[0] == 's') {
            if (c[3] == '1') {  // s001 for hold
                holdBrake();
            }
            else {              // s000 for release
                releaseBrake();
            }
        }
        else if (c[0] == 'a') {
            turnLeft();
        }
        else if (c[0] == 'd') {
            turnRight();
        }
        else if (c[0] == 'f') {
            turnFront();
        }
        else if (c[0] == 'q') {
            int dir = (c[2] - '0') * 10 + (c[3] - '0');
            turnLeftDelta(turnTrans(dir));
        }
        else if (c[0] == 'e') {
            int dir = (c[2] - '0') * 10 + (c[3] - '0');
            turnRightDelta(turnTrans(dir));
        }
        else {
            printf("Error: get unknow command\n");
        }
        fflush(stdin);
    }
    run(0);
}