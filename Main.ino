#include <Adafruit_SH1106_STM32.h>
#include <PixyI2C.h>
#include <Wire.h>

PixyI2C pixy;
Adafruit_SH1106 display(-1);

// Varieble fllowed by the rules of Defining
#define LDRSense 300
#define shootSense 3000

// Variebles (Data type => integer)
int gy;
int dir;
int buff[8];
int LCD = 0;
int Diff;
int LDR_COUNT;
int angleBall,angleGoal,goal;
int shootAmount, shootCount;
int v = 45000;
int distanceBall, distanceGoal;
int KP, KI, KD;
int xBall, yBall;
int xGoal, yGoal;
int shr, shl, shb;
double Heading;
int counter = 0;
int LastHeading, LastTime;
int LDRF, LDRL, LDRR, LDRB;
int LDR_F, LDR_L, LDR_R, LDR_B;
// R 1
// int xRobot = 113, yRobot = 165;
// R 2
int xRobot = 115, yRobot = 155;


// Variebles (Data type => boolean)
bool isBall, isGoal;
bool shootSet;

void setup() {
  initiall();
}

void loop() {
  Algorithm2();
}