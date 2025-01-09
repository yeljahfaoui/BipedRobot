/*
 * @Author : Youssef EL JAHFAOUI.
 * @Country : Morocco.
 * @Purpose of this project : Personal tests and research for robotics.
 * @Implementation year : 2023.
*/
#include "Servo.h"
#include "SR04.h"
#define TRIG_PIN 10
#define ECHO_PIN 11

const byte servosPins[8] = {2,3,4,5,6,7,8,9};

const int ALPHA_ZERO = 20; // Starting alpha angle value is 40 degree.

const int leanAngleMax = 10; // Lean by 10 degree max.
const int changeStanceDelay = 20; // 20 ms for changing the stance delay.
const int interDelay = 50; //  50 ms for inter-delay.
const long obstacleLimit = 10; // The limit distance to consider the robot is detecting an obstacle.
const int crouchLimit = 70; // The max value for the crouch.
const int kneeStep = 2;

int moveSpeed = 10; // 10 ms for general move speed.
int step1 = 10; // 10 ms for movement step.
long d; // Distance variable.
int moveSpeed2 = 1; // 1 ms for knee move speed.

bool moveJustStarted = true;

int currentServoPosition[8];

int currentAlpha = 0;
int currentBeta = 0;
int betaR = 0;
int betaL = 0;

Servo servos[8];
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);

void setup() {
  // put your setup code here, to run once:
  initServos();
  delay(2000);
  startPosition();
  betaR = betaL = currentBeta;
  delay(2000);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  // move forward.
  start();
}

// Initializing servos and calibrating
void initServos() {
  for(int i = 0 ; i < 8 ; i++) {
    servos[i].attach(servosPins[i]);
    if(i == 1) { // Right Knee default angle
      servos[i].write(180);
      currentServoPosition[i] = 180;
    } else if(i == 5) { //Left knee default angle
      servos[i].write(0);
      currentServoPosition[i] = 0;
    } else {
      servos[i].write(90);
      currentServoPosition[i] = 90;
    }
  }
}

void startPosition() {
  updateStance(ALPHA_ZERO);
}

void start() {
  go(true);
}

// positive angle --> move leg forward.
void moveRightLeg(int angle) {
  servos[0].write(90 + angle);
}

void moveLeftLeg(int angle) {
  servos[4].write(90 - angle);
}

void moveRightKnee(int angle) {
  servos[1].write(180 - angle);
}

void moveLeftKnee(int angle) {
  servos[5].write(0 + angle);
}

void moveRightFoot(int angle) {
  servos[2].write(90 + angle);
}

void moveLeftFoot(int angle) {
  servos[6].write(90 - angle);
}

// beta = 2 * alpha.
// gamma = beta - alpha.
// alpha is between -45 & 45 degree. 
// beta always positive between 0 and 180 degree.
// gamma between -45 & 45 degree.
void go(bool moveForward) {

  int dir = moveForward ? 1 : -1;
  int alphaR, alphaL;
  
  alphaR = alphaL = currentAlpha;

  if(moveJustStarted) {
    leanToRight();
    if(kneeStep > 0) {
      for(int i = 1 ; i <= step1 ; i++) {
        bendLeftKnee(alphaL, betaL);
      }
    }
  }
    
  for(int i = 1 ; i <= step1 ; i++) {
    // right leg
    moveRightLeg(alphaR - (i * dir));
    moveRightFoot((betaR - alphaR) + (i * dir)); // gamma = beta - alpha.
    // left leg
    moveLeftLeg(alphaL + (i * dir));
    //moveLeftFoot((betaL - alphaL) - (i * dir));
    resetLeftKnee(alphaL + (i * dir), betaL);
    delay(moveSpeed);
  }

  delay(interDelay);
  
  resetLeanToRight();

  leanToLeft();

  delay(interDelay);

  alphaR -= step1 * dir;
  alphaL += step1 * dir;

  for(int i = 1 ; i <= step1 ; i++) {
    // right leg
    moveRightLeg(alphaR + (i * dir));
    //moveRightFoot((betaR - alphaR) - (i * dir));
    bendRightKnee(alphaR + (i * dir), betaR);
    // left leg
    moveLeftLeg(alphaL - (i * dir));
    moveLeftFoot((betaL - alphaL) + (i * dir));
    delay(moveSpeed);
  }

  alphaR += step1 * dir;
  alphaL -= step1 * dir;

  for(int i = 1 ; i <= step1 ; i++) {
    // right leg
    moveRightLeg(alphaR + (i * dir));
    //moveRightFoot(betaR - alphaR - (i * dir));
    resetRightKnee(alphaR + (i * dir), betaR);
    // left leg
    moveLeftLeg(alphaL - (i * dir));
    moveLeftFoot(betaL - alphaL + (i * dir));
    delay(moveSpeed);
  }

  delay(interDelay);

  resetLeanToLeft();

  leanToRight();
 
  delay(interDelay);

  alphaR += step1 * dir;
  alphaL -= step1 * dir;

  for(int i = 1 ; i <= step1 ; i++) {
    // right leg
    moveRightLeg(alphaR - (i * dir));
    moveRightFoot(betaR - alphaR + (i * dir));
    // left leg
    moveLeftLeg(alphaL + (i * dir));
    //moveLeftFoot(betaL - alphaL - (i * dir));
    bendLeftKnee(alphaL + (i * dir), betaL);
    delay(moveSpeed);
  }

  moveJustStarted = false;
}

void bendRightKnee(int currentAlphaR, int& currentBetaR) {
  if(kneeStep == 0) {
    moveRightFoot(currentBetaR - currentAlphaR);
    return;
  }
  for(int i = 1 ; i <= kneeStep ; i++) {
    moveRightKnee(currentBetaR + i);
    moveRightFoot(currentBetaR - currentAlphaR + i);
    delay(moveSpeed2);
  }
  currentBetaR += kneeStep;
}

void resetRightKnee(int currentAlphaR, int& currentBetaR) {
  if(kneeStep == 0) {
    moveRightFoot(currentBetaR - currentAlphaR);
    return;
  }
  for(int i = 1 ; i <= kneeStep ; i++) {
    moveRightKnee(currentBetaR - i);
    moveRightFoot(currentBetaR - currentAlphaR - i);
    delay(moveSpeed2);
  }
  currentBetaR -= kneeStep;
}

void bendLeftKnee(int currentAlphaL, int& currentBetaL) {
  if(kneeStep == 0) {
    moveLeftFoot(currentBetaL - currentAlphaL);
    return;
  }
  for(int i = 1 ; i <= kneeStep ; i++) {
    moveLeftKnee(currentBetaL + i);
    moveLeftFoot(currentBetaL - currentAlphaL + i);
    delay(moveSpeed2);
  }
  currentBetaL += kneeStep;
}

void resetLeftKnee(int currentAlphaL, int& currentBetaL) {
  if(kneeStep == 0) {
    moveLeftFoot(currentBetaL - currentAlphaL);
    return;
  }
  for(int i = 1 ; i <= kneeStep ; i++) {
    moveLeftKnee(currentBetaL - i);
    moveLeftFoot(currentBetaL - currentAlphaL - i);
    delay(moveSpeed2);
  }
  currentBetaL -= kneeStep;
}

void leanToLeft() {
  for(int i = 0 ; i <= leanAngleMax; i++) {
    servos[3].write(90 - i);
    servos[7].write(90 - i);
    delay(moveSpeed);
  }
}

void leanToRight() {
  for(int i = 0 ; i <= leanAngleMax ; i++) {
    servos[7].write(90 + i);
    servos[3].write(90 + i);
    delay(moveSpeed);
  }
}

void resetLeanToLeft() {
  for(int i = 0 ; i <= leanAngleMax; i++) {
    servos[3].write((90 - leanAngleMax) + i);
    servos[7].write((90 - leanAngleMax) + i);
    delay(moveSpeed);
  }
}

void resetLeanToRight() {
  for(int i = 0 ; i <= leanAngleMax  ; i++) {
    servos[7].write((90 + leanAngleMax) - i);
    servos[3].write((90 + leanAngleMax) - i);
    delay(moveSpeed);
  }
}

void updateStance(int alpha) {
  
  if(alpha == 0 || alpha > crouchLimit || (alpha == currentAlpha)) {
    return;
  }

  int delta;
  int j;

  if(alpha > currentAlpha) {
    delta = alpha - currentAlpha;
    j = 1;
  } else {
    delta = currentAlpha - alpha;
    j = -1;
  }
  
  for(int i = 0 ; i <= delta ; i++) {
    // right leg
    moveRightLeg(currentAlpha + i * j);
    moveRightKnee(2 * (currentAlpha + i * j)); // beta_i = 2 * (alpha0 + i)
    moveRightFoot(currentAlpha + i * j);
    // Left leg
    moveLeftLeg(currentAlpha + i * j);
    moveLeftKnee(2 * (currentAlpha + i * j)); // beta_i = 2 * (alpha0 + i)
    moveLeftFoot(currentAlpha + i * j);
    delay(changeStanceDelay);
  }
  
  // Save current stance positions
  currentAlpha = alpha;
  currentBeta = 2 * currentAlpha;
}
// Signed by : Youssef EL JAHFAOUI
