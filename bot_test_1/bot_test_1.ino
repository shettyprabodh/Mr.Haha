#include "Wire.h"

const int NUM_PINS = 2;

const int HIGH_POWER[] = {13, 12, 11, A0, A1};
const int LOW_POWER[] = {}; //Currently nothing needs to be grounded

//Front Motors: FORWARD OPERATION: Pin[0] HIGH Pin[1] LOW
const int FRONT_LEFT[] = {10, 9};
const int FRONT_RIGHT[] = {8, 7};

//Rear Motors: FORWARD OPERATION: Pin[0] HIGH Pin[1] LOW
const int REAR_RIGHT[] = {A2, A3};
const int REAR_LEFT[] = {6, 5};

enum Operation
{
  REST, FORWARD, REVERSE, LEFT, RIGHT,
};

int serial_data = 0, incoming = 0;
int flag = 1;

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < NUM_PINS; i++)
  {
    pinMode(HIGH_POWER[i], OUTPUT);
    //    Serial.println(HIGH_POWER[i], DEC);
    pinMode(HIGH_POWER[i + 2], OUTPUT);
    //    Serial.println(HIGH_POWER[i+2], DEC);
    pinMode(FRONT_LEFT[i], OUTPUT);
    pinMode(FRONT_RIGHT[i], OUTPUT);
    pinMode(REAR_LEFT[i], OUTPUT);
    pinMode(REAR_RIGHT[i], OUTPUT);

    digitalWrite(HIGH_POWER[i], HIGH);
    digitalWrite(HIGH_POWER[i + 2], HIGH);
  }
  pinMode(HIGH_POWER[4], OUTPUT);
  digitalWrite(HIGH_POWER[4], HIGH);  
}

void driveMotor(const int *motorPins, int delay_msec, bool forward)
{
  int val = LOW;
  if (forward)
    val = HIGH;
  digitalWrite(motorPins[0], val);
  digitalWrite(motorPins[1], !val);
  //  delay(delay_msec);
}

void moveForward()
{
  //  driveMotor(FRONT_LEFT, 100, 1);
  analogWrite(FRONT_LEFT[0], 130);
  digitalWrite(FRONT_LEFT[1], 0);

  driveMotor(FRONT_RIGHT, 100, 1);
  //  driveMotor(REAR_LEFT, 100, 1);
  digitalWrite(REAR_LEFT[0], HIGH);
  analogWrite(REAR_LEFT[1], 0);
  
  driveMotor(REAR_RIGHT, 100, 1);
  delay(100);
}

void moveRight()
{
  for(int i = 0; i < 10; i++)
  {
    //  driveMotor(FRONT_LEFT, 100, 1);
    analogWrite(FRONT_LEFT[0], 130);
    digitalWrite(FRONT_LEFT[1], 0);
  
    driveMotor(FRONT_RIGHT, 100, 0);
    //  driveMotor(REAR_LEFT, 100, 1);
    digitalWrite(REAR_LEFT[0], HIGH);
    analogWrite(REAR_LEFT[1], 0);
    
    driveMotor(REAR_RIGHT, 100, 0);
    delay(95);
  }
}


void moveReverse()
{
  analogWrite(FRONT_LEFT[1], 130);
  digitalWrite(FRONT_LEFT[0], 0);

  driveMotor(FRONT_RIGHT, 100, 0);
  //  driveMotor(REAR_LEFT, 100, 1);
  digitalWrite(REAR_LEFT[1], HIGH);
  analogWrite(REAR_LEFT[0], 0);
  
  driveMotor(REAR_RIGHT, 100, 0);
  delay(100); 
}

void moveLeft()
{
  for(int i = 0; i < 10; i++)
  {
    digitalWrite(FRONT_LEFT[0], LOW);
    analogWrite(FRONT_LEFT[1], 255);
  
    driveMotor(FRONT_RIGHT, 100, 1);
    //  driveMotor(REAR_LEFT, 100, 0);
    digitalWrite(REAR_LEFT[0], LOW);
    digitalWrite(REAR_LEFT[1], HIGH);
    //  digitalWrite(REAR_LEFT[1], LOW);
    //  analogWrite(REAR_LEFT[0], LOW);
    driveMotor(REAR_RIGHT, 100, 1);
    delay(95);
  }
}


void moveDont()
{
  digitalWrite(FRONT_LEFT[0], LOW);
  digitalWrite(FRONT_LEFT[1], LOW);

  digitalWrite(FRONT_RIGHT[0], LOW);
  digitalWrite(FRONT_RIGHT[1], LOW);

  digitalWrite(REAR_LEFT[0], LOW);
  digitalWrite(REAR_LEFT[1], LOW);

  digitalWrite(REAR_RIGHT[0], LOW);
  digitalWrite(REAR_RIGHT[1], LOW);
}

//bool flag = 1;
void loop() 
{
  incoming = Serial.available();
  if(!incoming)
    incoming = Serial.available();
  serial_data = Serial.parseInt();
  switch(serial_data)
  {
    case FORWARD: moveForward();
                            break;
    case REVERSE: moveReverse();
                            break;
    case LEFT: moveLeft();
                            break;
    case RIGHT: moveRight();
                            break;
    default: moveDont();                                                           
  }
//  moveDont();
//  delay(500);
//  flag = 0;
}
