#include "I2Cdev.h"
#include <PID_v1.h>
#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

const int ENA = 5;  
const int IN1 = 7; 
const int ENB = 6;
const int IN3 = 8; 
const int STBY = 3;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
volatile bool mpuInterrupt = false;


double input, output;
double setpoint = 0.2;  
double Kp = 20.0;
double Ki = 1.0;
double Kd = .1;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
double fallLimit = 50.0;

float mpuOffset = -6.5;

void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(STBY, OUTPUT);

  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  mpu.setXAccelOffset(-2990); 
  mpu.setYAccelOffset(-119);
  mpu.setZAccelOffset(1612);
  mpu.setXGyroOffset(43);
  mpu.setYGyroOffset(-7);
  mpu.setZGyroOffset(12);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("MPU DMP prêt !");
  } else {
    Serial.print("Erreur d'initialisation DMP: ");
    Serial.println(devStatus);
    while (1);
  }

  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255, 255); 

}
void loop() {

  if (!dmpReady) return;

  fifoCount = mpu.getFIFOCount();
  if (fifoCount < packetSize) return;

  mpuIntStatus = mpu.getIntStatus();

  mpu.getFIFOBytes(fifoBuffer, packetSize);
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  float pitch = ypr[2] * 180 / M_PI;
  input = -pitch + mpuOffset;

  pid.Compute();

  Serial.print("pitch: ");
  Serial.println(pitch);
  Serial.print("input: ");
  Serial.println(input);
  Serial.print("output: ");
  Serial.println(output);
if (abs(input) > fallLimit) {
    stopMotors();
    Serial.println("Chute détectée !");
}

 else {
    driveMotors(output);
  }
}

void driveMotors(double pidValue) {
  digitalWrite(STBY, HIGH);
  int pwm = min(250, abs(pidValue));
if (pwm < 10) pwm = 0;

  if (pidValue > 0) {
    digitalWrite(IN1, LOW);
    analogWrite(ENA, pwm);
    digitalWrite(IN3, LOW);
    analogWrite(ENB, pwm);
  } else {
    digitalWrite(IN1, HIGH);
    analogWrite(ENA, pwm);
    digitalWrite(IN3, HIGH);
    analogWrite(ENB, pwm);
  }
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(STBY, LOW);
}
