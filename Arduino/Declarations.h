# ifndef DECLARATIONS_H
# define DECLARATIONS_H

// Librerias utilizadas en el codigo.
#include "Wire.h"
#include <TFMPlus.h>
#include <MPU6050_light.h>
#include "Adafruit_TCS34725.h"
#include <ColorConverterLib.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include "Simple_MPU6050.h"

// Clase de Motor
// pin = Pin de Direccion del Motor a controlar.
// en  = Pin de control de velocidad PWM.
class Motor {
private:
  int pin = 0;
  int en = 0;
public:
  Motor(int _pin, int _en) {
    pin = _pin;
    en = _en;
  }

  void Initialize() {
    pinMode(pin, OUTPUT);
    pinMode(en, OUTPUT);
  }

  void Move(int vel, int maxVel = 255) {
    vel = constrain(vel, -maxVel, maxVel);
    analogWrite(en, abs(vel));
    digitalWrite(pin, vel > 0 ? HIGH : LOW);
  }
};

// Clases:
TFMPlus SensorA;
TFMPlus SensorB;
TFMPlus SensorC;
MPU6050 mpu(Wire);
Motor A(3, 6);
Motor B(4, 5);
Servo servo;
Simple_MPU6050 mpu2;
SoftwareSerial openMV(12, 45);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

// Variables Globales:
bool turning = false;
bool blueStop = false;
bool colorStop = false;
bool leftSide = false;
long timer = 0;
long correctionTimer = 0;
int angleSetpoint = 0;
int lineSensor = 0;
int TotalSensor = 0;
int lineSensorCalibration = 999;
int servoAngle = 180;
char openMVValue;
double hue, saturation, value;
float angle = 0;
float angle2 = 0;
float r, g, b;
uint16_t clear, red, green, blue;

int16_t dist_a = 0;
int16_t dist_b = 0;
int16_t dist_c = 0; 

bool isBlue() {
  return ((hue * 360) > 170 && (hue * 360) < 270);
}

void Print_Values (int16_t *gyro, int16_t *accel, int32_t *quat) {
  Quaternion q;
  VectorFloat gravity;
  float ypr[3] = { 0, 0, 0 };
  float xyz[3] = { 0, 0, 0 };
  mpu2.GetQuaternion(&q, quat);
  mpu2.GetGravity(&gravity, &q);
  mpu2.GetYawPitchRoll(ypr, &q, &gravity);
  mpu2.ConvertToDegrees(ypr, xyz);

  angle2 = xyz[0];
}

// Funcion para imprimir todos los valores del Robot y poder monitorear.
void printData() {
  Serial.print("Sensor Izquierda: ");
  Serial.print(dist_a);
  Serial.print("cm | Sensor Derecha: ");
  Serial.print(dist_b);
  Serial.print("cm | Sensor Frente: ");
  Serial.print(dist_c);
  Serial.print("cm | Setpoint Angle: ");
  Serial.print(angleSetpoint);
  Serial.print(" | Angle: ");
  Serial.print(angle);
  Serial.print(" | Inclination: ");
  Serial.print(mpu.getAngleX());
  Serial.print(" | White Floor: ");
  Serial.print(lineSensorCalibration);
  Serial.print(" | Floor: ");
  Serial.print(TotalSensor);
  Serial.print(" | OpenMV: ");
  Serial.print(openMVValue);
  Serial.println();
}

# endif