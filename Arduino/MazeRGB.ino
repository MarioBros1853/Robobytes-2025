#include "Declarations.h"

void setup() {
  // UART Communications
  openMV.begin(9600);
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
  SensorA.begin(&Serial3);
  SensorB.begin(&Serial2);
  SensorC.begin(&Serial1);

  // i2C Communications
  Wire.begin();
  tcs.begin();

  mpu2.begin();
  mpu2.Set_DMP_Output_Rate_Hz(10);
  mpu2.CalibrateMPU();
  mpu2.load_DMP_Image();
  mpu2.on_FIFO(Print_Values);

  mpu.begin();

  // Calibrate Line Sensor
  for (int i = 0; i < 100; i++) {
    int lecture = analogRead(A1);
    lineSensorCalibration = lecture > 50 && lecture < lineSensorCalibration ? lecture : lineSensorCalibration;
  }
  lineSensorCalibration -= 200;

  // Pins
  pinMode(49, OUTPUT);
  servo.attach(11);
  servo.write(90);
  A.Initialize();
  B.Initialize();
}

void loop() {
  unsigned long ms = millis();
  uint16_t clear, red, green, blue;

  // Lectura de Sensores:
  mpu2.dmp_read_fifo(false);
  mpu.update();
  angle = -mpu.getAngleZ();
  openMVValue = openMV.available() ? openMV.read() : '9';

  // Hacemos un Promedio de 5 lecturas de linea para asegurarnos que haya un color negro.
  for (int i = 0; i < 5; i++)
    TotalSensor += analogRead(A1);

  TotalSensor = TotalSensor / 5;

  // Lectura de Color RGB y conversion HSV:
  tcs.getRawData(&red, &green, &blue, &clear);
  r = red;
  r /= clear;
  g = green;
  g /= clear;
  b = blue;
  b /= clear;
  r *= 256;
  g *= 256;
  b *= 256;
  ColorConverter::RgbToHsv(static_cast<uint8_t>(r), static_cast<uint8_t>(g), static_cast<uint8_t>(b), hue, saturation, value);

  dist_a = SensorA.getData(dist_a) ? dist_a : 700;
  dist_b = SensorB.getData(dist_b) ? dist_b : 700;
  dist_c = SensorC.getData(dist_c) ? dist_c : 700;

  if (openMVValue != '9' && !turning) {
    A.Move(-50);
    B.Move(-50);
    delay(200);
    A.Move(0);
    B.Move(0);
    delay(300);

    int n = openMVValue == '0' ? 0 : openMVValue == '1' ? 1
                                   : openMVValue == '2' ? 2
                                                        : 3;
    for (int i = 0; i < n; i++) {
      digitalWrite(49, HIGH);
      delay(100);

      digitalWrite(49, LOW);
      delay(100);

      servoAngle = servoAngle == 180 ? 0 : 180;

      if (servoAngle == 0) {
        for (int pos = 0; pos <= 180; pos += 1) {
          servo.write(pos);
          delay(2);
        }
      } else {
        for (int pos = 180; pos >= 0; pos -= 1) {
          servo.write(pos);
          delay(1);
        }
      }
    }

    turning = true;
    timer = ms + 2000;
  }

  // Si hay pared en frente determinar a donde girara y detener el robot.
  else if ((dist_c > 0 && dist_c < 13 && dist_a < 700 && dist_b < 700 && mpu.getAngleX() < 7) && !turning) {
    angleSetpoint += dist_a > 20 && dist_b > 20 ? -90 : (dist_a <= 4 && dist_b <= 4) ? 90
                                                      : dist_a > dist_b              ? -90
                                                                                     : 90;

    turning = true;
    A.Move(0);
    B.Move(0);
    delay(300);
    timer = ms + 1500;
  }

  // Si detcta color azil detener por 5 segundos.
  else if (isBlue() && !turning && !blueStop) {
    A.Move(50);
    B.Move(50);
    delay(400);
    turning = true;
    blueStop = true;
    timer = ms + 5000;
  }

  // Retroceder si hay deteccion de negro y girar hacia un lugar libre.
  else if (TotalSensor < lineSensorCalibration && !turning) {
    angleSetpoint += dist_a > 20 && dist_b > 20 ? -90 : (dist_a <= 4 && dist_b <= 4) ? 90
                                                      : dist_a > dist_b              ? -90
                                                                                     : 90;
    turning = true;
    timer = ms + 1500;
  }

  // Control System for wall centering.
  dist_a = dist_a >= 10 ? 4 : dist_a;
  dist_b = dist_b >= 10 ? 4 : dist_b;
  int err = dist_a - dist_b;
  int k = 7;
  int u = err * k;

  // Resetear el timer una vez haya terminado.
  turning = (turning && ms > timer) ? false : turning;
  blueStop = !isBlue() ? false : blueStop;

  // Si la variable turning esta encendida el robot se detendra y corregira los valores necesarios
  // Esta funcion se utiliza para realizar los giros o retroceder por la cantidad de tiempo que se le indique en las condiciones con variable timer.
  if (turning && ms < timer) {
    // Control system for gyroscope values.
    int eb = angleSetpoint - angle;
    int kb = 4;
    int ub = eb * kb;

    // Retroceder si el color negro lo sigue detectando si no gira al angulo de la brujula.
    if (TotalSensor < lineSensorCalibration) {
      A.Move(-100);
      B.Move(-100);
    } else {
      A.Move(+ub);
      B.Move(-ub);
    }
  } else {
    // Control system for gyroscope values.
    int eb = angleSetpoint - angle;
    int kb = 2;
    int ub = eb * kb;

    int vel = mpu.getAngleX() < 13 ? 45 : 80;
    int vel_a = vel - u + ub;
    int vel_b = vel + u - ub;
    A.Move(vel_a);
    B.Move(vel_b);
  }

  //printData();
}