#include <BleMouse.h>
#include <MPU6050.h>
#include <Wire.h>
BleMouse bleMouse;
MPU6050 Giroscopio;
int16_t ax,ay,az,gx,gy,gz;
void setup() {
  Serial.begin(115200);
  Wire.begin();
  bleMouse.begin();
  Giroscopio.initialize();
  Giroscopio.CalibrateGyro(7);
  delay(20);
}

void loop() {
  Giroscopio.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
  Serial.println("x:");
  Serial.println(gx);
  Serial.println("y:");
  Serial.println(gz);
  delay(300);
}
