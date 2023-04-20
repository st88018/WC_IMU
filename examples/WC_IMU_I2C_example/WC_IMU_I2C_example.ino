#include <WC_IMU.h>
#include <Wire.h>

#define SLAVE_ADDR 0x09

WC_IMU WC_IMU;

void setup() {
  pinMode(1, OUTPUT);
  digitalWrite(1, HIGH);
  Serial.begin(115200);
  Wire.begin();
  Serial.print(" End setup ");
}

void loop() {
  delay(50);
  WC_IMU.updateI2CData(6);  // mode 1: angles 2: UI 3: acc 4: temp 5: GDir 6: all
  showData();
}

void showData() {
  Serial.print("AvgX: ");
  Serial.print(WC_IMU.RecievedIMUData[0],4);
  Serial.print(" AvgY: ");
  Serial.print(WC_IMU.RecievedIMUData[1],4);
  Serial.print(" AvgZ: ");
  Serial.print(WC_IMU.RecievedIMUData[2],4);
  Serial.print(" UIX: ");
  Serial.print(WC_IMU.RecievedIMUData[3],4);
  Serial.print(" UIY: ");
  Serial.print(WC_IMU.RecievedIMUData[4],4);
  Serial.print(" UIZ: ");
  Serial.print(WC_IMU.RecievedIMUData[5],4);
  Serial.print(" ACC0: ");
  Serial.print(WC_IMU.RecievedIMUData[6],4);
  Serial.print(" ACC1: ");
  Serial.print(WC_IMU.RecievedIMUData[7],4);
  Serial.print(" ACC2: ");
  Serial.print(WC_IMU.RecievedIMUData[8],4);
  Serial.print(" temp: ");
  Serial.print(WC_IMU.RecievedIMUData[9],1);
  Serial.print(" GDir: ");
  Serial.println(WC_IMU.RecievedIMUData[10]);
}
