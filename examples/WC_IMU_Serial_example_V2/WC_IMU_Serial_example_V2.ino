#include <WC_IMU.h>

WC_IMU WC_IMU;

void setup() {
  pinMode(1, OUTPUT);
  digitalWrite(1, HIGH);
  Serial.begin(115200);
  Serial1.begin(921600,SERIAL_8N1,38,39);
}

void loop() {
  while (Serial1.available() && !WC_IMU.newData) 
  {
    WC_IMU.readSerialData(Serial1.read());
  }
  WC_IMU.parseData();
  showData();
}
void showData() {
  Serial.print("KalAvgX: ");
  Serial.print(WC_IMU.RecievedIMUData[0],4);
  Serial.print(" KalAvgY: ");
  Serial.print(WC_IMU.RecievedIMUData[1],4);
  Serial.print(" KalAvgZ: ");
  Serial.print(WC_IMU.RecievedIMUData[2],4);
  Serial.print(" KalUIX: ");
  Serial.print(WC_IMU.RecievedIMUData[3],4);
  Serial.print(" KalUIY: ");
  Serial.print(WC_IMU.RecievedIMUData[4],4);
  Serial.print(" KalUIZ: ");
  Serial.print(WC_IMU.RecievedIMUData[5],4);
  Serial.print(" Temp: ");
  Serial.print(WC_IMU.RecievedIMUData[6],2);
  Serial.print(" Gdir: ");
  Serial.println(WC_IMU.RecievedIMUData[7]);
}
