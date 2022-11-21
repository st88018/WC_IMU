#include <WC_IMU.h>

WC_IMU WC_IMU;

void setup() {
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
  Serial.print("KalAvgRoll: ");
  Serial.print(WC_IMU.RecievedIMUData[0],4);
  Serial.print(" KalAvgPitch: ");
  Serial.print(WC_IMU.RecievedIMUData[1],4);
  Serial.print(" KalUIRoll: ");
  Serial.print(WC_IMU.RecievedIMUData[2],4);
  Serial.print(" KalUIPitch: ");
  Serial.print(WC_IMU.RecievedIMUData[3],4);
  Serial.print(" Temp: ");
  Serial.print(WC_IMU.RecievedIMUData[4],2);
  Serial.print(" Gdir: ");
  Serial.println(WC_IMU.RecievedIMUData[5]);
}
