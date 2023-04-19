#include <WC_IMU.h>

WC_IMU WC_IMU;

void setup() {
  pinMode(1, OUTPUT);
  digitalWrite(1, HIGH);
  Serial.begin(115200);
  Serial1.begin(921600,SERIAL_8N1,38,39);
  Serial.print(" STart ");
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
