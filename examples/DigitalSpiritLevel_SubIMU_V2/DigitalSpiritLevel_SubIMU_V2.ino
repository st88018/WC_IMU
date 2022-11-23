#include <WC_IMU.h>
#include <WC_ICM42688.h>
#include <JY901.h>
#include "SPI.h"

TaskHandle_t Core1Task;
TaskHandle_t Core2Task;

WC_IMU kalman;



WC_ICM42688_SPI ICM42688(48);

double acc[3], gyro[3];
float tempRaw;
float JY901X,JY901Y,JY901Z;
float KalX,KalY,KalZ,KalAvgX,KalAvgY,KalAvgZ, KalUIX, KalUIY, KalUIZ;
long loggertimer,plottertimer,kalmaintimer[2];
int logercounter;
String dataString;
bool NoSD = true;

void setup() {
  xTaskCreatePinnedToCore( Core1code,"Core1Task",100000,NULL,1,&Core1Task,0);
  xTaskCreatePinnedToCore( Core2code,"Core2Task",100000,NULL,1,&Core2Task,1);
  pinMode(1, OUTPUT);
  digitalWrite(1, HIGH);
  delay(1000);
  Serial.begin(115200);
  Serial1.begin(921600,SERIAL_8N1,38,39);
}
void Core1code( void * pvParameters ){
  SPI.begin(21,10,47,48);
  int status;
  while((status =ICM42688.begin()) !=0){
    if(status == -1){
      Serial.println("bus data access error");
    } else 
      Serial.println("Chip versions do not match");
    delay(1000);
  }
  Serial.println("ICM42688 begin success!!!");
  ICM42688.setODRAndFSR(ALL,ODR_1KHZ,FSR_3);
  ICM42688.startTempMeasure();
  ICM42688.startGyroMeasure(LN_MODE);
  ICM42688.startAccelMeasure(LN_MODE);
  delay(1000); // Wait for sensor to stabilize
  ICM42688updateAll();
  for(;;){
    if (millis()-plottertimer > 10){
      plottertimer = millis();
      plotter();
      OutputData();
    }
    ICM42688updateAll();
    delay(1);
  }
}
void Core2code( void * pvParameters ){
  kalman.init();
  for(;;){
    kalman.doKalman();
    KalX = kalman.getX();
    KalY = kalman.getY();
    KalZ = kalman.getZ(); 
    KalAvgX = kalman.getXAvg();
    KalAvgY = kalman.getYAvg();
    KalAvgZ = kalman.getZAvg();
    KalUIX = kalman.getUIX();
    KalUIY = kalman.getUIY();
    KalUIZ = kalman.getUIZ();
  }
}
void loop(){}
void plotter(){
  Serial.print(KalX,4); Serial.print("\t");
  Serial.print(KalY,4); Serial.print("\t");
  Serial.print(KalZ,4); Serial.print("\t");
  Serial.print("\t");
  Serial.print(KalUIX,4); Serial.print("\t");
  Serial.print(KalUIY,4); Serial.print("\t");
  Serial.print(KalUIZ,4); Serial.print("\t");
  Serial.print("\t");
  Serial.print(acc[0]); Serial.print("\t");
  Serial.print(acc[1]); Serial.print("\t");
  Serial.print(acc[2]); Serial.print("\t");
  Serial.print("\t");
  Serial.print(gyro[0]); Serial.print("\t");
  Serial.print(gyro[1]); Serial.print("\t");
  Serial.print(gyro[2]); Serial.print("\t");
  Serial.print("\t");
  // Serial.print(kalmaintimer[1]-kalmaintimer[0]);
  Serial.print("\r\n");
} 
void ICM42688updateAll(){
  double ICM42688Temp;
  if((ICM42688Temp=ICM42688.getTemperature())!= 0){
    tempRaw = ICM42688Temp;
  }
  if((ICM42688Temp=ICM42688.getAccelDataX())  != 0){
    acc[0] = ICM42688Temp;
  }
  if((ICM42688Temp=ICM42688.getAccelDataY())  != 0){
    acc[1] = ICM42688Temp;
  }
  if((ICM42688Temp=ICM42688.getAccelDataZ())  != 0){
    acc[2] = ICM42688Temp;
  }
  if((ICM42688Temp=ICM42688.getGyroDataX()) != 0){
    gyro[0] = ICM42688Temp;
  }
  if((ICM42688Temp=ICM42688.getGyroDataY()) != 0){
    gyro[1] = ICM42688Temp;
  }
  if((ICM42688Temp=ICM42688.getGyroDataZ()) != 0){
    gyro[2] = ICM42688Temp;
  }
  kalman.updateICM42688(acc,gyro);
}
void OutputData(){
  dataString += "<";
  dataString += String(KalAvgX,4);
  dataString += ",";
  dataString += String(KalAvgY,4);
  dataString += ",";
  dataString += String(KalAvgZ,4);
  dataString += ",";
  dataString += String(KalUIX,4);
  dataString += ",";
  dataString += String(KalUIY,4);
  dataString += ",";
  dataString += String(KalUIZ,4);
  dataString += ",";
  dataString += String(tempRaw,2);
  dataString += ",";
  dataString += String(kalman.Gdir);
  dataString += ">";
  Serial1.print(dataString);
  dataString = "";
}