#include <WC_IMU.h>
#include <WC_ICM42688.h>

#define RESTRICT_PITCH

WC_IMU kalman;
WC_IMU kalmanLN;

/* IMU Data */
WC_ICM42688_SPI ICM42688(48);
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

void setup() {
  pinMode(1, OUTPUT);
  digitalWrite(1, HIGH);
  Serial.begin(115200);
  SPI.begin(21, 10, 47, 48);
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
  kalman.init();
  kalmanLN.init();
  kalmanLN.setQbias(0);
  
}

void loop() {
  ICM42688updateAll();
  kalman.doKalman();
  kalmanLN.doKalman();
  Serial.print(kalman.getPitch(),4); Serial.print("\t");
  Serial.print(kalman.getPitchAvg(),4); Serial.print("\t");
  Serial.print(kalmanLN.getPitch(),4); Serial.print("\t");
  Serial.print("\t");
  Serial.print(kalman.getRoll(),4); Serial.print("\t");
  Serial.print(kalman.getRollAvg(),4); Serial.print("\t");
  Serial.print(kalmanLN.getRoll(),4); Serial.print("\t");
  Serial.print("\r\n");
  // delay(20);
}

void ICM42688updateAll(){
  accX = ICM42688.getAccelDataX();
  accY = ICM42688.getAccelDataY();
  accZ = ICM42688.getAccelDataZ();
  tempRaw = ICM42688.getTemperature();
  gyroX = ICM42688.getGyroDataX();
  gyroY = ICM42688.getGyroDataY();
  gyroZ = ICM42688.getGyroDataZ();
  kalman.updateICM42688(accX,accY,accZ,gyroX,gyroY,gyroZ,tempRaw);
  kalmanLN.updateICM42688(accX,accY,accZ,gyroX,gyroY,gyroZ,tempRaw);
}