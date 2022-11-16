#include <WC_Kalman.h>
#include <WC_ICM42688.h>


#define RESTRICT_PITCH

WC_Kalman kalmanX;
WC_Kalman kalmanY;
WC_Kalman kalmanXLN;
WC_Kalman kalmanYLN;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
double kalAngleXLN, kalAngleYLN; // Calculated angle using a low noise Kalman filter

uint32_t timer;

WC_ICM42688_SPI ICM42688(48);

void setup() {
  pinMode(1, OUTPUT);
  digitalWrite(1, LOW);
  Serial.begin(115200);
  kalmanXLN.initLN();
  kalmanYLN.initLN();

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

  /* Set kalman and gyro starting angle */
  accX = ICM42688.getAccelDataX();
  accY = ICM42688.getAccelDataY();
  accZ = ICM42688.getAccelDataZ();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  kalmanXLN.setAngle(roll); // Set starting angle
  kalmanYLN.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}

void loop() {

  accX = ICM42688.getAccelDataX();
  accY = ICM42688.getAccelDataY();
  accZ = ICM42688.getAccelDataZ();
  tempRaw = ICM42688.getTemperature();;
  gyroX = ICM42688.getGyroDataX();
  gyroY = ICM42688.getGyroDataY();
  gyroZ = ICM42688.getGyroDataZ();;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    kalmanXLN.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;    
    gyroXangle = roll;
  } else{
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
    kalAngleXLN = kalmanXLN.getAngle(roll, gyroXrate, dt);
  }
    

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  kalAngleYLN = kalmanYLN.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    kalmanYLN.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else{
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
    kalAngleYLN = kalmanYLN.getAngle(pitch, gyroYrate, dt);
  }
    
  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);
  kalAngleXLN = kalmanXLN.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.97 * (compAngleX + gyroXrate * dt) + 0.03 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.97 * (compAngleY + gyroYrate * dt) + 0.03 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");

  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");

  Serial.print("\t");
#endif

//  Serial.print(roll); Serial.print("\t");
//  Serial.print(gyroXangle); Serial.print("\t");
//  Serial.print(compAngleX,4); Serial.print("\t");
 Serial.print(kalAngleX,4); Serial.print("\t");
 Serial.print(kalAngleXLN,4); Serial.print("\t");
 Serial.print(dt,6); Serial.print("\t");

 Serial.print("\t");

//  Serial.print(pitch); Serial.print("\t");
//  Serial.print(gyroYangle); Serial.print("\t");
//  Serial.print(compAngleY,4); Serial.print("\t");
//  Serial.print(kalAngleY,4); Serial.print("\t");
//  Serial.print(kalAngleYLN,4); Serial.print("\t");

#if 0 // Set to 1 to print the temperature
  Serial.print("\t");

  double temperature = (double)tempRaw / 340.0 + 36.53;
  Serial.print(temperature); Serial.print("\t");
#endif

  Serial.print("\r\n");
  delay(2);
}
