#ifndef _WC_IMU_h_
#define _WC_IMU_h_

#include "Arduino.h"
#include <Math.h>
#include "Wire.h"

class WC_IMU{
public:
    //Kalman Filter
    WC_IMU(TwoWire *wire = &Wire);
    void init();
    void updateICM42688(double acc[3],double gyro[3]);
    void doKalman();
    void doAvg();
    void getKalman();
    float getX();
    float getY();
    float getZ();
    float getXAvg();
    float getYAvg();
    float getZAvg();
    void setQangle(float Q_angle);
    void setQbias(float Q_bias);
    void setRmeasure(float R_measure);
    void setAvgScale(int AvgScale);
    void setUIthreshold(float UI_Threshold);
    float getUIX();
    float getUIY();
    float getUIZ();
    float getQangle();
    float getQbias();
    float getRmeasure();
    float RawXAng, RawYAng, RawZAng;
    //Serial Recieve
    void readSerialData(unsigned char rc);
    void parseData();
    float RecievedIMUData[11];
    boolean newData = false;
    int Gdir;
    //I2C Recieve
    void updateI2CData(uint8_t mode = 1); // mode 1: angles 2: UI 3: acc 4: time

private:
    //Kalman Filter
    float XAng, YAng, ZAng;
    float AvgXAng,AvgYAng,AvgZAng;
    float XA[500],YA[500],ZA[500];
    int AvgScale;
    float UI_Threshold;
    double dt;
    float Q_angle; // Process noise variance for the accelerometer
    float Q_bias; // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise
    float biasX, biasY, biasZ;
    float angularvelocityX, angularvelocityY, angularvelocityZ;
    float PX[2][2], PY[2][2], PZ[2][2];
    int timer, UItimerX, UItimerY, UItimerZ;
    float UIX[2],UIY[2],UIZ[2];
    bool uix, uiy, uiz;
    double accX, accY, accZ;
    double gyroX, gyroY, gyroZ;
    bool IMUupdated;
    //Serial Recieve
    const int numChars = 128;
    char receivedChars[128];
    //I2C Recieve
    uint8_t  _address      = 0x09;
    uint8_t  _error        = 0;
    TwoWire*  _wire;
    float readRegfloat(uint8_t reg);
    uint8_t readReg(uint8_t reg);
};
#endif