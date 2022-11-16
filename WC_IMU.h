#ifndef _WC_IMU_h_
#define _WC_IMU_h_

#include "Arduino.h"
#include <Math.h>

class WC_IMU{
public:
    WC_IMU();
    void init();
    void updateICM42688(double aX,double aY,double aZ,double gX,double gY,double gZ,int16_t t);
    void doKalman();
    void doAvg();
    float getKalman(bool isRoll);
    float getPitch();
    float getRoll();
    float getPitchAvg();
    float getRollAvg();
    void setQangle(float Q_angle);
    void setQbias(float Q_bias);
    void setRmeasure(float R_measure);
    void setAvgScale(int AvgScale);
    void setUIthreshold(float UI_Threshold);
    float getUIPitch();
    float getUIRoll();
    float getQangle();
    float getQbias();
    float getRmeasure();

    void readSerialData(unsigned char rc);
    void parseData();
    float RecievedIMUData[4];
    boolean newData = false;
private:
    //Kalman Filter
    float RawPitch, RawRoll;
    float Pitch,Roll;
    float AvgPitch,AvgRoll;
    float PitchA[500],RollA[500];
    int AvgScale;
    float UI_Threshold;
    double dt;
    float Q_angle; // Process noise variance for the accelerometer
    float Q_bias; // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise
    float biasR,biasP;
    float angularvelocityX,angularvelocityY;
    float PP[2][2], PR[2][2];
    int timer,UItimerP,UItimerR;
    float UIPitch[2],UIRoll[2];
    bool UIP, UIR;
    double accX, accY, accZ;
    double gyroX, gyroY, gyroZ;
    int16_t tempRaw;
    bool IMUupdated;
    //Serial Recieve
    const int numChars = 32;
    char receivedChars[32];
};
#endif