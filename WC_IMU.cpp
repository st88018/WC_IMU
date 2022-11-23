#include "WC_IMU.h"

WC_IMU::WC_IMU(){
    Q_angle = 0.001f;
    Q_bias = 0.003f;
    R_measure = 0.03f;

    RawXAng = 0.0f;
    RawYAng = 0.0f;
    YAng = 0.0f;
    XAng = 0.0f;
    biasY = 0.0f;
    biasX = 0.0f;
    AvgScale = 500;
    UI_Threshold = 0.75;

    PZ[0][0] = 0.0f;
    PZ[0][1] = 0.0f;
    PZ[1][0] = 0.0f;
    PZ[1][1] = 0.0f;

    PY[0][0] = 0.0f;
    PY[0][1] = 0.0f;
    PY[1][0] = 0.0f;
    PY[1][1] = 0.0f;

    PX[0][0] = 0.0f;
    PX[0][1] = 0.0f;
    PX[1][0] = 0.0f;
    PX[1][1] = 0.0f;
};

void WC_IMU::init(){
    if(RawXAng ==0 || RawYAng == 0 || RawZAng == 0){
        Serial.println("ICM42688 not begin!");
    }
    XAng = RawXAng;
    YAng = RawYAng;
    ZAng = RawZAng;
    timer = micros();
    UItimerX = millis();
    UItimerY = millis();
    UItimerZ = millis();
};

void WC_IMU::updateICM42688(double acc[3],double gyro[3]){
    for(Gdir = 0; Gdir < 6; Gdir++){
        if(Gdir<3){
            if(acc[Gdir]>500) break;
        }else{
            if(acc[Gdir-3]<-500) break;
        }
    }
    // Serial.print("Acc is at: ");Serial.println(Gdir);
    RawXAng = atan2(acc[1], acc[2]) * RAD_TO_DEG; // deg
    RawYAng = atan2(-acc[0], acc[2]) * RAD_TO_DEG; // deg
    RawZAng = atan2(-acc[0], -acc[1]) * RAD_TO_DEG; // deg
    angularvelocityX = gyro[0]; // deg/s
    angularvelocityY = gyro[1]; // deg/s
    angularvelocityZ = gyro[2]; // deg/s
    
    IMUupdated = true;
};

void WC_IMU::doKalman(){
    if(IMUupdated){
        dt = (double)(micros()-timer) /1000000;
        timer = micros();
        getKalman();
        doAvg();
        IMUupdated = false;
    }else{
        // Serial.print("No IMU");
    }
};

void WC_IMU::getKalman(){

    XAng += dt * (angularvelocityX-biasX);
    PX[0][0] += dt * (dt*PX[1][1] - PX[0][1] - PX[1][0] + Q_angle);
    PX[0][1] -= dt * PX[1][1];
    PX[1][0] -= dt * PX[1][1];
    PX[1][1] += Q_bias * dt;
    float Sx = PX[0][0] + R_measure;
    float Kx[2]; // Kalman gain
    Kx[0] = PX[0][0] / Sx;
    Kx[1] = PX[1][0] / Sx;
    float yx = RawXAng - XAng; // Angle difference
    XAng += Kx[0] * yx;
    biasX += Kx[1] * yx;
    float Px00_temp = PX[0][0];
    float Px01_temp = PX[0][1];
    PX[0][0] -= Kx[0] * Px00_temp;
    PX[0][1] -= Kx[0] * Px01_temp;
    PX[1][0] -= Kx[1] * Px00_temp;
    PX[1][1] -= Kx[1] * Px01_temp;

    YAng += dt * (angularvelocityY-biasY);
    PY[0][0] += dt * (dt*PY[1][1] - PY[0][1] - PY[1][0] + Q_angle);
    PY[0][1] -= dt * PY[1][1];
    PY[1][0] -= dt * PY[1][1];
    PY[1][1] += Q_bias * dt;
    float Sy = PY[0][0] + R_measure;
    float Ky[2]; // Kalman gain
    Ky[0] = PY[0][0] / Sy;
    Ky[1] = PY[1][0] / Sy;
    float yy = RawYAng - YAng; // Angle difference
    YAng += Ky[0] * yy;
    biasY += Ky[1] * yy;
    float Py00_temp = PY[0][0];
    float Py01_temp = PY[0][1];
    PY[0][0] -= Ky[0] * Py00_temp;
    PY[0][1] -= Ky[0] * Py01_temp;
    PY[1][0] -= Ky[1] * Py00_temp;
    PY[1][1] -= Ky[1] * Py01_temp;

    ZAng += dt * (angularvelocityZ-biasZ);
    PZ[0][0] += dt * (dt*PZ[1][1] - PZ[0][1] - PZ[1][0] + Q_angle);
    PZ[0][1] -= dt * PZ[1][1];
    PZ[1][0] -= dt * PZ[1][1];
    PZ[1][1] += Q_bias * dt;
    float Sz = PZ[0][0] + R_measure;
    float Kz[2]; // Kalman gain
    Kz[0] = PZ[0][0] / Sz;
    Kz[1] = PZ[1][0] / Sz;
    float yz = RawZAng - ZAng; // Angle difference
    ZAng += Kz[0] * yz;
    biasZ += Kz[1] * yz;
    float Pz00_temp = PZ[0][0];
    float Pz01_temp = PZ[0][1];
    PZ[0][0] -= Kz[0] * Pz00_temp;
    PZ[0][1] -= Kz[0] * Pz01_temp;
    PZ[1][0] -= Kz[1] * Pz00_temp;
    PZ[1][1] -= Kz[1] * Pz01_temp;
}; 

void WC_IMU::doAvg(){  
    float XSum = 0;
    float YSum = 0;
    float ZSum = 0;
    for(int i=0; i<AvgScale-1; i++){
        XA[i] = XA[i+1];
        YA[i] = YA[i+1];
        ZA[i] = ZA[i+1];
        XSum += XA[i];
        YSum += YA[i];
        ZSum += ZA[i];
    }
    XA[AvgScale-1] = XAng;
    YA[AvgScale-1] = YAng;
    ZA[AvgScale-1] = ZAng;
    XSum += XAng;
    YSum += YAng;
    ZSum += ZAng;

    AvgXAng = XSum/AvgScale;
    AvgYAng = YSum/AvgScale;
    AvgZAng = ZSum/AvgScale;
}

float WC_IMU::getUIX(){
    if(AvgXAng>UIX[0]){
        UIX[0] = AvgXAng;
    }
    if(AvgXAng<UIX[1]){
        UIX[1] = AvgXAng;
    }
    if(uix){
        if((abs(UIX[1]-UIX[0]))<UI_Threshold){
            // Serial.print("[0]: "); Serial.print(UIPitch[0],4);  Serial.print("\t");
            // Serial.print("[1]: "); Serial.print(UIPitch[1],4);  Serial.print("\t");
            return ((UIX[0]+UIX[1])/2);
        }else{
            UItimerX = millis();
            UIX[0] = AvgXAng;
            UIX[1] = AvgXAng;
            uix = false;
            return AvgXAng;
        }
    }else{
        if (millis() - UItimerX > 1000){
            uix = true;
        }
        return AvgXAng;
    }
}

float WC_IMU::getUIY(){
    if(AvgYAng>UIY[0]){
        UIY[0] = AvgYAng;
    }
    if(AvgYAng<UIY[1]){
        UIY[1] = AvgYAng;
    }
    if(uiy){
        if((abs(UIY[1]-UIY[0]))<UI_Threshold){
            return ((UIY[1]+UIY[0])/2);
        }else{
            UItimerY = millis();
            UIY[0] = AvgYAng;
            UIY[1] = AvgYAng;
            uiy = false;
            return AvgYAng;
        }
    }else{
        if (millis() - UItimerY > 1000){
            uiy = true;
        }
        return AvgYAng;
    }
}

float WC_IMU::getUIZ(){
    if(AvgYAng>UIZ[0]){
        UIZ[0] = AvgYAng;
    }
    if(AvgYAng<UIZ[1]){
        UIZ[1] = AvgZAng;
    }
    if(uiz){
        if((abs(UIZ[1]-UIZ[0]))<UI_Threshold){
            return ((UIZ[1]+UIZ[0])/2);
        }else{
            UItimerZ = millis();
            UIZ[0] = AvgZAng;
            UIZ[1] = AvgZAng;
            uiz = false;
            return AvgZAng;
        }
    }else{
        if (millis() - UItimerZ > 1000){
            uiz = true;
        }
        return AvgZAng;
    }
}

void WC_IMU::setUIthreshold(float UI_Threshold) { this->UI_Threshold = UI_Threshold; };
void WC_IMU::setQangle(float Q_angle) { this->Q_angle = Q_angle; };
void WC_IMU::setQbias(float Q_bias) { this->Q_bias = Q_bias; };
void WC_IMU::setRmeasure(float R_measure) { this->R_measure = R_measure; };
void WC_IMU::setAvgScale(int AvgScale) { this->AvgScale = AvgScale; };
float WC_IMU::getX() { return this->XAng; };
float WC_IMU::getY() { return this->YAng; };
float WC_IMU::getZ() { return this->ZAng; };
float WC_IMU::getXAvg() { return this->AvgXAng; };
float WC_IMU::getYAvg() { return this->AvgYAng; };
float WC_IMU::getZAvg() { return this->AvgZAng; };
float WC_IMU::getQangle() { return this->Q_angle; };
float WC_IMU::getQbias() { return this->Q_bias; };
float WC_IMU::getRmeasure() { return this->R_measure; };

void WC_IMU::readSerialData(unsigned char rc){
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';

    if (recvInProgress == true) {
        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }else {
            receivedChars[ndx] = '\0'; // terminate the string
            recvInProgress = false;
            ndx = 0;
            newData = true;
        }
    }else if (rc == startMarker) {
        recvInProgress = true;
    }
}

void WC_IMU::parseData() {      // split the data into its parts
    if(newData){
        char * strtokIndx; // this is used by strtok() as an index

        strtokIndx = strtok(receivedChars,",");      // get the first part - the string
        if(strtokIndx != NULL) RecievedIMUData[0] = atof(strtokIndx);
    
        strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
        if(strtokIndx != NULL) RecievedIMUData[1] = atof(strtokIndx);     // convert this part to an integer

        strtokIndx = strtok(NULL, ",");
        if(strtokIndx != NULL) RecievedIMUData[2] = atof(strtokIndx);     // convert this part to a float

        strtokIndx = strtok(NULL, ",");
        if(strtokIndx != NULL) RecievedIMUData[3] = atof(strtokIndx);     // convert this part to a float

        strtokIndx = strtok(NULL, ",");
        if(strtokIndx != NULL) RecievedIMUData[4] = atof(strtokIndx);     // convert this part to a float

        strtokIndx = strtok(NULL, ",");
        if(strtokIndx != NULL) RecievedIMUData[5] = atof(strtokIndx);     // convert this part to a float
        
        strtokIndx = strtok(NULL, ",");
        if(strtokIndx != NULL) RecievedIMUData[6] = atof(strtokIndx);     // convert this part to a float

        strtokIndx = strtok(NULL, ",");
        if(strtokIndx != NULL) RecievedIMUData[7] = atof(strtokIndx);     // convert this part to a float
        
        newData = false;
    }
}