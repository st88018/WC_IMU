#include "WC_IMU.h"

WC_IMU::WC_IMU(){
    Q_angle = 0.001f;
    Q_bias = 0.003f;
    R_measure = 0.03f;

    RawRoll = 0.0f;
    RawPitch = 0.0f;
    Pitch = 0.0f;
    Roll = 0.0f;
    biasP = 0.0f;
    biasR = 0.0f;
    AvgScale = 50;
    UI_Threshold = 0.05;

    PP[0][0] = 0.0f;
    PP[0][1] = 0.0f;
    PP[1][0] = 0.0f;
    PP[1][1] = 0.0f;

    PR[0][0] = 0.0f;
    PR[0][1] = 0.0f;
    PR[1][0] = 0.0f;
    PR[1][1] = 0.0f;
};

void WC_IMU::init(bool yesAdaptiveZ){
    if(RawRoll ==0 || RawPitch == 0){
        Serial.println("ICM42688 not begin!");
    }
    Roll = RawRoll;
    Pitch = RawPitch;
    timer = micros();
    UItimerP = millis();
    UItimerR = millis();
    AdaptiveZ = yesAdaptiveZ;
};

void WC_IMU::updateICM42688(double acc[3],double gyro[3],int16_t t){
    for(Gdir = 0; Gdir < 6; Gdir++){
        if(Gdir<3){
            if(acc[Gdir]>500) break;
        }else{
            if(acc[Gdir-3]<-500) break;
        }
    }
    // Serial.print("Acc is at: ");Serial.println(Gdir);
    // Now restrict Roll to ±90deg
    RawRoll = atan(acc[1] / sqrt(acc[0] * acc[0] + acc[2] * acc[2])) * RAD_TO_DEG; // deg
    RawPitch = atan2(-acc[0], acc[2]) * RAD_TO_DEG; // deg
    angularvelocityX = gyro[0]; // deg/s
    angularvelocityY = gyro[1]; // deg/s
    
    IMUupdated = true;
};

void WC_IMU::doKalman(){
    if(IMUupdated){
        dt = (double)(micros()-timer) /1000000;
        timer = micros();
        if((RawPitch < -90 && Pitch >90) || (RawPitch > 90 && Pitch < -90)){
            Pitch = RawPitch;
        }else{
            Pitch = getKalman(false);
        }
        if (abs(Pitch) > 90 ){
            angularvelocityX = -angularvelocityX;
        }
        Roll = getKalman(true);
        doAvg();
        IMUupdated = false;
    }else{
        // Serial.print("No IMU");
    }
};

float WC_IMU::getKalman(bool isRoll){
    if(isRoll){
        Roll += dt * (angularvelocityX-biasR);
        PR[0][0] += dt * (dt*PR[1][1] - PR[0][1] - PR[1][0] + Q_angle);
        PR[0][1] -= dt * PR[1][1];
        PR[1][0] -= dt * PR[1][1];
        PR[1][1] += Q_bias * dt;
        float S = PR[0][0] + R_measure;
        float K[2]; // Kalman gain
        K[0] = PR[0][0] / S;
        K[1] = PR[1][0] / S;
        float y = RawRoll - Roll; // Angle difference
        Roll += K[0] * y;
        biasR += K[1] * y;
        float P00_temp = PR[0][0];
        float P01_temp = PR[0][1];
        PR[0][0] -= K[0] * P00_temp;
        PR[0][1] -= K[0] * P01_temp;
        PR[1][0] -= K[1] * P00_temp;
        PR[1][1] -= K[1] * P01_temp;
        return Roll;
    }else{
        Pitch += dt * (angularvelocityY-biasP);
        PP[0][0] += dt * (dt*PP[1][1] - PP[0][1] - PP[1][0] + Q_angle);
        PP[0][1] -= dt * PP[1][1];
        PP[1][0] -= dt * PP[1][1];
        PP[1][1] += Q_bias * dt;
        float S = PP[0][0] + R_measure;
        float K[2]; // Kalman gain
        K[0] = PP[0][0] / S;
        K[1] = PP[1][0] / S;
        float y = RawPitch - Pitch; // Angle difference
        Pitch += K[0] * y;
        biasP += K[1] * y;
        float P00_temp = PR[0][0];
        float P01_temp = PR[0][1];
        PP[0][0] -= K[0] * P00_temp;
        PP[0][1] -= K[0] * P01_temp;
        PP[1][0] -= K[1] * P00_temp;
        PP[1][1] -= K[1] * P01_temp;
        return Pitch;
    }
}; 

void WC_IMU::doAvg(){
    float PitchSum = 0;
    float RollSum = 0;
    for(int i=0; i<AvgScale-1; i++){
        PitchA[i] = PitchA[i+1];
        RollA[i] = RollA[i+1];
        PitchSum += PitchA[i];
        RollSum += RollA[i];
    }
    PitchA[AvgScale-1] = Pitch;
    RollA[AvgScale-1] = Roll;
    PitchSum += Pitch;
    RollSum += Roll;

    AvgPitch = PitchSum/AvgScale;
    AvgRoll = RollSum/AvgScale;
}

float WC_IMU::getUIPitch(){
    if(AvgPitch>UIPitch[0]){
        UIPitch[0] = AvgPitch;
    }
    if(AvgPitch<UIPitch[1]){
        UIPitch[1] = AvgPitch;
    }
    if(UIP){
        if((abs(UIPitch[1]-UIPitch[0]))<UI_Threshold){
            // Serial.print("[0]: "); Serial.print(UIPitch[0],4);  Serial.print("\t");
            // Serial.print("[1]: "); Serial.print(UIPitch[1],4);  Serial.print("\t");
            return ((UIPitch[0]+UIPitch[1])/2);
        }else{
            UItimerP = millis();
            UIPitch[0] = AvgPitch;
            UIPitch[1] = AvgPitch;
            UIP = false;
            return AvgPitch;
        }
    }else{
        if (millis() - UItimerP > 1000){
            UIP = true;
        }
        return AvgPitch;
    }
}

float WC_IMU::getUIRoll(){
    if(AvgRoll>UIRoll[0]){
        UIRoll[0] = AvgRoll;
    }
    if(AvgRoll<UIRoll[1]){
        UIRoll[1] = AvgRoll;
    }
    if(UIR){
        if((abs(UIRoll[1]-UIRoll[0]))<UI_Threshold){
            return ((UIRoll[1]+UIRoll[0])/2);
        }else{
            UItimerR = millis();
            UIRoll[0] = AvgRoll;
            UIRoll[1] = AvgRoll;
            UIR = false;
            return AvgRoll;
        }
    }else{
        if (millis() - UItimerR > 1000){
            UIR = true;
        }
        return AvgRoll;
    }
}

void WC_IMU::setUIthreshold(float UI_Threshold) { this->UI_Threshold = UI_Threshold; };
void WC_IMU::setQangle(float Q_angle) { this->Q_angle = Q_angle; };
void WC_IMU::setQbias(float Q_bias) { this->Q_bias = Q_bias; };
void WC_IMU::setRmeasure(float R_measure) { this->R_measure = R_measure; };
void WC_IMU::setAvgScale(int AvgScale) { this->AvgScale = AvgScale; };
float WC_IMU::getPitch() { return this->Pitch; };
float WC_IMU::getRoll() { return this->Roll; };
float WC_IMU::getPitchAvg() { return this->AvgPitch; };
float WC_IMU::getRollAvg() { return this->AvgRoll; };
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
        
        newData = false;
    }
}