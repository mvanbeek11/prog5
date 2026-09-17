#include "BPM280.h"
#include <Wire.h>
#include <stdio.h>
#include <math.h>

BMP280::BMP280()
{

}

char BMP280::begin()
{
  Wire.begin();
  return (readCalibration());
}
bool BMP280::readCalibration()
{
	return (
		readCalibrationValue(0x88, dig_T1, false) &&
		readCalibrationValue(0x8A, dig_T2, true)  &&
		readCalibrationValue(0x8C, dig_T3, true)  &&
		readCalibrationValue(0x8E, dig_P1, false) &&
		readCalibrationValue(0x90, dig_P2, true)  &&
		readCalibrationValue(0x92, dig_P3, true)  &&
		readCalibrationValue(0x94, dig_P4, true)  &&
		readCalibrationValue(0x96, dig_P5, true)  &&
		readCalibrationValue(0x98, dig_P6, true)  &&
		readCalibrationValue(0x9A, dig_P7, true)  &&
		readCalibrationValue(0x9C, dig_P8, true)  &&
		readCalibrationValue(0x9E, dig_P9, true)
	);
}

bool BMP280::readCalibrationValue(uint8_t address, double &value, bool isSigned){
  unsigned char data[2];

  data[0] = address;
  if (!readBytes(data,2)){
    value = 0;
    return false;

  }
  uint16_t raw = ((uint16_t)data[1] <<8 )|data[0];
  value = isSigned ? (double)(int16_t)raw : (double)raw;
  return true;

}
char BMP280::readBytes(unsigned char *values, char length){
  char x;
  char error;
  Wire.beginTransmission(BMP280_ADDR);
  Wire.write(values[0]);
  error = Wire.endTransmission();
  if (error == 0){
    Wire.requestFrom(BMP280_ADDR, length);
    while(Wire.available() != length) ; 
		for(x=0;x<length;x++)
		{
			values[x] = Wire.read();
		}
		return(1);
	}
	return(0);

}
char BMP280::writeBytes(unsigned char *values, char length){
  char error;
  Wire.beginTransmission(BMP280_ADDR);
  Wire.write(values, length);
  error = Wire.endTransmission();
  if (error == 0){
    return (1);
    }
  else
  return(0);
}

char BMP280::startMeasurement(void){
  unsigned char data[2];
  data[0] = BMP280_REG_CONTROL;
  data[1]= BMP280_COMMAND_PRESSURE3;
  if (writeBytes(data, 2)){
    return 24;
  
  }
  else 
  return 0;
}

char BMP280::getUnPT(double &uP, double &uT){
  unsigned char data[6];
  char result;
  data[0] = BMP280_REG_RESULT_PRESSURE;
  result = readBytes(data, 6);
  if (result){
    uP = (double)((uint32_t)data[0] <<12 | (uint32_t)data[1] << 4| (uint32_t)data[2]>> 4);
    uT = (double)((uint32_t)data[3] <<12 | (uint32_t)data[4] << 4| (uint32_t)data[5]>> 4);
  }
  return (result);
}

char BMP280::getTemperatureAndPressure(double &T, double &P){
  uint8_t waitTime = startMeasurement();
  if (waitTime == 0)
  return false;
  delay (waitTime);
  double uT;
  double uP;
  char result = getUnPT(uP,uT);
  if (result!=0){
    result = calcTemperature(T,uT);
    if(result){
      result = calcPressure(P, uP);
      if(result)
      return (1);
    }
  
  } 
}

char BMP280::calcTemperature(double &T, double &uT){
  double var1 = (uT/16384.0 - dig_T1/1024.0)*dig_T2;
	double var2 = ((uT/131072.0 - dig_T1/8192.0)*(uT/131072.0 - dig_T1/8192.0))*dig_T3;
	t_fine = var1+var2;
	T = (var1+var2)/5120.0;
  if(T>100||T<-100)
  return(0);
return(1);
}

char BMP280::calcPressure(double &P, double &uP){
  double var1 , var2 ;
	
	var1 = (t_fine/2.0) - 64000.0;
  var2 = var1 * (var1 * dig_P6/32768.0);
  var2 = var2 + (var1 * dig_P5 * 2.0);
  var2 = (var2/4.0)+((dig_P4)*65536.0);
  var1 = (dig_P3 * var1 * var1/524288.0 + dig_P2 * var1) / 524288.0;
  var1 = (1.0 + var1/32768.0) * dig_P1;
  P = 1048576.0- uP;
  P = (P-(var2/4096.0))*6250.0/var1 ;
  var1 = dig_P9*P*P/2147483648.0;
  var2 = P*dig_P8/32768.0;
  P = P + (var1+var2+dig_P7)/16.0;
  P = P/100.0 ;
	
	if(P>1200.0 || P < 800.0)return (0);
	return (1);
}