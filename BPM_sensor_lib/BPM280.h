#ifndef BMP280_h
#define BMP280_h
#include <Arduino.h>

class BMP280
{
  public: 
    BMP280();

    char begin();
    char getTemperatureAndPressure(double &T, double &P);

  private:
    char startMeasurement(void);
    char calcTemperature(double &T, double &uT);
    char calcPressure(double &P, double &uP);
    bool readCalibration();
    bool readCalibrationValue(uint8_t address, double &value, bool isSigned);
    char readBytes(unsigned char *values, char length);
    char writeBytes(unsigned char *values, char length);
    char getUnPT(double &uP, double &uT);	
    double dig_T1, dig_T2 , dig_T3 , dig_T4 , dig_P1, dig_P2 , dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    double t_fine;
};
#define BMP280_ADDR 0x76 // 7-bit address

#define	BMP280_REG_CONTROL 0xF4
#define	BMP280_REG_RESULT_PRESSURE 0xF7			// 0xF7(msb) , 0xF8(lsb) , 0xF9(xlsb) : stores the pressure data.

#define	BMP280_COMMAND_PRESSURE3 0x31  

#endif 