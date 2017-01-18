#pragma once

/* aDXL362 library by Richard Whitney <richard@particle.io>
 */

// This will load the definition for common Particle variable types
#include "Particle.h"


//------------- Accelerometer --------------

class ADXL362
{
public:

  ADXL362(int slaceSelectPin);

  //
  // Basic Device control and readback functions
  //
  void begin();
  void beginMeasure();
  int readX16();
  int readY16();
  int readZ16();
  int readX();
  int readY();
  int readZ();
  void readXYZTData(short &XData, short &YData, short &ZData, float &Temperature);
  void readXYZmg(int &X, int &Y, int &Z);
  void XYZmgtoRPT(int X, int Y, int Z, float &Rho, float &Phi, float &Theta);
  int16_t readTemp();

  // need to add the following functions
  // void mapINT1(
  // void mapINT2
  // void autoSleep
  // void activityInterruptControl
  //    -Activity, Inactivity, Both
  //    - Referenced, Absolute
  //    - Free Fall, Linked Mode, Loop Mode


  void checkAllControlRegs();

  void setRange(uint8_t Range);
  void setBandwidth(uint8_t BandWidth);
  void setOutputDatarate(uint8_t ODR);
  void setNoiseLevel(uint8_t NoiseLevel);

  //  Low-level SPI control, to simplify overall coding
  uint8_t SPIreadOneRegister(uint8_t regAddress);
  void SPIwriteOneRegister(uint8_t regAddress, uint8_t regValue);
  int  SPIreadTwoRegisters(uint8_t regAddress);
  void SPIwriteTwoRegisters(uint8_t regAddress, int twoRegValue);

private:
  uint8_t mgperLSB; // default +-2g XL362_FILTER_FLAG_2G -> 1mg/LSB (ADXL362 Datasheet page 4)
  uint16_t slaveSelectPin;
};
