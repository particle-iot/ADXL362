#pragma once

// This will load the definition for common Particle variable types
#include "Particle.h"


//------------- Accelerometer --------------


class ADXL362
{
public:

  /**
   * Creates a new ADXL362 class to manage a ADXL362 chip.
   * @param slaveSelectPin The pin of the chip-select for SPI communication.
   */
  ADXL362(int slaveSelectPin=SS);

  //
  // Basic Device control and readback functions
  //

  /**
   * Initialize the device. Perform Initial SPI setup, soft reset of device.
   * This method should be called before any others are used.
   */
  void begin();

  /**
   * turn on Measurement mode - required after reset
  */
  void beginMeasure();

  /**
   * Read the X-axis value using 16-bits.
   * @return The X-axis register as a 16-bit value.
   */
  int readX16();

  /**
   * Read the Y-axis value using 16-bits.
   * @return The Y-axis register as a 16-bit value.
   */
  int readY16();

  /**
   * Read the Z-axis value using 16-bits.
   * @return The Z-axis register as a 16-bit value.
   */
  int readZ16();

  /**
   * Read the X-axis value using 8-bits.
   * @return The X-axis register as a 8-bit value.
   */
  int readX();

  /**
   * Read the Y-axis value using 8-bits.
   * @return The Y-axis register as a 8-bit value.
   */
  int readY();

  /**
   * Read the Z-axis value using 8-bits.
   * @return The Z-axis register as a 8-bit value.
   */
  int readZ();

  /**
   * Read the X, Y, Z axes and temperature data in a single operation. All values are
   * 16-bits wide and  all values are sampled at the same time so the axes
   * and temperature reading are consistent.
   *
   * @param XData receives the X axis data
   * @param YData receives the Y axis data
   * @param ZData receives the Z axis data
   * @param Temperature receives the temperature
   */
  void readXYZTData(short &XData, short &YData, short &ZData, float &Temperature);

  /**
  * Read the X, Y, Z axis data in a single operation. All values are
  * 16-bits wide and all values are sampled at the same time so they are consistent.
  *
  * @param X receives the X axis data
  * @param Y receives the Y axis data
  * @param Z receives the Z axis data
  */
  void readXYZmg(int &X, int &Y, int &Z);

  /**
   * Convert the X, Y and Z axis values to rotation angle.
   */
  void XYZmgtoRPT(int X, int Y, int Z, float &Rho, float &Phi, float &Theta);

  /**
   * Read the internal temperature.
   * @return the internal temperature as a raw value.
   */
  int16_t readTemp();

  // need to add the following functions
  // void mapINT1(
  // void mapINT2
  // void autoSleep
  // void activityInterruptControl
  //    -Activity, Inactivity, Both
  //    - Referenced, Absolute
  //    - Free Fall, Linked Mode, Loop Mode

  /**
   * diagnostic function to output the state of all registers to the given
   * Print stream.
   * @param output  The stream to output to. Default is serial
   */
  void checkAllControlRegs(Print& output=Serial);


  /**
   *  Modify range (+-2g +-4g +-8g - ADXL362 Datasheep Page 33
   * Choose RangeFlag between XL362_FILTER_FLAG_2G (default), XL362_FILTER_FLAG_4G, XL362_FILTER_FLAG_8G
   */
  void setRange(uint8_t Range);

  /**
   * modify Bandwidth - ADXL362 Datasheep Page 33
   * @param BandWidth Choose Bandwidth between XL362_FILTER_FLAG_HBW (default), XL362_FILTER_FLAG_FBW
  */
  void setBandwidth(uint8_t BandWidth);

  /**
   * Modify Output Data Rate - ADXL362 Datasheep Page 33
   * @param ODR the data rate to set. Choose ODR between  XL362_FILTER_FLAG_ODR12,
   *   XL362_FILTER_FLAG_ODR25, XL362_FILTER_FLAG_ODR50, XL362_FILTER_FLAG_ODR100 (default),
   *   XL362_FILTER_FLAG_ODR200 , XL362_FILTER_FLAG_ODR400
   */
  void setOutputDatarate(uint8_t ODR);

  /**
   * Modify Noise Level - ADXL362 Datasheep Page 34
   * @param NoiseLevel the noise level to set. Choose NoiseLevel between XL362_POWER_FLAG_NOISE_NORMAL (default),
   *    XL362_POWER_FLAG_NOISE_LOW, XL362_POWER_FLAG_NOISE_ULTRALOW
  */
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

/* Configuration values */

#define XL362_FILTER_FLAG_2G 0b00000000
#define XL362_FILTER_FLAG_4G 0b01000000
#define XL362_FILTER_FLAG_8G 0b10000000

#define XL362_FILTER_FLAG_HBW 0b10000
#define XL362_FILTER_FLAG_FBW 0b00000

#define XL362_FILTER_FLAG_ODR12  0b000
#define XL362_FILTER_FLAG_ODR25  0b001
#define XL362_FILTER_FLAG_ODR50  0b010
#define XL362_FILTER_FLAG_ODR100 0b011
#define XL362_FILTER_FLAG_ODR200 0b100
#define XL362_FILTER_FLAG_ODR400 0b111

#define XL362_POWER_FLAG_NOISE_NORMAL   0b000000
#define XL362_POWER_FLAG_NOISE_LOW      0b010000
#define XL362_POWER_FLAG_NOISE_ULTRALOW 0b100000

#define XL362_POWER_FLAG_MEASURE_STANDBY 0b00
#define XL362_POWER_FLAG_MEASURE_RUNING  0b10
