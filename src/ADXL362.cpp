/* aDXL362 library by Richard Whitney <richard@particle.io>
 */

#include "ADXL362.h"
#include <math.h>


/*
 Arduino Library for Analog Devices ADXL362 - Micropower 3-axis accelerometer
 go to http://www.analog.com/ADXL362 for datasheet

 License: CC BY-SA 3.0: Creative Commons Share-alike 3.0. Feel free
 to use and abuse this code however you'd like. If you find it useful
 please attribute, and SHARE-ALIKE!

 Created June 2012
 by Anne Mahaffey - hosted on http://annem.github.com/ADXL362
 Modified Mars 2014
 by pixelk
 Modified for Spark Core/Button October 2014
 by jenesaisdiq
 Updated for Particle InternetButton February 2018
 by csylvain

 */


 /**
  * Uncomment to turn debugging on
  */
// #define ADXL362_DEBUG

/* ADXL Registers */

#define XL362_DEVID_AD      0x00
#define XL362_DEVID_MST     0x01
#define XL362_PARTID      0x02
#define XL362_REVID     0x03
#define XL362_XDATA     0x08
#define XL362_YDATA     0x09
#define XL362_ZDATA     0x0A
#define XL362_STATUS      0x0B
#define XL362_FIFO_ENTRIES_L    0x0C
#define XL362_FIFO_ENTRIES_H    0x0D
#define XL362_XDATA_L     0x0E
#define XL362_XDATA_H     0x0F
#define XL362_YDATA_L     0x10
#define XL362_YDATA_H     0x11
#define XL362_ZDATA_L     0x12
#define XL362_ZDATA_H     0x13
#define XL362_TEMP_L      0x14
#define XL362_TEMP_H      0x15
#define XL362_SOFT_RESET    0x1F
#define XL362_THRESH_ACT_L    0x20
#define XL362_THRESH_ACT_H    0x21
#define XL362_TIME_ACT        0x22
#define XL362_THRESH_INACT_L  0x23
#define XL362_THRESH_INACT_H  0x24
#define XL362_TIME_INACT_L    0x25
#define XL362_TIME_INACT_H    0x26
#define XL362_ACT_INACT_CTL   0x27
#define XL362_FIFO_CONTROL    0x28
#define XL362_FIFO_SAMPLES    0x29
#define XL362_INTMAP1     0x2A
#define XL362_INTMAP2     0x2B
#define XL362_FILTER_CTL    0x2C
#define XL362_POWER_CTL     0x2D
#define XL362_SELF_TEST     0x2E


ADXL362::ADXL362(int slaveSelectPin) : slaveSelectPin(uint16_t(slaveSelectPin)) {}

//
//  begin()
//  Initial SPI setup, soft reset of device
//
void ADXL362::begin() {
  pinMode(slaveSelectPin, OUTPUT);
  SPI.begin();
  SPI.setDataMode(SPI_MODE0); //CPHA = CPOL = 0    MODE = 0
  delay(500);

  // soft reset
  mgperLSB = 1;
  SPIwriteOneRegister(XL362_SOFT_RESET, 0x52);  // Write to SOFT RESET, "R"
  delay(10);
#ifdef ADXL362_DEBUG
  Serial.println("Soft Reset\n");
#endif
 }


//
//  beginMeasure()
//  turn on Measurement mode - required after reset
//
void ADXL362::beginMeasure() {
  uint8_t temp = SPIreadOneRegister(XL362_POWER_CTL); // read Reg 2D before modifying for measure mode
#ifdef ADXL362_DEBUG
  Serial.print(  "Setting Measeurement Mode - Reg XL362_POWER_CTL before = ");
  Serial.print(temp);
#endif

  // turn on measurement mode
  temp = (temp & 0b11111100) | XL362_POWER_FLAG_MEASURE_RUNING;     // turn on measurement bit in Reg XL362_POWER_CTL
  SPIwriteOneRegister(XL362_POWER_CTL, temp); // Write to XL362_POWER_CTL, Measurement Mode
  delay(10);

#ifdef ADXL362_DEBUG
  temp = SPIreadOneRegister(XL362_POWER_CTL);
  Serial.print(  ", Reg XL362_POWER_CTL after = ");
  Serial.println(temp);
#endif
}



//
//  readX(), readY(), readZ(), readT()
//reading off the special 8-bit registers as documented in the ADXL362 spec
//IMPORTANT to make it a signed 8-bit int so that the data is interpreted correctly
int ADXL362::readX(){
  int8_t XDATA = SPIreadOneRegister(0x08);
#ifdef ADXL362_DEBUG
 Serial.print("XDATA = ");
 Serial.println(XDATA);
#endif
  return (int)XDATA;
}

int ADXL362::readY(){
  int8_t YDATA = SPIreadOneRegister(0x09);
#ifdef ADXL362_DEBUG
 Serial.print("\tYDATA = ");
 Serial.println(YDATA);
#endif
  return (int)YDATA;
}

int ADXL362::readZ(){
  int8_t ZDATA = SPIreadOneRegister(0x0A);
#ifdef ADXL362_DEBUG
 Serial.print("\tZDATA = ");
 Serial.println(ZDATA);
#endif
  return (int)ZDATA;
}

//Temperature only has a 12-bit sign extended to 16-bit version, so read both regs
//@return 8 MSB to mimic the 8 MSB accelerometer registers
int ADXL362::readT(){
  int16_t TEMP = SPIreadTwoRegisters(XL362_TEMP_L);
  TEMP = (TEMP & 0xfff0) >> 4; // grab 8 MSB plus SX bits
  if (TEMP & 0x0f00) { // if SX bits == 1
    TEMP &= 0x00ff; // use only the 8 data bits
    TEMP *= -1; // and make it negative
  }
#ifdef ADXL362_DEBUG
  Serial.print("\tTDATA = ");
  Serial.println(TDATA);
#endif
  return (int)TDATA;
}

//
//  readX16(), readY16(), readZ16(), readT16()
//  Read 12-bit sign extended X, Y, Z, and Temp registers and return 16-bit values
//
int ADXL362::readX16(){
  int16_t XDATA = SPIreadTwoRegisters(XL362_XDATA_L);
#ifdef ADXL362_DEBUG
  Serial.print("XDATA = ");
  Serial.println(XDATA);
#endif
  return XDATA;
}

int ADXL362::readY16(){
  int16_t YDATA = SPIreadTwoRegisters(XL362_YDATA_L);
#ifdef ADXL362_DEBUG
  Serial.print("\tYDATA = ");
  Serial.println(YDATA);
#endif
  return YDATA;
}

int ADXL362::readZ16(){
  int16_t ZDATA = SPIreadTwoRegisters(XL362_ZDATA_L);
#ifdef ADXL362_DEBUG
  Serial.print("\tZDATA = ");
  Serial.println(ZDATA);
#endif
  return ZDATA;
}

// Temp sensor returns a 12-bit value in two 8-bit regs with 4-bit sign extension
//  worth knowing that this is an INTERNAL temperature measurement, so doesn't reflect
//  the environment itself accurately, but is (or can be) used to increase acceleration sensor accuracy
int ADXL362::readT16(){
  int16_t TDATA = SPIreadTwoRegisters(XL362_TEMP_L);
#ifdef ADXL362_DEBUG
  Serial.print("\tTEMP = ");
  Serial.println(TDATA);
#endif
  return TDATA;
}

void ADXL362::readXYZT(short &XData, short &YData, short &ZData, short &TData){

  // burst SPI read
  // A burst read of all three axis is required to guarantee all measurements correspond to same sample time
  digitalWrite(slaveSelectPin, LOW);

  SPI.transfer(0x0B);  // read instruction
  SPI.transfer(XL362_XDATA);  // Start at XData Reg
  XData = SPI.transfer(0x00);
  YData = SPI.transfer(0x00);
  ZData = SPI.transfer(0x00);
  digitalWrite(slaveSelectPin, HIGH);

  digitalWrite(slaveSelectPin, LOW);
  SPI.transfer(0x0B);  // read instruction
  SPI.transfer(XL362_TEMP_L);  // Start at Temp Reg
  TData = (SPI.transfer(0x00) & 0xf0) >> 4;
  TData = TData + ((SPI.transfer(0x00) & 0x0f) << 4);
  digitalWrite(slaveSelectPin, HIGH);

#ifdef ADXL362_DEBUG
  Serial.print("XDATA = "); Serial.print(XData);
  Serial.print("\tYDATA = "); Serial.print(YData);
  Serial.print("\tZDATA = "); Serial.print(ZData);
  Serial.println("\tTDATA = "); Serial.println(TData);
#endif
}

void ADXL362::readXYZT16(int &XData, int &YData, int &ZData, int &TData){

  // burst SPI read
  // A burst read of all three axis is required to guarantee all measurements correspond to same sample time
  digitalWrite(slaveSelectPin, LOW);

  SPI.transfer(0x0B);  // read instruction
  SPI.transfer(XL362_XDATA_L);  // Start at XData Reg
  XData = SPI.transfer(0x00);
  XData = XData + ((short)SPI.transfer(0x00) << 8);
  YData = SPI.transfer(0x00);
  YData = YData + ((short)SPI.transfer(0x00) << 8);
  ZData = SPI.transfer(0x00);
  ZData = ZData + ((short)SPI.transfer(0x00) << 8);
  TData = SPI.transfer(0x00);
  TData = TData + ((short)SPI.transfer(0x00) << 8);
  //Temperature = (float)RawTemperature * 0.065; // let InternetButton scale this. return raw value.
  digitalWrite(slaveSelectPin, HIGH);

#ifdef ADXL362_DEBUG
  Serial.print("XDATA16 = "); Serial.print(XData);
  Serial.print("\tYDATA16 = "); Serial.print(YData);
  Serial.print("\tZDATA16 = "); Serial.print(ZData);
  Serial.println("\tTDATA16 = "); Serial.println(TData);
#endif
}

void ADXL362::readXYZmg(int &X, int &Y, int &Z){
  // burst SPI read
  // A burst read of all three axis is required to guarantee all measurements correspond to same sample time
  digitalWrite(slaveSelectPin, LOW);
  SPI.transfer(0x0B);  // read instruction
  SPI.transfer(XL362_XDATA_L);  // Start at XData Reg
  short XData = SPI.transfer(0x00);
  XData = XData + ((short)SPI.transfer(0x00) << 8);
  short YData = SPI.transfer(0x00);
  YData = YData + ((short)SPI.transfer(0x00) << 8);
  short ZData = SPI.transfer(0x00);
  ZData = ZData + ((short)SPI.transfer(0x00) << 8);
  digitalWrite(slaveSelectPin, HIGH);

  X = (int)XData * mgperLSB;
  Y = (int)YData * mgperLSB;
  Z = (int)ZData * mgperLSB;

#ifdef ADXL362_DEBUG
  Serial.print("x = "); Serial.print(X);
  Serial.print("\ty = "); Serial.print(Y);
  Serial.println("\tz = "); Serial.print(Z);
#endif
}

void ADXL362::XYZmgtoRPT(int X, int Y, int Z, float &Rho, float &Phi, float &Theta){
  Rho = atan2(float(X), sqrt(pow(float(Y),2)+pow(float(Z),2)));
  Rho *= 180/M_PI;

  Phi = atan2(float(Y), sqrt(pow(float(X),2)+pow(float(Z),2)));
  Phi *= 180/M_PI;

  Theta = atan2(sqrt(pow(float(X),2)+pow(float(Y),2)),float(Z));
  Theta *= 180/M_PI;
}

void ADXL362::checkAllControlRegs(Print& output){
  //uint8_t filterCntlReg = SPIreadOneRegister(0x2C);
  //uint8_t ODR = filterCntlReg & 0x07;  output.print("ODR = ");  output.println(ODR, HEX);
  //uint8_t ACT_INACT_CTL_Reg = SPIreadOneRegister(0x27);      output.print("ACT_INACT_CTL_Reg = "); output.println(ACT_INACT_CTL_Reg, HEX);
  digitalWrite(slaveSelectPin, LOW);
  SPI.transfer(0x0B);  // read instruction
  SPI.transfer(0x20);  // Start burst read at Reg 20
  output.println("Start Burst Read of all Control Regs - Library version 6-24-2012:");
  output.print("Reg XL362_THRESH_ACT_L   = B");   output.println(SPI.transfer(0x00), BIN);
  output.print("Reg XL362_THRESH_ACT_H   = B");   output.println(SPI.transfer(0x00), BIN);
  output.print("Reg XL362_TIME_ACT       = B");   output.println(SPI.transfer(0x00), BIN);
  output.print("Reg XL362_THRESH_INACT_L = B");   output.println(SPI.transfer(0x00), BIN);
  output.print("Reg XL362_THRESH_INACT_H = B");   output.println(SPI.transfer(0x00), BIN);
  output.print("Reg XL362_TIME_INACT_L   = B");   output.println(SPI.transfer(0x00), BIN);
  output.print("Reg XL362_TIME_INACT_H   = B");   output.println(SPI.transfer(0x00), BIN);
  output.print("Reg XL362_ACT_INACT_CTL  = B");   output.println(SPI.transfer(0x00), BIN);
  output.print("Reg XL362_FIFO_CONTROL   = B");   output.println(SPI.transfer(0x00), BIN);
  output.print("Reg XL362_FIFO_SAMPLES   = B");   output.println(SPI.transfer(0x00), BIN);
  output.print("Reg XL362_INTMAP1        = B");   output.println(SPI.transfer(0x00), BIN);
  output.print("Reg XL362_INTMAP2        = B");   output.println(SPI.transfer(0x00), BIN);
  output.print("Reg XL362_FILTER_CTL     = B");   output.println(SPI.transfer(0x00), BIN);
  output.print("Reg XL362_POWER_CTL      = B");   output.println(SPI.transfer(0x00), BIN);
  output.print("Reg XL362_SELF_TEST      = B");   output.println(SPI.transfer(0x00), BIN);

  digitalWrite(slaveSelectPin, HIGH);
}

void ADXL362::setRange(uint8_t Range){
  // Modify range (+-2g +-4g +-8g - ADXL362 Datasheep Page 33
  // Choose RangeFlag between XL362_FILTER_FLAG_2G (default), XL362_FILTER_FLAG_4G, XL362_FILTER_FLAG_8G
  uint8_t temp = SPIreadOneRegister(XL362_FILTER_CTL);  // read Reg XL362_FILTER_CTL before modifying
#ifdef ADXL362_DEBUG
  Serial.print(  "Setting Measurement Range - Reg XL362_FILTER_CTL before = ");
  Serial.print(temp);
#endif

  switch ( Range ) { // Range affects converting LSB to mg
  case XL362_FILTER_FLAG_2G:
    mgperLSB = 1;
    break;
  case XL362_FILTER_FLAG_4G:
    mgperLSB = 2;
    break;
  case XL362_FILTER_FLAG_8G:
    mgperLSB = 4;
    break;
  default:
    // YOU SHOULDN'T BE HERE !
    mgperLSB = 1;
    break;
  }

  temp = (temp & 0b00111111) | Range;
  SPIwriteOneRegister(XL362_FILTER_CTL, temp); // Write to XL362_FILTER_CTL
  delay(10);

#ifdef ADXL362_DEBUG
  temp = SPIreadOneRegister(XL362_FILTER_CTL);
  Serial.print(  ", Reg after = ");
  Serial.println(temp);
#endif
}

void ADXL362::setBandwidth(uint8_t BandWidth){
  // modify Bandwidth - ADXL362 Datasheep Page 33
  // Choose Bandwidth between XL362_FILTER_FLAG_HBW (default), XL362_FILTER_FLAG_FBW
  uint8_t temp = SPIreadOneRegister(XL362_FILTER_CTL);  // read Reg XL362_FILTER_CTL before modifying
#ifdef ADXL362_DEBUG
  Serial.print(  "Setting BandWidth - Reg XL362_FILTER_CTL before = ");
  Serial.print(temp);
#endif

  temp = (temp & 0b11101111) | BandWidth;
  SPIwriteOneRegister(XL362_FILTER_CTL, temp); // Write to XL362_FILTER_CTL
  delay(10);

#ifdef ADXL362_DEBUG
  temp = SPIreadOneRegister(XL362_FILTER_CTL);
  Serial.print(  ", Reg after = ");
  Serial.println(temp);
#endif
}

void ADXL362::setOutputDatarate(uint8_t ODR){
  // modify Output Data Rate - ADXL362 Datasheep Page 33
  // Choose ODR between  XL362_FILTER_FLAG_ODR12, XL362_FILTER_FLAG_ODR25, XL362_FILTER_FLAG_ODR50, XL362_FILTER_FLAG_ODR100 (default), XL362_FILTER_FLAG_ODR200 , XL362_FILTER_FLAG_ODR400
  uint8_t temp = SPIreadOneRegister(XL362_FILTER_CTL);  // read Reg XL362_FILTER_CTL before modifying
#ifdef ADXL362_DEBUG
  Serial.print(  "Setting Output Data Rate - Reg XL362_FILTER_CTL before = ");
  Serial.print(temp);
#endif

  temp = (temp & 0b11111000) | ODR;
  SPIwriteOneRegister(XL362_FILTER_CTL, temp); // Write to XL362_FILTER_CTL
  delay(10);

#ifdef ADXL362_DEBUG
  temp = SPIreadOneRegister(XL362_FILTER_CTL);
  Serial.print(  ", Reg after = ");
  Serial.println(temp);
#endif
}

void ADXL362::setNoiseLevel(uint8_t NoiseLevel){
  // modify Noise Level - ADXL362 Datasheep Page 34
  // Choose NoiseLevel between XL362_POWER_FLAG_NOISE_NORMAL (default), XL362_POWER_FLAG_NOISE_LOW, XL362_POWER_FLAG_NOISE_ULTRALOW
  uint8_t temp = SPIreadOneRegister(XL362_POWER_CTL); // read Reg XL362_FILTER_CTL before modifying
#ifdef ADXL362_DEBUG
  Serial.print(  "Setting Output Data Rate - Reg XL362_POWER_CTL before = ");
  Serial.print(temp);
#endif

  temp = (temp & 0b11001111) | NoiseLevel;
  SPIwriteOneRegister(XL362_POWER_CTL, temp); // Write to XL362_FILTER_CTL
  delay(10);

#ifdef ADXL362_DEBUG
  temp = SPIreadOneRegister(XL362_POWER_CTL);
  Serial.print(  ", Reg after = ");
  Serial.println(temp);
#endif
}

// Basic SPI routines to simplify code
// read and write one register

uint8_t ADXL362::SPIreadOneRegister(uint8_t regAddress){
  uint8_t regValue = 0;

  digitalWrite(slaveSelectPin, LOW);
  SPI.transfer(0x0B);  // read instruction
  SPI.transfer(regAddress);
  regValue = SPI.transfer(0x00);
  digitalWrite(slaveSelectPin, HIGH);

  return regValue;
}

void ADXL362::SPIwriteOneRegister(uint8_t regAddress, uint8_t regValue){

  digitalWrite(slaveSelectPin, LOW);
  SPI.transfer(0x0A);  // write instruction
  SPI.transfer(regAddress);
  SPI.transfer(regValue);
  digitalWrite(slaveSelectPin, HIGH);
}

int ADXL362::SPIreadTwoRegisters(uint8_t regAddress){
  int twoRegValue = 0;

  digitalWrite(slaveSelectPin, LOW);
  SPI.transfer(0x0B);  // read instruction
  SPI.transfer(regAddress);
  twoRegValue = SPI.transfer(0x00);
  twoRegValue = twoRegValue + (SPI.transfer(0x00) << 8);
  digitalWrite(slaveSelectPin, HIGH);

  return twoRegValue;
}

void ADXL362::SPIwriteTwoRegisters(uint8_t regAddress, int twoRegValue){

  uint8_t twoRegValueH = twoRegValue >> 8;
  uint8_t twoRegValueL = twoRegValue;

  digitalWrite(slaveSelectPin, LOW);
  SPI.transfer(0x0A);  // write instruction
  SPI.transfer(regAddress);
  SPI.transfer(twoRegValueL);
  SPI.transfer(twoRegValueH);
  digitalWrite(slaveSelectPin, HIGH);
}
