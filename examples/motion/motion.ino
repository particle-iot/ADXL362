

#include "ADXL362.h"

ADXL362 adxl362;

void setup()
{
   adxl362.begin();
}

void loop()
{
  int x, y, z;

  adxl362.readXYZmg(x, y, z);

  Serial.printlnf("x: %d, y: %d, z: %d", x, y, z);
  delay(500);
}
