#ifndef POLOLU_SIMPLE_H
#define POLOLU_SIMPLE_H

#include <HardwareSerial.h>

class pololu_simple {

public:

  pololu_simple(HardwareSerial *newport);
  int readByte();
  void exitSafeStart();
  void setMotorSpeed(int speed);
  unsigned char setMotorLimit(unsigned char  limitID, unsigned int limitValue);
  unsigned int getVariable(unsigned char variableID);

private:
  HardwareSerial *port;
};

#endif // POLOLU_SIMPLE_H
