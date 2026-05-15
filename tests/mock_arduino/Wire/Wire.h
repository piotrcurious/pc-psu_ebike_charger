#ifndef WIRE_H
#define WIRE_H
#include <stdint.h>
class TwoWire { public: void begin(int sda, int scl) {} };
extern TwoWire Wire;
#endif
