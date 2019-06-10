#include "constants.h"

#ifdef __arm__ // raspberry pi
const char *portname = "/dev/serial0";
#else // x86 with usb adaptor
const char *portname = "/dev/ttyUSB0";
#endif

const int SPEED = B9600;
const int PARITY = 0;
