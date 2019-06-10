#include "constants.h"
#include "util.h"

int try_close(int fd) {
    if (fd > 0)
        return close(fd);
    return -1;
}

int try_open(const char *portname) {
    struct termios serialport;
    memset(&serialport, 0, sizeof(struct termios));

    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
        return -1;

    // begin gross POSIX serial port code
    if (tcgetattr(fd, &serialport) != 0) {
        perror("Failed to get attrs");
        printf("Failed to get attrs for port %s\n", portname);
        try_close(fd);
        return -2;
    }

    cfsetispeed(&serialport, SPEED);
    cfsetospeed(&serialport, SPEED);

    serialport.c_cflag = (serialport.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    serialport.c_iflag &= ~IGNBRK; // disable break processing
    serialport.c_lflag = 0;        // no signaling chars, no echo,
                                   // no canonical processing
    serialport.c_oflag = 0;        // no remapping, no delays
    serialport.c_cc[VMIN]  = 1;    // read blocks
    serialport.c_cc[VTIME] = 50;   // 5 seconds read timeout

    serialport.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
    serialport.c_cflag |= (CLOCAL | CREAD);        // ignore modem controls,
                                                   // enable reading
    serialport.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    serialport.c_cflag |=  PARITY;
    serialport.c_cflag &= ~CSTOPB;
    serialport.c_cflag &= ~CRTSCTS; // vs code doesn't find this one flag? still compiles.

    if (tcsetattr(fd, TCSANOW, &serialport) != 0) {
        perror("tcsetattr failed");
        printf("error %d from tcsetattr", errno);
        try_close(fd);
        return -3;
    }

    return fd;
}

// convert from DDDMM.mmmmm (decimal minutes) to DDD.dddddd (plain decimal) format
// TODO: double check this!
double decimal_minutes2decimal_decimal(const double decimal_minutes) {
    double degrees = ((int) (decimal_minutes/100.0)); // DDD
    double minutes = decimal_minutes - 100*degrees;   // MM.mmmmmm
    double decimal = minutes / 60;                    // 0.ddddddd
    return (degrees + decimal);                       // DDD.dddddd
}

ssize_t find_string_start(const char *buf, const char *substr, size_t buf_size, size_t substr_size) {
    ssize_t buffoff = 0;
    bool found = true;
    while (strncmp(substr, buf+buffoff, substr_size) != 0) {
        buffoff += 1;
        if (buffoff > buf_size - 1) {
            found = false;
            break;
        }
    }

    if (found)
        return buffoff;
    return -1;
}