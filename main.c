#include <errno.h>
#include <stdbool.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>

#ifdef __arm__ // raspberry pi
const char *portname = "/dev/serial0";
#else // x86 with usb adaptor
const char *portname = "/dev/ttyUSB0";
#endif
const int SPEED = B9600;
const int PARITY = 0;

int try_close(int fd) {
    if (fd > 0)
        close(fd);
}

int main(int argc, char const *argv[]) {
    int retcode;
    struct termios serialport;
    memset(&serialport, 0, sizeof(struct termios));

    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("Failed to open");
        printf("Failed to open seiral port %s\n", portname);
        return 1;
    }

    // begin gross POSIX serial port code
    if (tcgetattr(fd, &serialport) != 0) {
        perror("Failed to get attrs");
        printf("Failed to get attrs for port %s\n", portname);
        try_close(fd);
        return 2;        
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
        return 3;
    }

    // main loop
    char buf[512] = {0};
    size_t buf_size = sizeof(buf);
    while (true) {
        // read until buffer is full
        size_t recieved_bytes = 0;
        while(recieved_bytes < buf_size) {
            size_t nbytes_read = read(fd, 
                                      buf + recieved_bytes,
                                      buf_size - recieved_bytes);
            recieved_bytes += nbytes_read;
        }
        // TODO: make sure that the GPGGA message was read all the way into the buffer correctly.
        // this method can chop a message in a random place.

        // find GPGGA message (coords)
        // buffoff (buffer offset) can overrun the buffer, so check for that
        // and avoid processing if no GPGGA message is found.
        size_t buffoff = 0;
        bool no_gpgga = false;
        while (strncmp("$GPGGA", buf + buffoff, 6) != 0) {
            buffoff += 1;
            if (buffoff > buf_size-1) {
                printf("No GPGGA message found this time\n");
                no_gpgga = true;
                break;
            }
        }
        // buffer has been overrun looking for a GPGGA message
        if (no_gpgga) 
            continue;

        // printf("%s", buf + buffoff);
        
        float timestamp           = 0.0;
        float lat                 = 0.0;
        char  lat_dir             = '\0';
        float lon                 = 0.0;
        char  lon_dir             = '\0';
        int   fix_qual            = 0.0;
        int   nsats               = 0;
        float horizontal_dilution = 0.0;
        float alt_sl              = 0.0;
        float alt_wgs84ellipsoid  = 0.0;

        // example: "$GPGGA,230404.00,37xx.xxxx,N,122xx.xxxx,W,1,06,1.74,24.8,M,-30.0,M,,*53"
        int nmatches = sscanf(buf + buffoff, 
            "$GPGGA,%f,%f,%1s,%f,%1s,%d,%d,%f,%f,M,%f,M,,*53", 
            &timestamp, &lat, &lat_dir, &lon, &lon_dir, &fix_qual,
            &nsats, &horizontal_dilution, &alt_sl, &alt_wgs84ellipsoid
            );

        // Obi-wan: Why do I get the feeling that you're going to be the death of me?
        // Annakin: Don't say that master, you're the only pointer I have.
        char lat_dir_ptr[2] = {lat_dir, '\0'};
        char lon_dir_ptr[2] = {lon_dir, '\0'};

        printf("\n%d matches. %d sats, quality %d, time %f, lat %f %s, lon %f %s\n", 
            nmatches, nsats, fix_qual, timestamp, 
            lat/100, lat_dir_ptr, lon/100, lon_dir_ptr);
        printf("altitude: %f (sl), %f (wgs84), %f (horiz_dil)\n", 
            alt_sl, alt_wgs84ellipsoid, horizontal_dilution);

        memset(buf, 0, sizeof(buf));
        tcflush(fd, TCIOFLUSH);
    }

    try_close(fd);
    return 0;
}
