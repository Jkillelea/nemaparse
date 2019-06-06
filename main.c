#include <errno.h>
#include <stdbool.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>

// vscode doesn't find this one flag in bits/termios.h for some reason.
#ifndef CRTSCTS
#warning "this bit of code is for keeping vscode happy and shouldn't compile!"
#define CRTSCTS  020000000000
#endif

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

int main(int argc, char const *argv[]) {
    int retcode;
    int fd = try_open(portname);
    if (fd < 0) {
        perror("Failed to open port");
        return 1;
    }

    // main loop
    char buf[512] = {0};
    size_t buf_size = sizeof(buf);
    while (true) {
        // read until buffer is full
        size_t recieved_bytes = 0;
        while(recieved_bytes < buf_size) {
            size_t nbytes_read = read(fd, buf + recieved_bytes,
                                        buf_size - recieved_bytes);
            recieved_bytes += nbytes_read;

            if (nbytes_read == 0) { // failed to read anything?
                perror("Failed to read from fd.");
                try_close(fd);
                fd = try_open(portname);
                if (fd < 0) {
                    perror("Failed to reopen fd.");
                    return 2;
                }
            }
        }
        // TODO: make sure that the GPGGA message was read all the way into the buffer correctly.
        // this method can chop a message in a random place.

        // find GPGGA message (coords)
        // buffoff (buffer offset) can overrun the buffer, so check for that
        // and avoid processing if no GPGGA message is found.
        size_t buffoff = 0;
        bool has_gpgga = true;
        while (strncmp("$GPGGA", buf + buffoff, 6) != 0) {
            buffoff += 1;
            if (buffoff > buf_size-1) {
                printf("No GPGGA message found this time\n");
                has_gpgga = false;
                break;
            }
        }
        // buffer has been overrun looking for a GPGGA message
        if (!has_gpgga)
            continue;

        // printf("%s", buf + buffoff);

        // TODO: parse checksum and DGPS data
        double timestamp           = 0.0;
        double lat_decimal_deg     = 0.0; // decimal degrees is a weird format
        char   lat_dir             = '\0';
        double lon_decimal_deg     = 0.0;
        char   lon_dir             = '\0';
        int    fix_qual            = 0.0;
        int    nsats               = 0;
        double horizontal_dilution = 0.0;
        double alt_sl              = 0.0;
        double alt_wgs84ellipsoid  = 0.0;

        // example: $GPGGA,003422.00,37xx.xxxxx,N,122xx.xxxxx,W,1,07,1.01,38.7,M,-30.0,M,,*55
        int nmatches = sscanf(buf + buffoff,
            "$GPGGA,%lf,%lf,%1s,%lf,%1s,%d,%d,%lf,%lf,M,%lf,M,,*53",
            &timestamp, &lat_decimal_deg, &lat_dir, &lon_decimal_deg,
            &lon_dir, &fix_qual, &nsats, &horizontal_dilution, &alt_sl,
            &alt_wgs84ellipsoid
            );

        // Obi-wan: Why do I get the feeling that you're going to be the death of me?
        // Annakin: Don't say that master, you're the only pointer I have.
        char lat_dir_ptr[2] = {lat_dir, '\0'};
        char lon_dir_ptr[2] = {lon_dir, '\0'};

        // convert from DDDMM.mmmmm (decimal minutes) to DDD.dddddd (plain decimal) format
        double lat_degrees = ((int) (lat_decimal_deg/100.0)); //  37.0000 N
        double lon_degrees = ((int) (lon_decimal_deg/100.0)); // 122.0000 W
        double lat_minutes = lat_decimal_deg - 100*lat_degrees; // MM.mmmmmm
        double lon_minutes = lon_decimal_deg - 100*lon_degrees;
        double lat_decimal = lat_minutes / 60; // 0.ddddddd
        double lon_decimal = lon_minutes / 60;
        double lat = lat_degrees + lat_decimal; // DDD.dddddd
        double lon = lon_degrees + lon_decimal;
        // printf("%lf -> (%lf)\n", lat_decimal_deg, lat_degrees + lat_decimal);
        // printf("%lf -> (%lf)\n", lon_decimal_deg, lon_degrees + lon_decimal);

        printf("\n%2d sats, quality %d, time %.0lf, %lf %s, %lf %s\n",
            nsats, fix_qual, timestamp, lat, lat_dir_ptr,
            lon, lon_dir_ptr);
        printf("altitude: %.1lf (sl), %.1lf (wgs84), %lf (horiz_dil), %d matches\n",
            alt_sl, alt_wgs84ellipsoid, horizontal_dilution, nmatches);

        // reset string buffer and flush serial buffer
        memset(buf, 0, sizeof(buf));
        tcflush(fd, TCIOFLUSH);
    }

    try_close(fd);
    return 0;
}
