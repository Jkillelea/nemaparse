#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "constants.h"
#include "util.h"

int main(int argc, char const *argv[]) {
    int fd = try_open(portname);
    if (fd < 0) {
        perror("Failed to open port");
        return 1;
    }

    // main loop
    char buf[512] = {0};
    const size_t buf_size = sizeof(buf);
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

        ssize_t buffoff = find_string_start(buf, "$GPGGA", buf_size, strlen("$GPGGA"));
        if (buffoff < 0)
            continue;

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
        printf("altitude: %.1lf (sl), %.1lf (wgs84), dop %.2lf, %d matches\n",
            alt_sl, alt_wgs84ellipsoid, horizontal_dilution, nmatches);

        // reset string buffer and flush serial buffer
        memset(buf, 0, sizeof(buf));
        tcflush(fd, TCIOFLUSH);
    }

    try_close(fd);
    return 0;
}
