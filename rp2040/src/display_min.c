/* display_min.c — see display_min.h. */
#include "display_min.h"
#include <stdio.h>
#include <math.h>

void format_coordinates(double lat, double lon, char *buffer, size_t size)
{
    snprintf(buffer, size, "%.5f %c, %.5f %c",
             fabs(lat), (lat >= 0.0) ? 'N' : 'S',
             fabs(lon), (lon >= 0.0) ? 'E' : 'W');
}

void open_osm_map(double lat, double lon)
{
    (void)lat;
    (void)lon;
}

void log_to_terminal(const char *message)
{
    (void)message;
}
