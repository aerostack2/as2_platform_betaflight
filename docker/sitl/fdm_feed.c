// Feeds Betaflight SITL a drone that is level and at rest, at 50 Hz.
// Packet layout and frame conventions follow betaflight src/test/sitl/sitl_harness.py.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

int main(void)
{
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  struct sockaddr_in dst = {.sin_family = AF_INET, .sin_port = htons(9003)};
  dst.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

  struct timespec t0, now;
  clock_gettime(CLOCK_MONOTONIC, &t0);
  for (;;) {
    clock_gettime(CLOCK_MONOTONIC, &now);
    double pkt[18] = {
      (now.tv_sec - t0.tv_sec) + (now.tv_nsec - t0.tv_nsec) / 1e9,  // timestamp (s)
      0.0, 0.0, 0.0,                // gyro (rad/s)
      0.0, 0.0, -9.80665,           // negated body specific force: 1 g up
      0.70710678, 0.0, 0.0, 0.70710678,  // level, Gazebo-bridge quaternion (w x y z)
      0.0, 0.0, 0.0,                // ENU velocity (m/s)
      999.0, 999.0, 0.0,            // lon, lat out of range: no GPS fix; altitude (m)
      101325.0,                     // pressure (Pa)
    };
    sendto(sock, pkt, sizeof(pkt), 0, (struct sockaddr *)&dst, sizeof(dst));
    usleep(20000);
  }
}
