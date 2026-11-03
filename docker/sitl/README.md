# Betaflight SITL

Betaflight 2026.6.2 built as a Linux program (SITL), to run the platform without
a flight controller. The image is about 6 MB: a static SITL binary, a feeder that
holds the simulated drone level and at rest, and busybox.

- MSP on `tcp://:5761` (UART1, for the platform) and `tcp://:5762` (UART2, for inspection).
- RC comes from `MSP_SET_RAW_RC`, as on the drone; ARM is AUX1 (1700-2100).
- The image is x86_64 only: on arm64 Linux, SITL's gyro and RX tasks never run.
  On Apple Silicon it runs under Docker's emulation.

```bash
docker build -t betaflight-sitl:2026.6.2 docker/sitl
docker run --rm -p 5761:5761 -p 5762:5762 betaflight-sitl:2026.6.2
```

The platform opens a serial device, so bridge one to the TCP port:

```bash
pixi exec -s socat socat pty,raw,echo=0,link=/tmp/ttyBF tcp:127.0.0.1:5761
```

Then run the platform with `device: /tmp/ttyBF` and `imu.acc_1g: 256.0`:
SITL's virtual accelerometer reads 256 at 1 g, where real 16 g sensors read 2048.

Leave `rc_hz` at 0. With an MSP receiver, the RC read back from the FC echoes the
platform's own commands, and the RC arm and offboard switches would act on them.
