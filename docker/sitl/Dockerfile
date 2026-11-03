# Betaflight SITL: the flight controller firmware as a Linux program, to run the
# platform without hardware. See README.md.
#
# x86_64 only: on arm64 Linux, SITL's gyro and RX tasks never run, so it ignores
# RC. On Apple Silicon the image runs under Docker's emulation.

FROM --platform=linux/amd64 debian:bookworm-slim AS build

ARG BETAFLIGHT_VERSION=2026.6.2

RUN apt-get update \
 && apt-get install -y --no-install-recommends build-essential ca-certificates git \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /src
RUN git clone --depth 1 --branch "${BETAFLIGHT_VERSION}" \
      --recurse-submodules --shallow-submodules \
      https://github.com/betaflight/betaflight.git
RUN make -C betaflight TARGET=SITL OPTIONS=SITL_STATIC -j"$(nproc)" \
 && install -D -s betaflight/obj/main/betaflight_SITL.elf /out/betaflight_sitl

COPY fdm_feed.c .
RUN gcc -O2 -static -s -o /out/fdm_feed fdm_feed.c


FROM --platform=linux/amd64 busybox:1.36.1-musl

COPY --from=build /out/ /usr/bin/
COPY sitl.cfg entrypoint.sh /etc/betaflight/

WORKDIR /data
EXPOSE 5761 5762
ENTRYPOINT ["/bin/sh", "/etc/betaflight/entrypoint.sh"]
