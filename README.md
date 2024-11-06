# as2_platform_betaflight

[Aerostack2](https://aerostack2.github.io/) Aerial platform for betaflight drones.

## Betaflight Information

This code is tested with betaflight 4.5.0 and uses the MSP protocol to communicate with the drone.

```
cd aerostack2_ws/src
git clone git@github.com:CVAR-A2RL/msp.git
as2 build
```

### Betaflight configuration
Channels:
    - 0: Roll
    - 1: Pitch
    - 2: Throttle
    - 3: Yaw
    - 4: Arm
    - 5: Mode
    - 6: Aux1
    - 7: Aux2

