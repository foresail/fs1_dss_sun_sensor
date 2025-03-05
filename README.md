# Foresail-1 DSS Sun Sensor

This repository contains the public available design files for the DSS sun sensor designed for the Foresail-1 Cubesat.

![DSS Sun Sensor v4](docs/dss_v4.jpg)

The general characteristics of sensor design are:
- Dimensions: 20.4 mm × 24.8 mm × 7 mm
- Mass: 4.4 grams
- Panel mountedable into 14x14 mm hole with two M2 screws.
- Electrical connection by soldering on the panel PCB or by harness.
- FOV: ~30 degrees, resolution: 0.027°, accuracy: ±0.008°
- Refresh rate up to 38 Hz. (Sampling time: 15 ms + communication overhead)
- Can provide following data products:
    - Raw current measurements
    - Point & intensity measurements
    - Sun vector & intensity measurements
    - Angles & intensity measurements
- I2C digital serial interface (hardware version 4)
- RS-485 digital interface (hardware version 5)
- Low power consumption < 4 mW when active
- Nominal operating voltage: 3.6 - 3.8V: Can be extened easily to 2.0 – 5.5 V.

## Documentation

- [FS1 ADCS DSS Sun Sensor Design Document](docs/FS1p_ADCS_DSS_Digital_Sun_Sensor_Hardware_Design.pdf)
- [Version 4 schematic (I2C variant)](v4/dss_v4_schema.pdf)
- [Version 4 firmware source code](v4/fw)
- [Version 5 schematic (RS-485 variant)](v5/dss_v5_schema.pdf)
- [Version 5 firmware source code](v5/fw)
- [Calibration and test tools for PC](calibration)
- [Mechanical design files](mechanical)

## Licence

The design is licenced under [CERN Open Hardware Licence Version 2](LICENCE) 