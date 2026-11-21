# ICM-45686 SPI sample programs

These examples are for the ICM-45686 SPI breakout in this repository. They initialize the sensor, run selectable six-axis orientation filters at 100 Hz, and send orientation to a Processing 4 viewer.

## Electrical connection

Use **3.3 V only**. Connect both supply rails; VDDIO is the digital I/O voltage, so a 5 V Arduino board must not be wired directly to this breakout.

| Breakout header | Signal | Raspberry Pi Pico |
|---|---|---:|
| J1.1 | CS | GP17 |
| J1.2 | SCLK | GP18 |
| J2.2 | MOSI | GP19 |
| J2.3 | MISO | GP16 |
| J2.1 | GND | GND |
| J1.6 | VDD | 3V3(OUT) |
| J1.7 | VDDIO | 3V3(OUT) |

`INT1` and `INT2` are not used by these polling examples.

## Before powering the board

1. Hand-solder J1 and J2 from the Bottom side as documented in `PCBA_PREFLIGHT.md`.
2. Check VDD-to-GND and VDDIO-to-GND resistance with power disconnected.
3. Verify there is no 5 V connection to VDD, VDDIO, or an SPI signal.
4. Use only one host interface. If changing between I2C and SPI sketches, completely remove power from the IMU first.

## Included examples

- `arduino_pico/ICM45686_SPI_Pico/`: Arduino IDE sketch specifically for a Raspberry Pi Pico. Install the `Raspberry Pi Pico/RP2040` board package by Earle F. Philhower, III, TDK's `ICM45686` library, and Arduino's `Madgwick` library. Select `Raspberry Pi Pico`, then upload `ICM45686_SPI_Pico.ino`.
- `processing4/ICM45686_3D_Viewer/`: Processing 4 sketch that reads `ORI,mode,roll,pitch,yaw` at 115200 baud and rotates a 3D object. Press `R` to make the current orientation the new zero. Click the filter button or press `F` to cycle through Madgwick, Kalman, and gyro-only modes.
- `pico_sdk/`: Raspberry Pi Pico SDK C example with no IMU driver dependency. Set `PICO_SDK_PATH`, configure with CMake, and copy the generated `.uf2` to the Pico in BOOTSEL mode.

The Pico Arduino sketch is split into tabs by responsibility:

- `ICM45686_SPI_Pico.ino`: wiring, IMU initialization, sample acquisition, and `setup()`/`loop()`.
- `ImuConfig.h`: Pico pins, SPI/ODR/range constants, filter mode, and shared data types.
- `AngleKalman.h`: reusable one-axis angle/bias Kalman filter.
- `OrientationFusion.h`: Madgwick, Kalman Roll/Pitch, gyro-only integration, and mode switching.
- `SerialProtocol.h`: `FILTER,0/1/2` command parsing and `ORI,mode,roll,pitch,yaw` output.

The Pico Arduino sketch configures SPI0 as MISO=GP16, SCLK=GP18, MOSI=GP19, while the TDK library controls CS=GP17. It requires the Earle Philhower Arduino-Pico core; the Arduino Mbed RP2040 core does not provide the required `SPI.setRX()`, `SPI.setTX()`, and `SPI.setSCK()` pin-assignment API.

It is a Pico port of TDK's official `Polling_SPI` example with Arduino's Madgwick IMU filter and a two-angle Kalman filter added. The Pico sends `ORI,mode,roll,pitch,yaw` lines at 100 Hz. It accepts `FILTER,0` for gyro only, `FILTER,1` for Madgwick, and `FILTER,2` for Kalman Roll/Pitch fusion.

The SPI protocol is 4-wire, MSB-first, SPI mode 3, and initially runs at 1 MHz. The ICM-45686 data sheet permits up to 24 MHz at the applicable VDDIO condition; increase the speed only after this 1 MHz test is stable.

## Processing 4 viewer

1. Upload the Pico Arduino sketch and close Arduino IDE's Serial Monitor.
2. Open `processing4/ICM45686_3D_Viewer/ICM45686_3D_Viewer.pde` in Processing 4.
3. Press Run. The sketch prints available serial ports and initially selects the last one. If that is not the Pico, change `PORT_INDEX` at the top of the Processing sketch.
4. Keep the board still when the viewer starts. Press `R` at any time to reset the displayed zero orientation.
5. Click the mode button or press `F` to cycle through Madgwick, Kalman, and gyro-only output. The active mode is confirmed by each packet returned from the Pico.

Madgwick fuses all three accelerometer and gyro axes in a quaternion orientation estimate. The Kalman mode uses independent angle/bias filters for Roll and Pitch: gyro rates predict each angle and gravity-derived accelerometer angles correct them. Kalman accelerometer correction is temporarily skipped when acceleration magnitude is outside 0.75-1.25 g so linear motion is less likely to tilt the estimate incorrectly. Its Yaw remains gyro integration because this six-axis board has no absolute heading measurement. Gyro-only mode integrates all three gyro axes and drifts faster. No mode can eliminate Yaw drift without a magnetometer, camera, or another external heading reference. This viewer displays orientation only; accelerometer integration is not accurate enough for stable XYZ position tracking.

## Sources

- Local: `datasheet/ds-000577-icm-45686-datasheet.pdf`, DS-000577 Rev. 1.0: SPI protocol, WHO_AM_I, power mode, ODR, and scaling.
- [TDK ICM45686 Arduino library](https://github.com/tdk-invn-oss/motion.arduino.ICM45686): official `Polling_SPI` example and API.
- [TDK ICM-45605 / ICM-45686 User Guide](https://www.invensense.tdk.com/wp-content/uploads/2024/07/AN-000478_ICM-45605-ICM-45686-User-Guide.pdf).
