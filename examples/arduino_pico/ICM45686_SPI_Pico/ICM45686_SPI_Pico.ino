/*
 * ICM-45686 SPI orientation sample for Raspberry Pi Pico.
 *
 * Arduino IDE:
 *   Core: Raspberry Pi Pico/RP2040 by Earle F. Philhower, III
 *   Board: Raspberry Pi Pico
 *   Libraries: ICM45686 by TDK InvenSense, Madgwick by Arduino
 *
 * Wiring (Pico physical pin -> breakout header):
 *   GP16, pin 21 SPI0 RX  -> J2.3 MISO
 *   GP17, pin 22 SPI0 CSn -> J1.1 CS
 *   GP18, pin 24 SPI0 SCK -> J1.2 SCLK
 *   GP19, pin 25 SPI0 TX  -> J2.2 MOSI
 *   GND,  pin 23          -> J2.1 GND
 *   3V3(OUT), pin 36      -> J1.6 VDD and J1.7 VDDIO
 *
 * Do not connect 5 V. INT1, INT2, AUX, and J2.7 (NC) are unused.
 * LED: ON while streaming; blinking means a startup error.
 *
 * Serial protocol:
 *   Output:  ORI,mode,roll,pitch,yaw
 *   Command: FILTER,0 (gyro), FILTER,1 (Madgwick), FILTER,2 (Kalman)
 */

#if !defined(ARDUINO_ARCH_RP2040)
#error "Select Raspberry Pi Pico/RP2040 by Earle F. Philhower, III in Arduino IDE."
#endif

#include <ICM45686.h>

#include "ImuConfig.h"
#include "OrientationFusion.h"
#include "SerialProtocol.h"

// The TDK driver controls CS during each SPI transfer.
ICM456xx imu(SPI, ImuConfig::PIN_CS, ImuConfig::SPI_HZ);
OrientationFusion orientationFusion;
SerialProtocol serialProtocol;
uint32_t nextSampleUs = 0;

[[noreturn]] void haltWithError(const char *message, int status = 0) {
  while (true) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.print("ERROR: ");
    Serial.print(message);
    if (status != 0) {
      Serial.print(" (status ");
      Serial.print(status);
      Serial.print(")");
    }
    Serial.println();
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(800);
  }
}

void configurePicoSpiPins() {
  // These calls must occur before imu.begin(), which calls SPI.begin().
  if (!SPI.setRX(ImuConfig::PIN_MISO)) {
    haltWithError("SPI.setRX(GP16) failed");
  }
  if (!SPI.setTX(ImuConfig::PIN_MOSI)) {
    haltWithError("SPI.setTX(GP19) failed");
  }
  if (!SPI.setSCK(ImuConfig::PIN_SCK)) {
    haltWithError("SPI.setSCK(GP18) failed");
  }
}

void initializeImu() {
  Serial.println("ICM-45686 Raspberry Pi Pico SPI test");

  int status = imu.begin();
  if (status != 0) {
    haltWithError("imu.begin() failed; check 3.3 V, GND, and SPI wiring", status);
  }

  status = imu.startAccel(ImuConfig::ODR_HZ, ImuConfig::ACCEL_FSR_G);
  if (status != 0) {
    haltWithError("imu.startAccel() failed", status);
  }

  status = imu.startGyro(ImuConfig::ODR_HZ, ImuConfig::GYRO_FSR_DPS);
  if (status != 0) {
    haltWithError("imu.startGyro() failed", status);
  }

  delay(100);  // Covers the documented gyro startup time.
}

bool sampleIsDue() {
  const uint32_t nowUs = micros();
  if (static_cast<int32_t>(nowUs - nextSampleUs) < 0) {
    return false;
  }

  nextSampleUs += ImuConfig::SAMPLE_PERIOD_US;
  if (static_cast<int32_t>(nowUs - nextSampleUs) >
      static_cast<int32_t>(ImuConfig::SAMPLE_PERIOD_US)) {
    nextSampleUs = nowUs + ImuConfig::SAMPLE_PERIOD_US;
  }
  return true;
}

int readImuSample(ImuSample &sample) {
  inv_imu_sensor_data_t rawData;
  const int status = imu.getDataFromRegisters(rawData);
  if (status != 0) {
    return status;
  }

  sample.accelXG =
      rawData.accel_data[0] * ImuConfig::ACCEL_FSR_G / ImuConfig::RAW_FULL_SCALE;
  sample.accelYG =
      rawData.accel_data[1] * ImuConfig::ACCEL_FSR_G / ImuConfig::RAW_FULL_SCALE;
  sample.accelZG =
      rawData.accel_data[2] * ImuConfig::ACCEL_FSR_G / ImuConfig::RAW_FULL_SCALE;
  sample.gyroXDps =
      rawData.gyro_data[0] * ImuConfig::GYRO_FSR_DPS / ImuConfig::RAW_FULL_SCALE;
  sample.gyroYDps =
      rawData.gyro_data[1] * ImuConfig::GYRO_FSR_DPS / ImuConfig::RAW_FULL_SCALE;
  sample.gyroZDps =
      rawData.gyro_data[2] * ImuConfig::GYRO_FSR_DPS / ImuConfig::RAW_FULL_SCALE;
  return 0;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  const uint32_t serialWaitStartMs = millis();
  while (!Serial && millis() - serialWaitStartMs < 2000) {
    // Give USB serial time to enumerate without blocking forever.
  }

  configurePicoSpiPins();
  initializeImu();
  orientationFusion.begin(ImuConfig::ODR_HZ);

  nextSampleUs = micros();
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Streaming orientation: ORI,mode,roll,pitch,yaw");
}

void loop() {
  serialProtocol.poll(Serial, orientationFusion);
  if (!sampleIsDue()) {
    return;
  }

  ImuSample sample;
  const int status = readImuSample(sample);
  if (status != 0) {
    Serial.print("read error: ");
    Serial.println(status);
    delay(100);
    return;
  }

  const OrientationAngles angles =
      orientationFusion.update(sample, ImuConfig::SAMPLE_PERIOD_SEC);
  SerialProtocol::writeOrientation(Serial, orientationFusion.mode(), angles);
}
