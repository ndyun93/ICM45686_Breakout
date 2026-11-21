/*
 * Raspberry Pi Pico SDK example for the ICM-45686 SPI breakout.
 *
 * Wiring (SPI0): MISO=GP16, CS=GP17, SCLK=GP18, MOSI=GP19.
 * Power both VDD and VDDIO from the Pico 3V3(OUT) pin, and connect GND.
 */

#include <stdio.h>
#include <string.h>

#include "hardware/spi.h"
#include "pico/stdlib.h"

#define ICM_SPI spi0
#define ICM_PIN_MISO 16
#define ICM_PIN_CS   17
#define ICM_PIN_SCK  18
#define ICM_PIN_MOSI 19

#define ICM_SPI_HZ 1000000u

#define ICM_REG_ACCEL_DATA_X1_UI 0x00
#define ICM_REG_PWR_MGMT0        0x10
#define ICM_REG_ACCEL_CONFIG0    0x1B
#define ICM_REG_GYRO_CONFIG0     0x1C
#define ICM_REG_WHO_AM_I         0x72

#define ICM_WHO_AM_I_VALUE 0xE9
#define ICM_READ_BIT       0x80

// 16 g / 100 Hz and 2000 dps / 100 Hz. Values are from TDK's register map.
#define ICM_ACCEL_CONFIG_16G_100HZ 0x19
#define ICM_GYRO_CONFIG_2000DPS_100HZ 0x19
#define ICM_PWR_GYRO_LN_ACCEL_LN 0x0F

static void icm_write_register(uint8_t reg, uint8_t value) {
    const uint8_t tx[2] = {reg, value};
    gpio_put(ICM_PIN_CS, 0);
    spi_write_blocking(ICM_SPI, tx, sizeof(tx));
    gpio_put(ICM_PIN_CS, 1);
}

static void icm_read_registers(uint8_t reg, uint8_t *data, size_t length) {
    // This example only reads up to the 14 contiguous sensor-data bytes.
    uint8_t tx[15] = {0};
    uint8_t rx[15] = {0};

    tx[0] = reg | ICM_READ_BIT;
    gpio_put(ICM_PIN_CS, 0);
    spi_write_read_blocking(ICM_SPI, tx, rx, length + 1);
    gpio_put(ICM_PIN_CS, 1);

    memcpy(data, &rx[1], length);
}

static int16_t little_endian_i16(const uint8_t *bytes) {
    return (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
}

static void icm_init_spi(void) {
    spi_init(ICM_SPI, ICM_SPI_HZ);
    gpio_set_function(ICM_PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(ICM_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(ICM_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_init(ICM_PIN_CS);
    gpio_set_dir(ICM_PIN_CS, GPIO_OUT);
    gpio_put(ICM_PIN_CS, 1);

    // TDK's official Arduino driver uses SPI mode 3: MSB-first, CPOL=1, CPHA=1.
    spi_set_format(ICM_SPI, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);  // Allow USB serial to enumerate.

    icm_init_spi();
    sleep_ms(10);

    uint8_t who_am_i = 0;
    icm_read_registers(ICM_REG_WHO_AM_I, &who_am_i, 1);
    printf("ICM-45686 SPI test, WHO_AM_I=0x%02X\n", who_am_i);
    if (who_am_i != ICM_WHO_AM_I_VALUE) {
        printf("Expected 0x%02X. Check 3.3 V, GND, CS/SCLK/MOSI/MISO, and SPI mode 3.\n",
               ICM_WHO_AM_I_VALUE);
        while (true) {
            sleep_ms(1000);
        }
    }

    icm_write_register(ICM_REG_ACCEL_CONFIG0, ICM_ACCEL_CONFIG_16G_100HZ);
    icm_write_register(ICM_REG_GYRO_CONFIG0, ICM_GYRO_CONFIG_2000DPS_100HZ);
    icm_write_register(ICM_REG_PWR_MGMT0, ICM_PWR_GYRO_LN_ACCEL_LN);
    sleep_ms(100);  // Gyro startup time is documented as up to 70 ms.

    printf("Streaming at 100 Hz: accel[g], gyro[dps], temperature[C]\n");
    while (true) {
        uint8_t raw[14];
        icm_read_registers(ICM_REG_ACCEL_DATA_X1_UI, raw, sizeof(raw));

        // Sensor registers default to little-endian data format.
        const int16_t accel_x = little_endian_i16(&raw[0]);
        const int16_t accel_y = little_endian_i16(&raw[2]);
        const int16_t accel_z = little_endian_i16(&raw[4]);
        const int16_t gyro_x = little_endian_i16(&raw[6]);
        const int16_t gyro_y = little_endian_i16(&raw[8]);
        const int16_t gyro_z = little_endian_i16(&raw[10]);
        const int16_t temperature = little_endian_i16(&raw[12]);

        printf("AccelX:%.4f,AccelY:%.4f,AccelZ:%.4f,"
               "GyroX:%.3f,GyroY:%.3f,GyroZ:%.3f,Temperature:%.2f\n",
               accel_x * 16.0f / 32768.0f,
               accel_y * 16.0f / 32768.0f,
               accel_z * 16.0f / 32768.0f,
               gyro_x * 2000.0f / 32768.0f,
               gyro_y * 2000.0f / 32768.0f,
               gyro_z * 2000.0f / 32768.0f,
               25.0f + temperature / 128.0f);

        sleep_ms(10);
    }
}
