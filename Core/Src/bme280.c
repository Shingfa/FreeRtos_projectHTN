// File: bme280.c
#include "bme280.h"
#include <string.h>
#include <math.h>

#define BME280_REG_ID          0xD0
#define BME280_REG_RESET       0xE0
#define BME280_REG_CTRL_HUM    0xF2
#define BME280_REG_STATUS      0xF3
#define BME280_REG_CTRL_MEAS   0xF4
#define BME280_REG_CONFIG      0xF5
#define BME280_REG_PRESS_MSB   0xF7
#define BME280_REG_CALIB00     0x88
#define BME280_REG_CALIB26     0xE1

static uint8_t BME280_Read8(BME280_HandleTypedef *bme, uint8_t reg) {
    uint8_t value;
    HAL_I2C_Mem_Read(bme->hi2c, bme->dev_addr, reg, 1, &value, 1, 100);
    return value;
}

static void BME280_ReadCalibrationData(BME280_HandleTypedef *bme) {
    uint8_t calib[26];
    HAL_I2C_Mem_Read(bme->hi2c, bme->dev_addr, BME280_REG_CALIB00, 1, calib, 26, 100);

    bme->dig_T1 = (calib[1] << 8) | calib[0];
    bme->dig_T2 = (calib[3] << 8) | calib[2];
    bme->dig_T3 = (calib[5] << 8) | calib[4];

    bme->dig_P1 = (calib[7] << 8) | calib[6];
    bme->dig_P2 = (calib[9] << 8) | calib[8];
    bme->dig_P3 = (calib[11] << 8) | calib[10];
    bme->dig_P4 = (calib[13] << 8) | calib[12];
    bme->dig_P5 = (calib[15] << 8) | calib[14];
    bme->dig_P6 = (calib[17] << 8) | calib[16];
    bme->dig_P7 = (calib[19] << 8) | calib[18];
    bme->dig_P8 = (calib[21] << 8) | calib[20];
    bme->dig_P9 = (calib[23] << 8) | calib[22];

    bme->dig_H1 = BME280_Read8(bme, 0xA1);

    uint8_t calib26[7];
    HAL_I2C_Mem_Read(bme->hi2c, bme->dev_addr, BME280_REG_CALIB26, 1, calib26, 7, 100);

    bme->dig_H2 = (calib26[1] << 8) | calib26[0];
    bme->dig_H3 = calib26[2];
    bme->dig_H4 = (calib26[3] << 4) | (calib26[4] & 0x0F);
    bme->dig_H5 = (calib26[5] << 4) | (calib26[4] >> 4);
    bme->dig_H6 = calib26[6];
}

uint8_t BME280_Init(BME280_HandleTypedef *bme) {
    if (BME280_Read8(bme, BME280_REG_ID) != 0x60)
        return 0;

    HAL_I2C_Mem_Write(bme->hi2c, bme->dev_addr, BME280_REG_RESET, 1, (uint8_t[]){0xB6}, 1, 100);
    HAL_Delay(100);

    BME280_ReadCalibrationData(bme);

    HAL_I2C_Mem_Write(bme->hi2c, bme->dev_addr, BME280_REG_CTRL_HUM, 1, (uint8_t[]){0x01}, 1, 100);
    HAL_I2C_Mem_Write(bme->hi2c, bme->dev_addr, BME280_REG_CTRL_MEAS, 1, (uint8_t[]){0x27}, 1, 100);
    HAL_I2C_Mem_Write(bme->hi2c, bme->dev_addr, BME280_REG_CONFIG, 1, (uint8_t[]){0xA0}, 1, 100);

    return 1;
}

uint8_t BME280_ReadTemperature(BME280_HandleTypedef *bme, float *temperature) {
    uint8_t data[3];
    HAL_I2C_Mem_Read(bme->hi2c, bme->dev_addr, 0xFA, 1, data, 3, 100);
    int32_t adc_T = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);

    int32_t var1 = ((((adc_T >> 3) - ((int32_t)bme->dig_T1 << 1))) * ((int32_t)bme->dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)bme->dig_T1)) * ((adc_T >> 4) - ((int32_t)bme->dig_T1))) >> 12) * ((int32_t)bme->dig_T3)) >> 14;

    bme->t_fine = var1 + var2;
    *temperature = (bme->t_fine * 5 + 128) >> 8;
    *temperature /= 100.0f;
    return 1;
}

uint8_t BME280_ReadPressure(BME280_HandleTypedef *bme, float *pressure) {
    uint8_t data[3];
    HAL_I2C_Mem_Read(bme->hi2c, bme->dev_addr, 0xF7, 1, data, 3, 100);
    int32_t adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);

    int64_t var1 = ((int64_t)bme->t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)bme->dig_P6;
    var2 = var2 + ((var1 * (int64_t)bme->dig_P5) << 17);
    var2 = var2 + (((int64_t)bme->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bme->dig_P3) >> 8) + ((var1 * (int64_t)bme->dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bme->dig_P1) >> 33;

    if (var1 == 0) return 0;

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bme->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bme->dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)bme->dig_P7) << 4);
    *pressure = p / 256.0f;
    return 1;
}

uint8_t BME280_ReadHumidity(BME280_HandleTypedef *bme, float *humidity) {
    uint8_t data[2];
    HAL_I2C_Mem_Read(bme->hi2c, bme->dev_addr, 0xFD, 1, data, 2, 100);
    int32_t adc_H = (data[0] << 8) | data[1];

    int32_t v_x1_u32r = bme->t_fine - 76800;
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)bme->dig_H4) << 20) - (((int32_t)bme->dig_H5) * v_x1_u32r)) + 16384) >> 15) *
                (((((((v_x1_u32r * ((int32_t)bme->dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)bme->dig_H3)) >> 11) + 32768)) >> 10) + 2097152) *
                ((int32_t)bme->dig_H2) + 8192) >> 14));

    v_x1_u32r = v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)bme->dig_H1)) >> 4);
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    *humidity = (v_x1_u32r >> 12) / 1024.0f;
    return 1;
}
