// File: bme280.h
#ifndef BME280_H_
#define BME280_H_

#include "stm32f1xx_hal.h" // hoặc thay bằng dòng phù hợp với dòng STM32 bạn đang dùng
#include <stdint.h>

typedef struct {
    int32_t t_fine;

    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;

    I2C_HandleTypeDef *hi2c;
    uint8_t dev_addr;
} BME280_HandleTypedef;

uint8_t BME280_Init(BME280_HandleTypedef *bme280);
uint8_t BME280_ReadTemperature(BME280_HandleTypedef *bme280, float *temperature);
uint8_t BME280_ReadPressure(BME280_HandleTypedef *bme280, float *pressure);
uint8_t BME280_ReadHumidity(BME280_HandleTypedef *bme280, float *humidity);

#endif // BME280_H_
