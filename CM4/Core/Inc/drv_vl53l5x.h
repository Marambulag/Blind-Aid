#ifndef DRV_VL53L5X_H
#define DRV_VL53L5X_H

#include "main.h"

// Usaremos el handle de I2C del MCAL (ST HAL)
typedef I2C_HandleTypeDef VL5_I2C_Handle_t;

// Handle del driver que contiene su estado y configuración
typedef struct {
    VL5_I2C_Handle_t *i2c_handle;
    uint8_t           i2c_address;
    uint8_t           is_ranging;
} VL5_Handle_t;

// API Pública del Driver
int8_t vl5_init(VL5_Handle_t *dev, VL5_I2C_Handle_t *i2c_handle);
int8_t vl5_start_ranging(VL5_Handle_t *dev);
int8_t vl5_check_data_ready(VL5_Handle_t *dev, uint8_t *is_ready);
int8_t vl5_get_ranging_data(VL5_Handle_t *dev, uint16_t *data_matrix_64);

#endif // DRV_VL53L5X_H
