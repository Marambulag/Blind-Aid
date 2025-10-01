#include "drv_vl53l5x.h"

// --- Funciones Privadas (static) para comunicación de bajo nivel ---
// Estas funciones son el puente entre nuestro driver y el MCAL (ST HAL)

static int8_t vl5_write_reg(VL5_Handle_t *dev, uint16_t reg, uint8_t *data, uint32_t len) {
    if (HAL_I2C_Mem_Write(dev->i2c_handle, dev->i2c_address, reg, I2C_MEMADD_SIZE_16BIT, data, len, 100) == HAL_OK) {
        return 0; // Éxito
    }
    return -1; // Error
}

static int8_t vl5_read_reg(VL5_Handle_t *dev, uint16_t reg, uint8_t *data, uint32_t len) {
    if (HAL_I2C_Mem_Read(dev->i2c_handle, dev->i2c_address, reg, I2C_MEMADD_SIZE_16BIT, data, len, 100) == HAL_OK) {
        return 0; // Éxito
    }
    return -1; // Error
}

// --- Implementación de la API Pública ---

int8_t vl5_init(VL5_Handle_t *dev, VL5_I2C_Handle_t *i2c_handle) {
    dev->i2c_handle = i2c_handle;
    dev->i2c_address = 0x52; // Dirección I2C por defecto del sensor
    dev->is_ranging = 0;

    // Aquí iría la secuencia completa de inicialización del sensor, que implica
    // escribir en múltiples registros para configurar resolución, frecuencia, etc.
    // Por simplicidad, solo verificamos si el dispositivo responde.
    if (HAL_I2C_IsDeviceReady(dev->i2c_handle, dev->i2c_address, 2, 100) != HAL_OK) {
        return -1; // El sensor no está conectado o no responde
    }

    // ... Secuencia de inicialización del sensor ...

    return 0; // Inicialización exitosa
}

int8_t vl5_start_ranging(VL5_Handle_t *dev) {
    // Aquí se escribiría el comando para iniciar la medición continua
    // Esta es una implementación de ejemplo, los registros reales son diferentes.
    uint8_t cmd = 0x01;
    if (vl5_write_reg(dev, 0x0042, &cmd, 1) == 0) {
        dev->is_ranging = 1;
        return 0;
    }
    return -1;
}

int8_t vl5_check_data_ready(VL5_Handle_t *dev, uint8_t *is_ready) {
    // Aquí se leería un registro de estado del sensor
    return vl5_read_reg(dev, 0x0043, is_ready, 1);
}

int8_t vl5_get_ranging_data(VL5_Handle_t *dev, uint16_t *data_matrix_64) {
    // Aquí se leería el bloque de 128 bytes (64 zonas * 2 bytes/zona)
    uint8_t buffer[128];
    if (vl5_read_reg(dev, 0x1000, buffer, 128) != 0) {
        return -1;
    }

    // Convertir el buffer de bytes a un array de uint16_t
    for (int i = 0; i < 64; i++) {
        data_matrix_64[i] = (buffer[i*2] << 8) | buffer[i*2 + 1];
    }
    return 0;
}
