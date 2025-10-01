#include "bsp_gpio.h"

// Estructura privada que describe completamente la configuración de un pin.
typedef struct {
    GPIO_TypeDef* port;
    uint16_t      pin;
    uint32_t      mode;
    uint32_t      pull;
    uint32_t      speed;
    uint32_t      alternate;
} PinConfig_t;

// --- TABLA DE CONFIGURACIÓN DE PINES ---
// ÚNICO LUGAR a modificar si el hardware cambia.
static const PinConfig_t AppPinConfigs[] = {
    // Configuración para el pin SDA del I2C1 del sensor ToF
    {
        .port      = GPIOB,
        .pin       = GPIO_PIN_9,
        .mode      = GPIO_MODE_AF_OD,       // Alternate Function, Open Drain (requerido para I2C)
        .pull      = GPIO_PULLUP,           // Pull-up interno habilitado
        .speed     = GPIO_SPEED_FREQ_HIGH,
        .alternate = GPIO_AF4_I2C1          // Función alternativa específica para I2C1 en PB9
    },
    // Configuración para el pin SCL del I2C1 del sensor ToF
    {
        .port      = GPIOB,
        .pin       = GPIO_PIN_8,
        .mode      = GPIO_MODE_AF_OD,
        .pull      = GPIO_PULLUP,
        .speed     = GPIO_SPEED_FREQ_HIGH,
        .alternate = GPIO_AF4_I2C1          // Función alternativa específica para I2C1 en PB8
    }
};

void bsp_gpio_init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    const uint32_t num_pins = sizeof(AppPinConfigs) / sizeof(PinConfig_t);

    // 1. Habilitar los clocks de los puertos GPIO necesarios
    __HAL_RCC_GPIOB_CLK_ENABLE();
    // __HAL_RCC_GPIOA_CLK_ENABLE(); // Habilitar otros si fueran necesarios

    // 2. Iterar sobre la tabla de configuración y inicializar cada pin
    for (uint32_t i = 0; i < num_pins; i++) {
        GPIO_InitStruct.Pin       = AppPinConfigs[i].pin;
        GPIO_InitStruct.Mode      = AppPinConfigs[i].mode;
        GPIO_InitStruct.Pull      = AppPinConfigs[i].pull;
        GPIO_InitStruct.Speed     = AppPinConfigs[i].speed;
        GPIO_InitStruct.Alternate = AppPinConfigs[i].alternate;

        HAL_GPIO_Init(AppPinConfigs[i].port, &GPIO_InitStruct);
    }
}
