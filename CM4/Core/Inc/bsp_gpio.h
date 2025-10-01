#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include "main.h"

/**
 * @brief Inicializa todos los pines de la aplicación según la tabla de configuración.
 * @note  Esta función debe ser llamada después de HAL_Init() y la configuración del reloj.
 */
void bsp_gpio_init(void);

#endif // BSP_GPIO_H
