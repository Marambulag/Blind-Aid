#include "app_sensor_manager.h"
#include "cmsis_os.h"
#include "drv_vl53l5x.h"

// Handle de la tarea del RTOS
osThreadId_t sensorTaskHandle;
const osThreadAttr_t sensorTask_attributes = {
  .name = "sensorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

// Declaración de la función de la tarea
static void SensorManagerTask(void *argument);

// Variables del sensor
static VL5_Handle_t tof_sensor;
static uint16_t distance_matrix[64];

// Handle del I2C (definido por CubeMX, usualmente en main.c)
extern I2C_HandleTypeDef hi2c1;

void app_sensor_manager_init(void) {
    sensorTaskHandle = osThreadNew(SensorManagerTask, NULL, &sensorTask_attributes);
}

static void SensorManagerTask(void *argument) {
  // 1. Inicializar el driver del sensor
  if (vl5_init(&tof_sensor, &hi2c1) != 0) {
    // Error: el sensor no se pudo inicializar. La tarea puede eliminarse o intentar de nuevo.
    osThreadTerminate(NULL);
  }

  // 2. Iniciar la medición
  vl5_start_ranging(&tof_sensor);

  // 3. Bucle principal de la tarea
  for(;;) {
    uint8_t is_data_ready = 0;

    // Esperar a que los datos estén listos
    vl5_check_data_ready(&tof_sensor, &is_data_ready);

    if (is_data_ready) {
      // Leer los datos del sensor
      if (vl5_get_ranging_data(&tof_sensor, distance_matrix) == 0) {
        // Éxito: Los datos están en 'distance_matrix'.
        // AQUÍ ES DONDE SE ENVIARÍAN LOS DATOS AL CORE M7
        // a través de la memoria compartida (con HSEM) o OpenAMP.
      }
    }

    // Esperar un tiempo antes de la siguiente comprobación
    osDelay(30); // Ajustar según la frecuencia de muestreo deseada
  }
}
