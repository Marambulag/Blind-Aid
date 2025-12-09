#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <HardwareSerial.h>

constexpr int NUM_MOTORS = 9;

// Ajusta a tus pines reales
int MOTOR_PIN[NUM_MOTORS] = {
  13, 12, 14,   // fila de arriba: idx 0,1,2
  26, 25,  2,   // fila del medio: idx 3,4,5
   4,  0, 27    // fila de abajo: idx 6,7,8
};

constexpr int PWM_FREQ = 25000;     // 25 kHz
constexpr int PWM_BITS = 10;        // 10 bits → 0..1023
constexpr int PWM_MAX  = (1 << PWM_BITS) - 1;

// --------- UART con Raspberry ----------
// Usamos UART2 con RX en GPIO16 (donde llega el TX de la RPi)
HardwareSerial SerialPi(2);
constexpr int RX_PI = 16;   // <-- CONECTA AQUÍ EL TX DE LA RPi
constexpr int TX_PI = 17;   // pin cualquiera libre; no es necesario conectarlo físicamente

// --------- Estructuras RTOS ----------
struct MotorMsg {
  uint8_t level[NUM_MOTORS];  // valores 0–9
};

QueueHandle_t motorQueue = nullptr;

// --------- Utilidades PWM ----------

inline int clampDuty(int x){
  if(x < 0)        return 0;
  if(x > PWM_MAX)  return PWM_MAX;
  return x;
}

void setMotor(int i, float d01){
  if(i < 0 || i >= NUM_MOTORS) return;
  if(d01 < 0) d01 = 0;
  if(d01 > 1) d01 = 1;
  int duty = int(d01 * PWM_MAX + 0.5f);
  // Estás usando el pin como "canal". Si ya te funcionaba antes, lo dejamos así.
  ledcWrite(MOTOR_PIN[i], duty);
}

void setMotorDuty(int i, int duty){
  ledcWrite(MOTOR_PIN[i], clampDuty(duty));
}

void allOff(){
  for(int i = 0; i < NUM_MOTORS; ++i){
    setMotor(i, 0.0f);
  }
}

void startupSelfTest() {
  const int rowDelayMs = 500;
  const int gapMs = 150;

  delay(rowDelayMs);
  setMotor(0, 0.75f);
  setMotor(1, 0.75f);
  setMotor(2, 0.75f);
  delay(rowDelayMs);
  setMotor(0, 0);
  setMotor(1, 0);
  setMotor(2, 0);

  setMotor(3, 0.75f);
  setMotor(4, 0.75f);
  setMotor(5, 0.75f);
  delay(rowDelayMs);
  setMotor(3, 0);
  setMotor(4, 0);
  setMotor(5, 0);

  setMotor(6, 0.75f);
  setMotor(7, 0.75f);
  setMotor(8, 0.75f);
  delay(rowDelayMs);
  setMotor(6, 0);
  setMotor(7, 0);
  setMotor(8, 0);
  delay(gapMs);
}

// --------- Parser de la línea UART ----------
// Espera algo como: "5 4 0 1 2 7 8 0 9"
bool parseLineToLevels(const String &line, uint8_t outLevels[NUM_MOTORS]){
  int temp[NUM_MOTORS];

  int n = sscanf(line.c_str(),
                 "%d %d %d %d %d %d %d %d %d",
                 &temp[0], &temp[1], &temp[2],
                 &temp[3], &temp[4], &temp[5],
                 &temp[6], &temp[7], &temp[8]);
  if(n != NUM_MOTORS){
    return false; // línea chafa
  }

  for(int i = 0; i < NUM_MOTORS; ++i){
    if(temp[i] < 0) temp[i] = 0;
    if(temp[i] > 9) temp[i] = 9;
    outLevels[i] = (uint8_t)temp[i];
  }
  return true;
}

// --------- Tarea: UART (desde Raspberry) ----------

void TaskUART(void *pv){
  String line;
  MotorMsg msg;
  uint8_t levels[NUM_MOTORS];

  for(;;){
    if(SerialPi.available()){
      line = SerialPi.readStringUntil('\n');
      line.trim();

      if(parseLineToLevels(line, levels)){
        for(int i = 0; i < NUM_MOTORS; ++i){
          msg.level[i] = levels[i];
        }
        xQueueOverwrite(motorQueue, &msg);

        // Log por USB para ver qué está llegando de la RPi
        Serial.print("Linea valida desde RPi: ");
        Serial.println(line);
      }else if(line.length() > 0){
        Serial.print("Linea invalida desde RPi: ");
        Serial.println(line);
      }
    }else{
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}

// --------- Tarea: Motores ----------

// Mínimo activo para niveles > 0
constexpr float MIN_ACTIVE = 0.25f;  // 30% PWM mínimo cuando hay vibración
constexpr float MAX_ACTIVE = 0.80f;  // 100%

void TaskMotors(void *pv){
  MotorMsg msg;
  bool hasData = false;

  allOff();

  for(;;){
    if(xQueueReceive(motorQueue, &msg, pdMS_TO_TICKS(50)) == pdTRUE){
      hasData = true;
      for(int i = 0; i < NUM_MOTORS; ++i){
        uint8_t lvl = msg.level[i];

        if (lvl == 0) {
          // Totalmente apagado
          setMotor(i, 0.0f);
        } else {
          // lvl 1..9 → [MIN_ACTIVE, MAX_ACTIVE]
          float frac = lvl / 9.0f;  // ~0.11 .. 1.0
          float d01  = MIN_ACTIVE + (MAX_ACTIVE - MIN_ACTIVE) * frac;
          if (d01 > 1.0f) d01 = 1.0f;
          setMotor(i, d01);
        }
      }
    }else{
      // Si quieres que se apaguen cuando no hay datos, puedes usar:
      // if(hasData){
      //   allOff();
      //   hasData = false;
      // }
    }
  }
}

// --------- setup / loop ----------

void setup(){
  Serial.begin(115200);
  delay(200);

  Serial.println("Iniciando...");

  // UART hacia la Raspberry Pi (GPIO16 como RX)
  SerialPi.begin(115200, SERIAL_8N1, RX_PI, TX_PI);
  Serial.print("UART con Raspberry iniciado en 115200 (RX=GPIO");
  Serial.print(RX_PI);
  Serial.println(").");

  // Inicializar PWM de motores
  for(int i=0;i<NUM_MOTORS;++i){
    ledcAttach(MOTOR_PIN[i], PWM_FREQ, PWM_BITS);  // tu forma original
    ledcWrite(MOTOR_PIN[i], 0);
  }

  Serial.println("Motores inicializados (con RTOS).");

  // Self-test de arranque
  Serial.println("Self-test de motores (fila por fila)...");
  startupSelfTest();
  Serial.println("Self-test terminado.");

  // Cola de un solo elemento: siempre nos quedamos con el último frame
  motorQueue = xQueueCreate(1, sizeof(MotorMsg));
  if(motorQueue == nullptr){
    Serial.println("Error creando cola de motores :(");
    while(true){
      delay(1000);
    }
  }

  // Task para recibir datos de la Raspberry por UART2
  xTaskCreatePinnedToCore(
    TaskUART,
    "UART_Task",
    4096,
    nullptr,
    1,
    nullptr,
    0
  );

  // Task para refrescar motores
  xTaskCreatePinnedToCore(
    TaskMotors,
    "Motors_Task",
    4096,
    nullptr,
    1,
    nullptr,
    1
  );
}

void loop(){
  vTaskDelay(portMAX_DELAY);
}
