#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <SPI.h>
#include <SD.h>

// ==============================
// CONFIGURACIÓN DE PINES Y CONSTANTES
// ==============================
#define SENSOR_PIN 35
#define VOLTAJE_PIN 34
#define TIEMPO_CALIBRACION_MS 10000  // 10 segundos de calibración

const float DIVISOR_FACTOR = 5.057;  // Ajustado para que 11V leídos correspondan a 12V reales

// ==============================
// VARIABLES PARA SENSOR ANALÓGICO
// ==============================
unsigned long tiempoInicio = 0;
bool calibrado = false;
float VREF = 0.0;
float sensibilidad = 0.0267;  // Valor por defecto; se ajusta en el monitor (convertido de mV/A a V/A)
const float CORRIENTE_IDLE = 0.164;  // Corriente en reposo

// Buffers y variables para filtrado de la señal
const uint8_t N_MEDIANA = 15;
float bufferMediana[N_MEDIANA] = {0.0};
uint8_t idxMediana = 0;

const uint8_t N_MEDIA = 27;
float bufferMedia[N_MEDIA] = {0.0};
uint8_t idxMedia = 0;
float sumaMedia = 0;

const uint8_t N_MEDIANA_VOLT = 7;
float bufferMedianaVolt[N_MEDIANA_VOLT] = {0.0};
uint8_t idxMedianaVolt = 0;

float corrienteIIR = 0.0;
const float alphaIIR = 0.2;

float sumaVref = 0.0;
unsigned long muestrasVref = 0;

// ==============================
// CONFIGURACIÓN CAN
// ==============================
CAN_device_t CAN_cfg;
const int rx_queue_size = 10;

// Variables globales para almacenar datos CAN actualizados
// Mensaje 0x290
uint8_t last_torque = 0, last_duty = 0, last_corriente = 0, last_switch = 0, last_temp = 0;
float last_voltaje_CAN = 0.0; // Calculado como data[3]*0.1
// Mensaje 0x292
uint8_t last_angle_dir = 0, last_mapa = 0, last_error = 0;

// ==============================
// FUNCIONES AUXILIARES
// ==============================
float medianaN(float* arr, uint8_t size) {
  float temp[size];
  memcpy(temp, arr, size * sizeof(float));
  for (uint8_t i = 0; i < size - 1; i++) {
    for (uint8_t j = i + 1; j < size; j++) {
      if (temp[j] < temp[i]) {
        float t = temp[i];
        temp[i] = temp[j];
        temp[j] = t;
      }
    }
  }
  return temp[size / 2];
}

// Procesa mensajes CAN: actualiza variables globales de los mensajes 0x290 y 0x292
void processCANMessages() {
  CAN_frame_t rx_frame;
  while (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 0) == pdTRUE) {
    if (rx_frame.MsgID == 0x290) {
      last_torque = rx_frame.data.u8[0];
      last_duty = rx_frame.data.u8[1];
      last_corriente = rx_frame.data.u8[2];
      last_voltaje_CAN = rx_frame.data.u8[3] * 0.1;  // Conversión a voltaje
      last_switch = rx_frame.data.u8[4];
      last_temp = rx_frame.data.u8[5];
    }
    else if (rx_frame.MsgID == 0x292) {
      last_angle_dir = rx_frame.data.u8[0];
      last_mapa = rx_frame.data.u8[3];
      last_error = rx_frame.data.u8[4];
    }
  }
}

// ==============================
// SETUP: CONFIGURACIÓN INICIAL
// ==============================
void setup() {
  Serial.begin(576000);
  analogReadResolution(12);
  
  // Solicitar la sensibilidad del sensor (en mV/A) y convertir a V/A
  Serial.println("Introduce la sensibilidad del sensor en mV/A y pulsa ENTER:");
  while (Serial.available() == 0) { delay(10); }
  sensibilidad = Serial.parseFloat() / 1000.0;
  Serial.print("Sensibilidad establecida: ");
  Serial.print(sensibilidad, 6);
  Serial.println(" V/A");
  
  // Iniciar calibración del sensor
  Serial.println("⚙️ Calibrando VREF durante 10 segundos, mantén el motor en idle...");
  tiempoInicio = millis();
  
  // Configurar CAN
  CAN_cfg.speed = CAN_SPEED_250KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_27;
  CAN_cfg.rx_pin_id = GPIO_NUM_26;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  ESP32Can.CANInit();
  Serial.printf("✅ CAN SPEED: %d kbps\n", CAN_cfg.speed);
}

// ==============================
// LOOP: PROCESAMIENTO PERIÓDICO
// ==============================
void loop() {
  unsigned long ahora = millis();
  
  // Procesar mensajes CAN y actualizar variables globales
  processCANMessages();
  
  // Lectura analógica del sensor de corriente
  int lecturaADC = analogRead(SENSOR_PIN);
  float voltaje = (lecturaADC / 4095.0) * 3.3;
  
  // Calibración de VREF durante TIEMPO_CALIBRACION_MS
  if (!calibrado) {
    sumaVref += voltaje;
    muestrasVref++;
    if (ahora - tiempoInicio >= TIEMPO_CALIBRACION_MS) {
      VREF = sumaVref / muestrasVref;
      calibrado = true;
      Serial.print("✅ Calibración completada. VREF = ");
      Serial.print(VREF, 4);
      Serial.println(" V");
      delay(1000);
    }
    return; // Hasta que se complete la calibración, no sigue
  }
  else {
    // Cálculos analógicos
    float corriente = (voltaje - VREF) / sensibilidad + CORRIENTE_IDLE;
    corrienteIIR = alphaIIR * corriente + (1 - alphaIIR) * corrienteIIR;
    
    bufferMediana[idxMediana] = voltaje;
    idxMediana = (idxMediana + 1) % N_MEDIANA;
    float voltajeFiltrado = medianaN(bufferMediana, N_MEDIANA);
    float corrienteFiltrada1 = (voltajeFiltrado - VREF) / sensibilidad + CORRIENTE_IDLE;
    
    sumaMedia -= bufferMedia[idxMedia];
    bufferMedia[idxMedia] = corrienteFiltrada1;
    sumaMedia += corrienteFiltrada1;
    idxMedia = (idxMedia + 1) % N_MEDIA;
    float corrienteFiltrada2 = sumaMedia / N_MEDIA;
    
    int lecturaVoltajeADC = analogRead(VOLTAJE_PIN);
    float voltajeADC = (lecturaVoltajeADC / 4095.0) * 3.3;
    bufferMedianaVolt[idxMedianaVolt] = voltajeADC;
    idxMedianaVolt = (idxMedianaVolt + 1) % N_MEDIANA_VOLT;
    float voltajeADC_med = medianaN(bufferMedianaVolt, N_MEDIANA_VOLT);
    float voltajeReal = voltajeADC_med * DIVISOR_FACTOR;
    
    // Imprimir línea CSV: Primero datos analógicos y luego datos CAN (0x290 y 0x292)
    // Datos analógicos:
    Serial.print(voltaje, 4); Serial.print(",");
    Serial.print(corriente, 4); Serial.print(",");
    Serial.print(voltajeFiltrado, 4); Serial.print(",");
    Serial.print(corrienteFiltrada1, 4); Serial.print(",");
    Serial.print(corrienteFiltrada2, 4); Serial.print(",");
    Serial.print(corrienteIIR, 4); Serial.print(",");
    Serial.print(voltajeReal, 3);
    // Datos CAN (mensaje 0x290):
    Serial.print(",");
    Serial.print(last_torque);
    Serial.print(",");
    Serial.print(last_duty);
    Serial.print(",");
    Serial.print(last_corriente);
    Serial.print(",");
    Serial.print(last_voltaje_CAN, 1);
    Serial.print(",");
    Serial.print(last_switch);
    Serial.print(",");
    Serial.print(last_temp);
    // Datos CAN (mensaje 0x292):
    Serial.print(",");
    Serial.print(last_angle_dir);
    Serial.print(",");
    Serial.print(last_mapa);
    Serial.print(",");
    Serial.println(last_error);
  }
}
