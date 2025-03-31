#include <Arduino.h>
#include "config.h"
#include <HardwareSerial.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <SPI.h>
#include <SD.h>

#define SENSOR_PIN 35
#define VOLTAJE_PIN 34
#define TIEMPO_CALIBRACION_MS 100

#define ENCODER_CLK 32
#define ENCODER_DT  33
#define ENCODER_SW 25

const float DIVISOR_FACTOR = 5.057;

unsigned long tiempoInicio = 0;
bool calibrado = false;
float VREF = 0.0;
float sensibilidad = 0.0406;

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

// ENCODER
int encoderSteps = 100;
int lastCLK = HIGH;

// PULSADOR
int pulsadorState = HIGH;
int lastPulsadorState = HIGH;
int pulsadorValor = 0;

// TEMPORIZADORES
CAN_device_t CAN_cfg;
const int rx_queue_size = 10;
const int interval = 10;

unsigned long previousMedicion = 0;

// Variables de CAN recibidas
uint8_t last_torque = 0;
uint8_t last_duty = 0;
uint8_t last_corriente = 0;
float last_voltaje_CAN = 0.0;
uint8_t last_switch = 0;
uint8_t last_temp = 0;
uint8_t last_angle_dir = 0;
uint8_t last_mapa = 0;
uint8_t last_error = 0;

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

void setup() {
  pinMode(PIN_5V_EN, OUTPUT);
  digitalWrite(PIN_5V_EN, HIGH);

  pinMode(CAN_SE_PIN, OUTPUT);
  digitalWrite(CAN_SE_PIN, LOW);

  Serial.begin(921600);
  analogReadResolution(12);

  pinMode(ENCODER_CLK, INPUT);
  pinMode(ENCODER_DT, INPUT);
  pinMode(ENCODER_SW, INPUT_PULLUP);
  lastCLK = digitalRead(ENCODER_CLK);

  sensibilidad = 0.0406;
  Serial.print("Sensibilidad establecida: ");
  Serial.print(sensibilidad, 6);
  Serial.println(" V/A");
  Serial.println("⚙️ Calibrando VREF durante 10 segundos, mantén el motor en idle...");
  tiempoInicio = millis();

  CAN_cfg.speed = CAN_SPEED_250KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_27;
  CAN_cfg.rx_pin_id = GPIO_NUM_26;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  ESP32Can.CANInit();
}

void loop() {
  unsigned long currentMillis = millis();

  // === CALIBRACIÓN ===
  if (!calibrado) {
    int lecturaADC = analogRead(SENSOR_PIN);
    float voltaje = (lecturaADC / 4095.0) * 3.3;
    sumaVref += voltaje;
    muestrasVref++;
    if (currentMillis - tiempoInicio >= TIEMPO_CALIBRACION_MS) {
      VREF = sumaVref / muestrasVref;
      calibrado = true;
      Serial.print("✅ Calibración completada. VREF = ");
      Serial.print(VREF, 4);
      Serial.println(" V");
      Serial.println("V_sensor(V),I_direct(A),V_mediana(V),I_filtrado1(A),I_filtrado2(A),I_IIR(A),V_real(V),Torque(bits),MotorDuty(%),I_CAN(A),V_CAN(V),SwitchPos,Temp(°C),DirAngle(bits),Mapa,Error,P(W),EncoderSteps,Pulsador");
      delay(1000);
    }
    return;
  }

  // === MEDICIÓN CADA INTERVALO ===
  if (currentMillis - previousMedicion >= interval) {
    previousMedicion = currentMillis;
 // recepción de mensajes CAN
    CAN_frame_t rx_frame;
    while (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 0) == pdTRUE) {
      if (rx_frame.MsgID == 0x290) {
        last_torque      = rx_frame.data.u8[0];
        last_duty        = rx_frame.data.u8[1];
        last_corriente   = rx_frame.data.u8[2];
        last_voltaje_CAN = rx_frame.data.u8[3] * 0.1;
        last_switch      = rx_frame.data.u8[4];
        last_temp        = rx_frame.data.u8[5];
      } else if (rx_frame.MsgID == 0x292) {
        last_angle_dir   = rx_frame.data.u8[0];
        last_mapa        = rx_frame.data.u8[3];
        last_error       = rx_frame.data.u8[4];
      }
    }

    // === ENCODER ===
    int currentCLK = digitalRead(ENCODER_CLK);
    if (currentCLK != lastCLK && currentCLK == LOW) {
      if (digitalRead(ENCODER_DT) != currentCLK) {
        encoderSteps++;
      } else {
        encoderSteps--;
      }
      encoderSteps = constrain(encoderSteps, 10, 250);
    }
    lastCLK = currentCLK;

    // === PULSADOR ===
    pulsadorState = digitalRead(ENCODER_SW);
    if (pulsadorState == LOW && lastPulsadorState == HIGH) {
      pulsadorValor++;
      if (pulsadorValor > 5) pulsadorValor = 0;
    }
    lastPulsadorState = pulsadorState;

    // === CORRIENTE ===
    int lecturaADC = analogRead(SENSOR_PIN);
    float voltaje = (lecturaADC / 4095.0) * 3.3;

    float corriente = (voltaje - VREF) / sensibilidad;
    corrienteIIR = alphaIIR * corriente + (1 - alphaIIR) * corrienteIIR;

    bufferMediana[idxMediana] = voltaje;
    idxMediana = (idxMediana + 1) % N_MEDIANA;
    float voltajeFiltrado = medianaN(bufferMediana, N_MEDIANA);
    float corrienteFiltrada1 = (voltajeFiltrado - VREF) / sensibilidad;

    sumaMedia -= bufferMedia[idxMedia];
    bufferMedia[idxMedia] = corrienteFiltrada1;
    sumaMedia += corrienteFiltrada1;
    idxMedia = (idxMedia + 1) % N_MEDIA;
    float corrienteFiltrada2 = sumaMedia / N_MEDIA;

    // === VOLTAJE REAL ===
    int lecturaVoltajeADC = analogRead(VOLTAJE_PIN);
    float voltajeADC = (lecturaVoltajeADC / 4095.0) * 3.3;
    bufferMedianaVolt[idxMedianaVolt] = voltajeADC;
    idxMedianaVolt = (idxMedianaVolt + 1) % N_MEDIANA_VOLT;
    float voltajeADC_med = medianaN(bufferMedianaVolt, N_MEDIANA_VOLT);
    float voltajeReal = voltajeADC_med * DIVISOR_FACTOR;

    float potencia = corrienteIIR * voltajeReal;

    // === ENVÍO CAN ===
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = 0x298;
    tx_frame.FIR.B.DLC = 8;
    tx_frame.data.u8[0] = pulsadorValor;
    tx_frame.data.u8[1] = encoderSteps;
    tx_frame.data.u8[2] = 0x00;
    tx_frame.data.u8[3] = 0x00;
    tx_frame.data.u8[4] = 0x00;
    tx_frame.data.u8[5] = 0x00;
    tx_frame.data.u8[6] = 0x00;
    tx_frame.data.u8[7] = 0x00;
    ESP32Can.CANWriteFrame(&tx_frame);

    // === IMPRESIÓN SERIAL ===
    Serial.print(voltaje, 4);             Serial.print(",");
    Serial.print(corriente, 4);           Serial.print(",");
    Serial.print(voltajeFiltrado, 4);     Serial.print(",");
    Serial.print(corrienteFiltrada1, 4);  Serial.print(",");
    Serial.print(corrienteFiltrada2, 4);  Serial.print(",");
    Serial.print(corrienteIIR, 4);        Serial.print(",");
    Serial.print(voltajeReal, 3);         Serial.print(",");

    Serial.print(last_torque);            Serial.print(",");
    Serial.print(last_duty);              Serial.print(",");
    Serial.print(last_corriente);         Serial.print(",");
    Serial.print(last_voltaje_CAN, 1);    Serial.print(",");
    Serial.print(last_switch);            Serial.print(",");
    Serial.print(last_temp);              Serial.print(",");
    Serial.print(last_angle_dir);         Serial.print(",");
    Serial.print(last_mapa);              Serial.print(",");
    Serial.print(last_error);             Serial.print(",");

    Serial.print(potencia, 4);            Serial.print(",");
    Serial.print(encoderSteps);           Serial.print(",");
    Serial.println(pulsadorValor);
  }
}
