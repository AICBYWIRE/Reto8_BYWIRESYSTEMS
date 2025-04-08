#include <Arduino.h>
#include "config.h"
#include <HardwareSerial.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <SPI.h>
#include <SD.h>

#define VOLTAJE_HALL     35
#define VOLTAJE_BATERIA  34
#define ENCODER_CLK      32
#define ENCODER_DT       33
#define ENCODER_SW       25

#define TIEMPO_CALIBRACION_MS 10000
const float DIVISOR_FACTOR = 5.057;

const float TORQUE_MAX_VOLANTE = 7.0;
const float TORQUE_MAX_CREMALLERA = 90.0;

float voltaje_suave = 0.0;
float torque_final_suave = 0.0;
float torque_final_volante_suave = 0.0;
const float alphaSuave = 0.05;
unsigned long previousSuavizado = 0;
const unsigned long intervaloSuavizado = 50;

float calcularTorqueVolante(float corriente_final) {
  float torque_final = 1.2 * corriente_final;
  float torque_final_volante = torque_final * (TORQUE_MAX_VOLANTE / TORQUE_MAX_CREMALLERA);
  return constrain(torque_final_volante, -TORQUE_MAX_VOLANTE, TORQUE_MAX_VOLANTE);
}

unsigned long tiempoInicio = 0;
bool calibrado = false;
float VREF = 0.0;
float sensibilidad = 0.0160;
float sumaVref = 0.0;
unsigned long muestrasVref = 0;

int encoderSteps = 100;
int lastEncoderSteps = 100;
int lastCLK = HIGH;
int pulsadorState = HIGH;
int lastPulsadorState = HIGH;
int pulsadorValor = 0;
int signoTorque = 0;

const uint8_t KERNEL_MEDIANA = 3;
float bufferMedianaVoltaje[KERNEL_MEDIANA] = {0.0};
uint8_t idxMedianaVoltaje = 0;

const uint8_t TAM_MEDIA_MOVIL = 10;
float bufferMediaVoltaje[TAM_MEDIA_MOVIL] = {0.0};
float sumaMediaVoltaje = 0.0;
uint8_t idxMediaVoltaje = 0;

float voltajeIIR = 0.0;
const float alphaVoltajeIIR = 0.5;

CAN_device_t CAN_cfg;
const int rx_queue_size = 10;
const int interval = 3;
unsigned long previousMedicion = 0;

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

  Serial.println("\u2699\ufe0f Calibrando VREF durante 10 segundos, mant\u00e9n el motor en idle...");
  tiempoInicio = millis();

  CAN_cfg.speed = CAN_SPEED_250KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_27;
  CAN_cfg.rx_pin_id = GPIO_NUM_26;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  ESP32Can.CANInit();
}

void loop() {
  unsigned long currentMillis = millis();

  if (!calibrado) {
    int lecturaADC = analogRead(VOLTAJE_HALL);
    float voltaje = (lecturaADC / 4095.0) * 3.3;
    sumaVref += voltaje;
    muestrasVref++;
    if (currentMillis - tiempoInicio >= TIEMPO_CALIBRACION_MS) {
      VREF = sumaVref / muestrasVref;
      calibrado = true;
      Serial.print("\u2705 Calibraci\u00f3n completada. VREF = ");
      Serial.print(VREF, 4);
      Serial.println(" V");
      Serial.println("V_hall(V),MotorDuty(%),I_CAN(A),V_CAN(V),SwitchPos,Temp(\u00b0C),DirAngle(bits),Mapa,Error,EncoderSteps,Pulsador,V_final(V),I_final(A),Torque_final(N\u00b7m),Torque_volante(N\u00b7m),Voltaje_suave(V),Torque_final_suave(N\u00b7m),Torque_volante_suave(N\u00b7m)");
      delay(1000);
    }
    return;
  }

  if (currentMillis - previousMedicion >= interval) {
    previousMedicion = currentMillis;

    CAN_frame_t rx_frame;
    while (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 0) == pdTRUE) {
      if (rx_frame.MsgID == 0x290) {
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

    int currentCLK = digitalRead(ENCODER_CLK);
    if (currentCLK != lastCLK && currentCLK == LOW) {
      encoderSteps += (digitalRead(ENCODER_DT) != currentCLK) ? 5 : -5;
      encoderSteps = constrain(encoderSteps, 10, 250);
    }
    lastCLK = currentCLK;

    pulsadorState = digitalRead(ENCODER_SW);
    if (pulsadorState == LOW && lastPulsadorState == HIGH) {
      pulsadorValor = (pulsadorValor + 1) % 6;
    }
    lastPulsadorState = pulsadorState;

    int lecturaADC = analogRead(VOLTAJE_HALL);
    float voltaje = (lecturaADC / 4095.0) * 3.3;

    bufferMedianaVoltaje[idxMedianaVoltaje] = voltaje;
    idxMedianaVoltaje = (idxMedianaVoltaje + 1) % KERNEL_MEDIANA;
    float voltajeMediana = medianaN(bufferMedianaVoltaje, KERNEL_MEDIANA);

    sumaMediaVoltaje -= bufferMediaVoltaje[idxMediaVoltaje];
    bufferMediaVoltaje[idxMediaVoltaje] = voltajeMediana;
    sumaMediaVoltaje += voltajeMediana;
    idxMediaVoltaje = (idxMediaVoltaje + 1) % TAM_MEDIA_MOVIL;
    float voltajeMediaMovil = sumaMediaVoltaje / TAM_MEDIA_MOVIL;

    voltajeIIR = alphaVoltajeIIR * voltajeMediaMovil + (1.0 - alphaVoltajeIIR) * voltajeIIR;
    float voltaje_final = voltajeIIR;

    float corriente_final = (voltaje_final - VREF) / sensibilidad;

    if (encoderSteps > lastEncoderSteps) signoTorque = +1;
    else if (encoderSteps < lastEncoderSteps) signoTorque = -1;
    lastEncoderSteps = encoderSteps;

    float torque_final = signoTorque * 1.2 * fabs(corriente_final);
    float torque_final_volante = calcularTorqueVolante(torque_final);

    if (currentMillis - previousSuavizado >= intervaloSuavizado) {
      previousSuavizado = currentMillis;
      voltaje_suave = alphaSuave * voltaje_final + (1.0 - alphaSuave) * voltaje_suave;
      torque_final_suave = alphaSuave * torque_final + (1.0 - alphaSuave) * torque_final_suave;
      torque_final_volante_suave = alphaSuave * torque_final_volante + (1.0 - alphaSuave) * torque_final_volante_suave;
    }

    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = 0x298;
    tx_frame.FIR.B.DLC = 8;
    tx_frame.data.u8[0] = pulsadorValor;
    tx_frame.data.u8[1] = encoderSteps;
    for (int i = 2; i < 8; i++) tx_frame.data.u8[i] = 0x00;
    ESP32Can.CANWriteFrame(&tx_frame);

    Serial.print(voltaje, 4);                  Serial.print(",");
    Serial.print(last_duty);                   Serial.print(",");
    Serial.print(last_corriente);              Serial.print(",");
    Serial.print(last_voltaje_CAN, 1);         Serial.print(",");
    Serial.print(last_switch);                 Serial.print(",");
    Serial.print(last_temp);                   Serial.print(",");
    Serial.print(last_angle_dir);              Serial.print(",");
    Serial.print(last_mapa);                   Serial.print(",");
    Serial.print(last_error);                  Serial.print(",");
    Serial.print(encoderSteps);                Serial.print(",");
    Serial.print(pulsadorValor);               Serial.print(",");
    Serial.print(voltaje_final, 4);            Serial.print(",");
    Serial.print(corriente_final, 4);          Serial.print(",");
    Serial.print(torque_final, 4);             Serial.print(",");
    Serial.print(torque_final_volante, 4);     Serial.print(",");
    Serial.print(voltaje_suave, 4);            Serial.print(",");
    Serial.print(torque_final_suave, 4);       Serial.print(",");
    Serial.println(torque_final_volante_suave, 4);
  }
}