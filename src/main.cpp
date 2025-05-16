// Código completo con rango de volante configurable
// y actualización automática de límites, escalado CAN y tope virtual

#include <Arduino.h>
#include "config.h"
#include <HardwareSerial.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <SPI.h>
#include <SD.h>
#include <Dynamixel2Arduino.h>

#define VOLTAJE_HALL     35
#define VOLTAJE_BATERIA  34
#define ENCODER_CLK      32
#define ENCODER_DT       33
#define ENCODER_SW       25

#define RS485_TX_PIN 22
#define RS485_RX_PIN 21
#define RS485_EN_PIN 17
#define RS485_SE_PIN 19
#define PIN_5V_EN    16

HardwareSerial DXL_SERIAL(2);
Dynamixel2Arduino dxl(DXL_SERIAL, RS485_EN_PIN);
const float DXL_PROTOCOL_VERSION = 2.0;
const uint32_t DXL_BAUDRATE = 57600;
const uint8_t DXL_ID = 1;

#define TIEMPO_CALIBRACION_MS 10000

const float VOLANTE_RANGO_GRADOS = 900.0; //
const float TORQUE_MAX_VOLANTE = 7.0;
const float TORQUE_MAX_CREMALLERA = 90.0;
const float TORQUE_LIMIT_VIRTUAL = 0.6;
const float TORQUE_ZONE_DEG = 10.0;
const float RIGIDEZ_CENTRADO_VEL = 0.00001;

int velocidad_simulada = 0;
const int VEL_MIN = 0;
const int VEL_MAX = 255;
const int VEL_STEP = 5;

float voltaje_suave = 0.0;
float torque_final_suave = 0.0;
float torque_final_volante_suave = 0.0;
const float alphaSuave = 0.05;
unsigned long previousSuavizado = 0;
const unsigned long intervaloSuavizado = 50;

unsigned long tiempoInicio = 0;
bool calibrado = false;
float VREF = 0.0;
float sensibilidad = 0.0160;
float sumaVref = 0.0;
unsigned long muestrasVref = 0;

int lastCLK = HIGH;
int pulsadorState = HIGH;
int lastPulsadorState = HIGH;
int pulsadorValor = 0;

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

int32_t pos_offset_dyna = 0;
float corriente_final = 0.0;
float torque_final = 0.0;
float torque_final_volante = 0.0;
float voltaje_final = 0.0;
float grados_volante = 0.0;
uint8_t direccion_bits = 0;
uint8_t bit_centrado = 0;
float par_virtual = 0.0;

float calcularTorqueVolante(float corriente) {
  float torque = 1.2 * corriente;
  return constrain(torque * (TORQUE_MAX_VOLANTE / TORQUE_MAX_CREMALLERA), -TORQUE_MAX_VOLANTE, TORQUE_MAX_VOLANTE);
}

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

void leerCAN() {
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
}

void leerPulsador() {
  int currentCLK = digitalRead(ENCODER_CLK);
  if (currentCLK != lastCLK && currentCLK == LOW) {
    if (digitalRead(ENCODER_DT) != currentCLK) velocidad_simulada += VEL_STEP;
    else velocidad_simulada -= VEL_STEP;
    velocidad_simulada = constrain(velocidad_simulada, VEL_MIN, VEL_MAX);
  }
  lastCLK = currentCLK;
  pulsadorState = digitalRead(ENCODER_SW);
  if (pulsadorState == LOW && lastPulsadorState == HIGH) pulsadorValor = (pulsadorValor + 1) % 6;
  lastPulsadorState = pulsadorState;
}

void leerSensorCorriente() {
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
  voltaje_final = voltajeIIR;
  corriente_final = (voltaje_final - VREF) / sensibilidad;
}

void calcularTorque() {
  torque_final = 1.2 * corriente_final;
  torque_final_volante = calcularTorqueVolante(torque_final);
  if (millis() - previousSuavizado >= intervaloSuavizado) {
    previousSuavizado = millis();
    voltaje_suave = alphaSuave * voltaje_final + (1.0 - alphaSuave) * voltaje_suave;
    torque_final_suave = alphaSuave * torque_final + (1.0 - alphaSuave) * torque_final_suave;
    torque_final_volante_suave = alphaSuave * torque_final_volante + (1.0 - alphaSuave) * torque_final_volante_suave;
  }
}

void calcularDireccion() {
  int32_t pos_actual = dxl.getPresentPosition(DXL_ID);
  int32_t delta_pos = pos_actual - pos_offset_dyna;
  grados_volante = (delta_pos / 4096.0) * 360.0;
  grados_volante = constrain(grados_volante, -VOLANTE_RANGO_GRADOS / 2, VOLANTE_RANGO_GRADOS / 2);
  float direccion_bits_f = (grados_volante + VOLANTE_RANGO_GRADOS / 2) * ((250.0 - 10.0) / VOLANTE_RANGO_GRADOS) + 10.0;
  direccion_bits_f = constrain(direccion_bits_f, 10.0, 250.0);
  direccion_bits = static_cast<uint8_t>(direccion_bits_f);
  bit_centrado = (abs(grados_volante) < 5.0) ? 1 : 0;
}

void aplicarTopeVirtual() {
  float torque_centrado_virtual = -RIGIDEZ_CENTRADO_VEL * velocidad_simulada * grados_volante;
  if (grados_volante > VOLANTE_RANGO_GRADOS / 2 - TORQUE_ZONE_DEG) par_virtual = -TORQUE_LIMIT_VIRTUAL;
  else if (grados_volante < -VOLANTE_RANGO_GRADOS / 2 + TORQUE_ZONE_DEG) par_virtual = TORQUE_LIMIT_VIRTUAL;
  else par_virtual = torque_centrado_virtual;
  if (abs(par_virtual) > 0.01) {
    dxl.torqueOn(DXL_ID);
    int16_t corriente_dxl_mA = (par_virtual / TORQUE_MAX_VOLANTE) * 1193;
    corriente_dxl_mA = constrain(corriente_dxl_mA, -1193, 1193);
    dxl.setGoalCurrent(DXL_ID, corriente_dxl_mA, UNIT_MILLI_AMPERE);
  } else dxl.torqueOff(DXL_ID);
}

void enviarCAN() {
  CAN_frame_t tx_frame;
  tx_frame.FIR.B.FF = CAN_frame_std;
  tx_frame.MsgID = 0x298;
  tx_frame.FIR.B.DLC = 8;
  tx_frame.data.u8[0] = pulsadorValor;
  tx_frame.data.u8[1] = direccion_bits;
  tx_frame.data.u8[2] = bit_centrado;
  tx_frame.data.u8[3] = (uint8_t)velocidad_simulada;
  for (int i = 4; i < 8; i++) tx_frame.data.u8[i] = 0x00;
  ESP32Can.CANWriteFrame(&tx_frame);
}

void imprimirDebug() {
  Serial.print(voltaje_final, 4); Serial.print(",");
  Serial.print(last_duty); Serial.print(",");
  Serial.print(last_corriente); Serial.print(",");
  Serial.print(last_voltaje_CAN, 1); Serial.print(",");
  Serial.print(last_switch); Serial.print(",");
  Serial.print(last_temp); Serial.print(",");
  Serial.print(last_angle_dir); Serial.print(",");
  Serial.print(last_mapa); Serial.print(",");
  Serial.print(last_error); Serial.print(",");
  Serial.print(direccion_bits); Serial.print(",");
  Serial.print(pulsadorValor); Serial.print(",");
  Serial.print(voltaje_final, 4); Serial.print(",");
  Serial.print(corriente_final, 4); Serial.print(",");
  Serial.print(torque_final, 4); Serial.print(",");
  Serial.print(torque_final_volante, 4); Serial.print(",");
  Serial.print(voltaje_suave, 4); Serial.print(",");
  Serial.print(torque_final_suave, 4); Serial.print(",");
  Serial.print(torque_final_volante_suave, 4); Serial.print(",");
  Serial.println((int)((par_virtual / TORQUE_MAX_VOLANTE) * 1193));
}

void setup() {
  pinMode(PIN_5V_EN, OUTPUT);      digitalWrite(PIN_5V_EN, HIGH);
  pinMode(RS485_EN_PIN, OUTPUT);  digitalWrite(RS485_EN_PIN, LOW);
  pinMode(RS485_SE_PIN, OUTPUT);  digitalWrite(RS485_SE_PIN, HIGH);
  delay(100);
  DXL_SERIAL.begin(DXL_BAUDRATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  dxl.begin();
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  Serial.begin(921600);
  analogReadResolution(12);
  if (dxl.ping(DXL_ID)) {
    Serial.println("✅ Dynamixel detectado.");
    dxl.torqueOff(DXL_ID);
  } else {
    Serial.println("❌ Dynamixel no responde.");
    while (true);
  }
  pos_offset_dyna = dxl.getPresentPosition(DXL_ID);
  Serial.print("📍 Offset posición inicial: "); Serial.println(pos_offset_dyna);
  pinMode(CAN_SE_PIN, OUTPUT); digitalWrite(CAN_SE_PIN, LOW);
  pinMode(ENCODER_CLK, INPUT); pinMode(ENCODER_DT, INPUT);
  pinMode(ENCODER_SW, INPUT_PULLUP); lastCLK = digitalRead(ENCODER_CLK);
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
  if (!calibrado) {
    int lecturaADC = analogRead(VOLTAJE_HALL);
    float voltaje = (lecturaADC / 4095.0) * 3.3;
    sumaVref += voltaje; muestrasVref++;
    if (currentMillis - tiempoInicio >= TIEMPO_CALIBRACION_MS) {
      VREF = sumaVref / muestrasVref; calibrado = true;
      Serial.print("✅ Calibración completada. VREF = "); Serial.print(VREF, 4); Serial.println(" V");
      Serial.println("V_hall(V),MotorDuty(%),I_CAN(A),V_CAN(V),SwitchPos,Temp(°C),DirAngle(bits),Mapa,Error,DireccionBits,Pulsador,V_final(V),I_final(A),Torque_final(N·m),Torque_volante(N·m),Voltaje_suave(V),Torque_final_suave(N·m),Torque_volante_suave(N·m),I_Dynamixel(mA)");
      delay(1000);
    }
    return;
  }
  if (currentMillis - previousMedicion >= interval) {
    previousMedicion = currentMillis;
    leerCAN(); leerPulsador(); leerSensorCorriente();
    calcularTorque(); calcularDireccion(); aplicarTopeVirtual();
    enviarCAN(); imprimirDebug();
  }
}
