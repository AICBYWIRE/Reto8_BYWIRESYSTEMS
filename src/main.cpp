#include <Arduino.h>
#include "config.h"
#include <HardwareSerial.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <SPI.h>
#include <SD.h>
#include <Dynamixel2Arduino.h>

// ========== CONFIGURACIÓN DE HARDWARE ==========
// Pines
#define VOLTAJE_HALL         35
#define VOLTAJE_BATERIA      34
#define ENCODER_CLK         32
#define ENCODER_DT          33
#define ENCODER_SW          25

#define RS485_TX_PIN        22
#define RS485_RX_PIN        21
#define RS485_EN_PIN        17
#define RS485_SE_PIN        19
#define PIN_5V_EN           16

// Configuración Dynamixel
HardwareSerial DXL_SERIAL(2);
Dynamixel2Arduino dxl(DXL_SERIAL, RS485_EN_PIN);
const float DXL_PROTOCOL_VERSION = 2.0;
const uint32_t DXL_BAUDRATE = 57600;
const uint8_t DXL_ID = 1;

// ========== PARÁMETROS DEL SISTEMA ==========
const float VOLANTE_RANGO_GRADOS = 900.0f;
const float TORQUE_MAX_VOLANTE = 8.0f;
const float TORQUE_MAX_CREMALLERA = 40.0f;
const float TORQUE_LIMIT_VIRTUAL = 5.0f;
const float TORQUE_ZONE_DEG = 10.0f;
const float RIGIDEZ_CENTRADO_VEL = 0.05f;

// Configuración velocidad simulada
int velocidad_simulada = 0;
const int VEL_MIN = 0;
const int VEL_MAX = 255;
const int VEL_STEP = 5;

// Filtrado de señales
const float ALPHA_SUAVE = 0.05f;
const float ALPHA_VOLTAJE_IIR = 0.5f;
const uint8_t KERNEL_MEDIANA = 3;
const uint8_t TAM_MEDIA_MOVIL = 5;

// ========== VARIABLES GLOBALES ==========
// Estado del sistema
struct SystemState {
    bool calibrado = false;
    bool torqueActivado = true;
    bool autocentradoActivado = false;
    int signoTorque = 0;
    int32_t pos_offset_dyna = 0;
};

// Mediciones
struct Measurements {
    float voltaje_suave = 0.0f;
    float torque_final_suave = 0.0f;
    float torque_final_volante_suave = 0.0f;
    float voltaje_final = 0.0f;
    float corriente_final = 0.0f;
    float torque_final = 0.0f;
    float torque_final_volante = 0.0f;
    float grados_volante = 0.0f;
    float par_virtual = 0.0f;
    float VREF = 0.0f;
};

// Comunicación CAN
struct CANData {
    uint8_t last_duty = 0;
    uint8_t last_corriente = 0;
    float last_voltaje_CAN = 0.0f;
    uint8_t last_switch = 0;
    uint8_t last_temp = 0;
    uint8_t last_angle_dir = 0;
    uint8_t last_mapa = 0;
    uint8_t last_error = 0;
};

// Encoder
struct EncoderState {
    int lastCLK = HIGH;
    int pulsadorState = HIGH;
    int lastPulsadorState = HIGH;
    int pulsadorValor = 0;
};

// Buffers para filtrado
struct FilterBuffers {
    float bufferMedianaVoltaje[KERNEL_MEDIANA] = {0.0f};
    float bufferMediaVoltaje[TAM_MEDIA_MOVIL] = {0.0f};
    float voltajeIIR = 0.0f;
    float sumaMediaVoltaje = 0.0f;
    uint8_t idxMedianaVoltaje = 0;
    uint8_t idxMediaVoltaje = 0;
};

// ========== INSTANCIAS ==========
SystemState systemState;
Measurements measurements;
CANData canData;
EncoderState encoderState;
FilterBuffers filterBuffers;

// Configuración CAN
CAN_device_t CAN_cfg;
const int rx_queue_size = 10;
const int interval = 3;
unsigned long previousMedicion = 0;
unsigned long previousSuavizado = 0;
const unsigned long intervaloSuavizado = 50;

// Calibración
const float SENSIBILIDAD = 0.0160f;
const uint32_t TIEMPO_CALIBRACION_MS = 10000;
unsigned long tiempoInicio = 0;
float sumaVref = 0.0f;
unsigned long muestrasVref = 0;

// ========== PROTOTIPOS DE FUNCIONES ==========
void setupHardware();
void calibrateSystem();
void leerCAN();
void leerPulsador();
void leerSensorCorriente();
void calcularDireccion();
void calcularTorque();
void aplicarTopeVirtual();
void enviarCAN();
void imprimirDebug();
void serialControlModoTorque();
float calcularTorqueVolante(float corriente);
float medianaN(float* arr, uint8_t size);
void checkDynamixelConnection();

// ========== FUNCIONES PRINCIPALES ==========
void setup() {
    setupHardware();
    calibrateSystem();
}

void loop() {
    unsigned long currentMillis = millis();

    if (!systemState.calibrado) return;

    if (currentMillis - previousMedicion >= interval) {
        previousMedicion = currentMillis;
        
        leerCAN();
        leerPulsador();
        leerSensorCorriente();
        calcularDireccion();
        calcularTorque();
        aplicarTopeVirtual();
        enviarCAN();
        imprimirDebug();
    }

    serialControlModoTorque();
}

// ========== IMPLEMENTACIÓN DE FUNCIONES ==========
void setupHardware() {
    // Configuración de pines
    pinMode(PIN_5V_EN, OUTPUT);
    digitalWrite(PIN_5V_EN, HIGH);
    pinMode(RS485_EN_PIN, OUTPUT);
    digitalWrite(RS485_EN_PIN, LOW);
    pinMode(RS485_SE_PIN, OUTPUT);
    digitalWrite(RS485_SE_PIN, HIGH);
    delay(100);

    // Inicialización comunicación serie
    Serial.begin(921600);
    while (!Serial); // Esperar a que se inicie el puerto serial

    // Configuración Dynamixel
    DXL_SERIAL.begin(DXL_BAUDRATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
    dxl.begin();
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
    checkDynamixelConnection();

    // Configuración ADC
    analogReadResolution(12);

    // Configuración pines encoder
    pinMode(ENCODER_CLK, INPUT);
    pinMode(ENCODER_DT, INPUT);
    pinMode(ENCODER_SW, INPUT_PULLUP);
    encoderState.lastCLK = digitalRead(ENCODER_CLK);

    // Configuración CAN
    pinMode(CAN_SE_PIN, OUTPUT);
    digitalWrite(CAN_SE_PIN, LOW);
    CAN_cfg.speed = CAN_SPEED_250KBPS;
    CAN_cfg.tx_pin_id = GPIO_NUM_27;
    CAN_cfg.rx_pin_id = GPIO_NUM_26;
    CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
    if (ESP32Can.CANInit() != ESP_OK) {
        Serial.println("Error al inicializar CAN");
    }
}

void checkDynamixelConnection() {
    if (dxl.ping(DXL_ID)) {
        Serial.println("✅ Dynamixel detectado.");
        dxl.torqueOff(DXL_ID);
    } else {
        Serial.println("❌ Dynamixel no responde.");
        while (true) {
            delay(1000);
            Serial.println("Reintentando conexión con Dynamixel...");
            if (dxl.ping(DXL_ID)) {
                Serial.println("✅ Dynamixel conectado después de reintento.");
                break;
            }
        }
    }
}

void calibrateSystem() {
    Serial.println("⚙️ Calibrando VREF durante 10 segundos, mantén el motor en idle...");
    tiempoInicio = millis();
    
    while (millis() - tiempoInicio < TIEMPO_CALIBRACION_MS) {
        int lecturaADC = analogRead(VOLTAJE_HALL);
        float voltaje = (lecturaADC / 4095.0f) * 3.3f;
        sumaVref += voltaje;
        muestrasVref++;
        delay(10);
    }
    
    measurements.VREF = sumaVref / muestrasVref;
    systemState.calibrado = true;
    Serial.print("✅ Calibración completada. VREF = ");
    Serial.print(measurements.VREF, 4);
    Serial.println(" V");
    Serial.println("V_hall(V),MotorDuty(%),I_CAN(A),V_CAN(V),SwitchPos,Temp(°C),DirAngle(bits),Mapa,Error,DireccionBits,Pulsador,V_final(V),I_final(A),Torque_final(N·m),Torque_volante(N·m),Voltaje_suave(V),Torque_final_suave(N·m),Torque_volante_suave(N·m),I_Dynamixel(mA)");
}

float calcularTorqueVolante(float corriente) {
    float torque = 1.2f * corriente;
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
            canData.last_duty = rx_frame.data.u8[1];
            canData.last_corriente = rx_frame.data.u8[2];
            canData.last_voltaje_CAN = rx_frame.data.u8[3] * 0.1f;
            canData.last_switch = rx_frame.data.u8[4];
            canData.last_temp = rx_frame.data.u8[5];
        } else if (rx_frame.MsgID == 0x292) {
            canData.last_angle_dir = rx_frame.data.u8[0];
            canData.last_mapa = rx_frame.data.u8[3];
            canData.last_error = rx_frame.data.u8[4];
        }
    }
}

void leerPulsador() {
    int currentCLK = digitalRead(ENCODER_CLK);
    if (currentCLK != encoderState.lastCLK && currentCLK == LOW) {
        if (digitalRead(ENCODER_DT) != currentCLK) {
            velocidad_simulada += VEL_STEP;
        } else {
            velocidad_simulada -= VEL_STEP;
        }
        velocidad_simulada = constrain(velocidad_simulada, VEL_MIN, VEL_MAX);
    }
    encoderState.lastCLK = currentCLK;

    encoderState.pulsadorState = digitalRead(ENCODER_SW);
    if (encoderState.pulsadorState == LOW && encoderState.lastPulsadorState == HIGH) {
        encoderState.pulsadorValor = (encoderState.pulsadorValor + 1) % 6;
    }
    encoderState.lastPulsadorState = encoderState.pulsadorState;
}

void leerSensorCorriente() {
    int lecturaADC = analogRead(VOLTAJE_HALL);
    float voltaje = (lecturaADC / 4095.0f) * 3.3f;

    // Filtrado de mediana
    filterBuffers.bufferMedianaVoltaje[filterBuffers.idxMedianaVoltaje] = voltaje;
    filterBuffers.idxMedianaVoltaje = (filterBuffers.idxMedianaVoltaje + 1) % KERNEL_MEDIANA;
    float voltajeMediana = medianaN(filterBuffers.bufferMedianaVoltaje, KERNEL_MEDIANA);

    // Filtrado de media móvil
    filterBuffers.sumaMediaVoltaje -= filterBuffers.bufferMediaVoltaje[filterBuffers.idxMediaVoltaje];
    filterBuffers.bufferMediaVoltaje[filterBuffers.idxMediaVoltaje] = voltajeMediana;
    filterBuffers.sumaMediaVoltaje += voltajeMediana;
    filterBuffers.idxMediaVoltaje = (filterBuffers.idxMediaVoltaje + 1) % TAM_MEDIA_MOVIL;

    float voltajeMediaMovil = filterBuffers.sumaMediaVoltaje / TAM_MEDIA_MOVIL;
    filterBuffers.voltajeIIR = ALPHA_VOLTAJE_IIR * voltajeMediaMovil + (1.0f - ALPHA_VOLTAJE_IIR) * filterBuffers.voltajeIIR;
    measurements.voltaje_final = filterBuffers.voltajeIIR;

    measurements.corriente_final = (measurements.voltaje_final - measurements.VREF) / SENSIBILIDAD;
}

void calcularDireccion() {
    static float grados_volante_anterior = 0.0f;
    int32_t pos_actual = dxl.getPresentPosition(DXL_ID);
    int32_t delta_pos = pos_actual - systemState.pos_offset_dyna;

    measurements.grados_volante = (delta_pos / 4096.0f) * 360.0f;
    measurements.grados_volante = constrain(measurements.grados_volante, -VOLANTE_RANGO_GRADOS / 2, VOLANTE_RANGO_GRADOS / 2);

    if (measurements.grados_volante > grados_volante_anterior) systemState.signoTorque = +1;
    else if (measurements.grados_volante < grados_volante_anterior) systemState.signoTorque = -1;
    grados_volante_anterior = measurements.grados_volante;
}

void calcularTorque() {
    measurements.torque_final = -systemState.signoTorque * 3 * fabs(measurements.corriente_final);
    measurements.torque_final_volante = calcularTorqueVolante(measurements.torque_final);

    unsigned long currentMillis = millis();
    if (currentMillis - previousSuavizado >= intervaloSuavizado) {
        previousSuavizado = currentMillis;
        measurements.voltaje_suave = ALPHA_SUAVE * measurements.voltaje_final + (1.0f - ALPHA_SUAVE) * measurements.voltaje_suave;
        measurements.torque_final_suave = ALPHA_SUAVE * measurements.torque_final + (1.0f - ALPHA_SUAVE) * measurements.torque_final_suave;
        measurements.torque_final_volante_suave = ALPHA_SUAVE * measurements.torque_final_volante + (1.0f - ALPHA_SUAVE) * measurements.torque_final_volante_suave;
    }
}

void aplicarTopeVirtual() {
    const float ZONA_MUERTA_CENTRADO = 3.0f;  // zona muerta de ±3°
    float torque_centrado_virtual = 0.0f;

    if (systemState.autocentradoActivado && abs(measurements.grados_volante) > ZONA_MUERTA_CENTRADO) {
        torque_centrado_virtual = -RIGIDEZ_CENTRADO_VEL * measurements.grados_volante;
    }

    if (measurements.grados_volante > VOLANTE_RANGO_GRADOS / 2 - TORQUE_ZONE_DEG) {
        measurements.par_virtual = -TORQUE_LIMIT_VIRTUAL;
    } else if (measurements.grados_volante < -VOLANTE_RANGO_GRADOS / 2 + TORQUE_ZONE_DEG) {
        measurements.par_virtual = TORQUE_LIMIT_VIRTUAL;
    } else {
        measurements.par_virtual = torque_centrado_virtual;
    }

    if (systemState.torqueActivado) {
        dxl.torqueOn(DXL_ID);
        float torque_total = measurements.torque_final_volante_suave + measurements.par_virtual;
        int16_t corriente_dxl_mA = (torque_total / TORQUE_MAX_VOLANTE) * 1193;
        corriente_dxl_mA = constrain(corriente_dxl_mA, -1193, 1193);
        dxl.setGoalCurrent(DXL_ID, corriente_dxl_mA, UNIT_MILLI_AMPERE);
    } else {
        dxl.torqueOff(DXL_ID);
    }
}

void enviarCAN() {
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = 0x298;
    tx_frame.FIR.B.DLC = 8;
    
    // Calcular dirección en bits (10-250)
    float direccion_bits_f = (measurements.grados_volante + VOLANTE_RANGO_GRADOS / 2) * (240.0f / VOLANTE_RANGO_GRADOS) + 10.0f;
    direccion_bits_f = constrain(direccion_bits_f, 10.0f, 250.0f);
    uint8_t direccion_bits = static_cast<uint8_t>(direccion_bits_f);
    
    // Bit de centrado (1 si está centrado, 0 si no)
    uint8_t bit_centrado = (abs(measurements.grados_volante) < 5.0f) ? 1 : 0;

    tx_frame.data.u8[0] = 5;
    tx_frame.data.u8[1] = direccion_bits;
    tx_frame.data.u8[2] = bit_centrado;
    tx_frame.data.u8[3] = (uint8_t)velocidad_simulada;
    tx_frame.data.u8[4] = 0x00;
    tx_frame.data.u8[5] = 0x00;
    tx_frame.data.u8[6] = 0x00;
    tx_frame.data.u8[7] = 0x00;
    
    if (ESP32Can.CANWriteFrame(&tx_frame) != ESP_OK) {
        Serial.println("Error al enviar mensaje CAN");
    }
}

void imprimirDebug() {
    Serial.print(measurements.voltaje_final, 4); Serial.print(",");
    Serial.print(canData.last_duty); Serial.print(",");
    Serial.print(canData.last_corriente); Serial.print(",");
    Serial.print(canData.last_voltaje_CAN, 1); Serial.print(",");
    Serial.print(canData.last_switch); Serial.print(",");
    Serial.print(canData.last_temp); Serial.print(",");
    Serial.print(canData.last_angle_dir); Serial.print(",");
    Serial.print(canData.last_mapa); Serial.print(",");
    Serial.print(canData.last_error); Serial.print(",");
    
    // Calcular dirección en bits para el debug (igual que en enviarCAN)
    float direccion_bits_f = (measurements.grados_volante + VOLANTE_RANGO_GRADOS / 2) * (240.0f / VOLANTE_RANGO_GRADOS) + 10.0f;
    direccion_bits_f = constrain(direccion_bits_f, 10.0f, 250.0f);
    uint8_t direccion_bits = static_cast<uint8_t>(direccion_bits_f);
    
    Serial.print(direccion_bits); Serial.print(",");
    Serial.print(encoderState.pulsadorValor); Serial.print(",");
    Serial.print(measurements.voltaje_final, 4); Serial.print(",");
    Serial.print(measurements.corriente_final, 4); Serial.print(",");
    Serial.print(measurements.torque_final, 4); Serial.print(",");
    Serial.print(measurements.torque_final_volante, 4); Serial.print(",");
    Serial.print(measurements.voltaje_suave, 4); Serial.print(",");
    Serial.print(measurements.torque_final_suave, 4); Serial.print(",");
    Serial.print(measurements.torque_final_volante_suave, 4); Serial.print(",");
    
    // Calcular corriente Dynamixel para el debug
    float torque_total = measurements.torque_final_volante_suave + measurements.par_virtual;
    int16_t corriente_dxl_mA = (torque_total / TORQUE_MAX_VOLANTE) * 1193;
    corriente_dxl_mA = constrain(corriente_dxl_mA, -1193, 1193);
    Serial.println(corriente_dxl_mA);
}

void serialControlModoTorque() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '1') {
            systemState.torqueActivado = !systemState.torqueActivado;
            Serial.print("🔧 Torque ");
            Serial.println(systemState.torqueActivado ? "ACTIVADO" : "DESACTIVADO");
        }
        if (c == '2') {
            systemState.autocentradoActivado = !systemState.autocentradoActivado;
            Serial.print("📐 Autocentrado ");
            Serial.println(systemState.autocentradoActivado ? "ACTIVADO" : "DESACTIVADO");
        }
    }
}
