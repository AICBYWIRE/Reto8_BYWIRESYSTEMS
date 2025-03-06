#include <Arduino.h>
#include "config.h"
#include <HardwareSerial.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <SPI.h>
#include <SD.h>

// 📌 Configuración CAN
CAN_device_t CAN_cfg;
const int rx_queue_size = 10;
const int sendInterval = 5;        // Enviar comando cada 5ms
unsigned long lastSendTime = 0;
bool commandSent = false; // Para mostrar solo una vez el mensaje enviado

// 📌 Variables de control generales
uint8_t gain_mode = 0x04;      // Valor por defecto: ganancia media
uint8_t target_angle = 0x80;   // Valor por defecto: posición central
bool send_command = false;     // Si se debe enviar comandos continuamente
bool monitoring_mode = false;  // Si se debe monitorear los mensajes CAN

// Bandera global para controlar la impresión única de "No hay cambios"
bool printedNoChange = false;

// Variables para almacenar el último mensaje 0x290 impreso
bool first_290 = true;
uint8_t last_torque = 0, last_duty = 0, last_corriente = 0, last_switch = 0, last_temp = 0;
float last_voltaje = 0.0;

// Variables para almacenar el último mensaje 0x292 impreso
bool first_292 = true;
uint8_t last_angle_dir = 0, last_mapa = 0, last_error = 0;

// Función auxiliar para determinar si hay una variación >= 10% (para valores enteros)
bool hasSignificantChange(uint8_t lastValue, uint8_t newValue) {
  if (lastValue == 0 && newValue != 0) return true;
  if (lastValue != 0 && (abs(newValue - lastValue) / (float)lastValue) >= 0.1)
    return true;
  return false;
}

// Función auxiliar para valores float (ej. voltaje) con variación >= 10%
bool hasSignificantChangeFloat(float lastValue, float newValue) {
  if (lastValue == 0.0 && newValue != 0.0) return true;
  if (lastValue != 0.0 && (fabs(newValue - lastValue) / lastValue) >= 0.1)
    return true;
  return false;
}

void SD_test(void) {
    SPI.begin(SD_SCLK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("SDCard MOUNT FAIL");
    } else {
        uint32_t cardSize = SD.cardSize() / (1024 * 1024);
        Serial.printf("✅ SDCard Size: %dMB\n", cardSize);
    }
}

void sendSteerToAngle(uint8_t gain, uint8_t angle) {
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = 0x298;
    tx_frame.FIR.B.DLC = 8;
    tx_frame.data.u8[0] = gain;  // Modo Steer to Angle con ganancia
    tx_frame.data.u8[1] = angle; // Ángulo objetivo
    for (int i = 2; i < 8; i++) {
        tx_frame.data.u8[i] = 0; // Bytes no usados
    }
    
    ESP32Can.CANWriteFrame(&tx_frame);
    if (!commandSent) {
        Serial.printf("📤 Comando enviado: 298 %02x %02x 00 00 00 00 00 00\n", gain, angle);
        commandSent = true; // Evita mostrarlo repetidamente
    }
}

void showMainMenu() {
    Serial.println("\n=== MENU PRINCIPAL ===");
    Serial.println("1 - Monitorear mensajes CAN (290 y 292)");
    Serial.println("2 - Controlar dirección (enviar ángulo)");
    Serial.println("Escribe 'exit' para salir de un modo y volver al menú principal.");
}

void setup() {
    pinMode(PIN_5V_EN, OUTPUT);
    digitalWrite(PIN_5V_EN, HIGH);

    pinMode(CAN_SE_PIN, OUTPUT);
    digitalWrite(CAN_SE_PIN, LOW);

    Serial.begin(115200);
    delay(100);
    SD_test();
    Serial.println("🚀 ESP32 CAN Monitor Iniciado");

    CAN_cfg.speed = CAN_SPEED_250KBPS;
    CAN_cfg.tx_pin_id = GPIO_NUM_27;
    CAN_cfg.rx_pin_id = GPIO_NUM_26;
    CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));

    ESP32Can.CANInit();
    Serial.printf("✅ CAN SPEED: %d kbps\n", CAN_cfg.speed);
    
    showMainMenu();
}

void loop() {
    CAN_frame_t rx_frame;
    bool mensajeProcesado = false;
    bool anyChangePrinted = false;
    unsigned long currentMillis = millis();

    // Leer comandos desde la terminal
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input == "1") {
            monitoring_mode = true;
            send_command = false;
            Serial.println("📡 Modo monitoreo activado. Mostrando mensajes 0x290 y 0x292 en tiempo real...");
        } else if (input == "2") {
            monitoring_mode = false;
            Serial.println("🎛 Modo control dirección activado. Escribe 'gain XX' y 'angle XX'");
        } else if (input == "exit") {
            monitoring_mode = false;
            send_command = false;
            showMainMenu();
        } else if (input.startsWith("gain")) {
            gain_mode = strtol(input.substring(5).c_str(), NULL, 16);
            Serial.printf("🎛 Ganancia ajustada a 0x%X\n", gain_mode);
        } else if (input.startsWith("angle")) {
            target_angle = strtol(input.substring(6).c_str(), NULL, 16);
            Serial.printf("🎯 Ángulo objetivo ajustado a 0x%X\n", target_angle);
            send_command = true;
            commandSent = false; // Permitir que el mensaje enviado se muestre una vez
            Serial.println("✅ Envío de comandos ACTIVADO.");
        }
    }

    // Modo monitoreo: Procesar todos los mensajes en la cola
    if (monitoring_mode) {
        // No reiniciamos printedNoChange aquí; se reiniciará solo si se detecta un cambio.
        while (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 0) == pdTRUE) {
            mensajeProcesado = true;
            // Procesar mensaje 0x290
            if (rx_frame.MsgID == 0x290) {
                float voltaje = rx_frame.data.u8[3] * 0.1;
                bool change = false;
                if (first_290) {
                    change = true;
                    first_290 = false;
                } else {
                    if (hasSignificantChange(last_torque, rx_frame.data.u8[0]) ||
                        hasSignificantChange(last_duty, rx_frame.data.u8[1]) ||
                        hasSignificantChange(last_corriente, rx_frame.data.u8[2]) ||
                        hasSignificantChangeFloat(last_voltaje, voltaje) ||
                        hasSignificantChange(last_switch, rx_frame.data.u8[4]) ||
                        hasSignificantChange(last_temp, rx_frame.data.u8[5])) {
                        change = true;
                    }
                }
                if (change) {
                    Serial.println("--- Mensaje #1 ECU (0x290) ---");
                    Serial.printf("Torque: %d bits\n", rx_frame.data.u8[0]);
                    Serial.printf("Motor Duty: %d %%\n", rx_frame.data.u8[1]);
                    Serial.printf("Corriente: %d A\n", rx_frame.data.u8[2]);
                    Serial.printf("Voltaje: %.1f V\n", voltaje);
                    Serial.printf("Switch Pos: %d\n", rx_frame.data.u8[4]);
                    Serial.printf("Temperatura ECU: %d C\n", rx_frame.data.u8[5]);
                    // Actualizar valores
                    last_torque = rx_frame.data.u8[0];
                    last_duty = rx_frame.data.u8[1];
                    last_corriente = rx_frame.data.u8[2];
                    last_voltaje = voltaje;
                    last_switch = rx_frame.data.u8[4];
                    last_temp = rx_frame.data.u8[5];
                    anyChangePrinted = true;
                    printedNoChange = false;  // Se detectó cambio, reiniciamos la bandera
                }
            }
            // Procesar mensaje 0x292
            else if (rx_frame.MsgID == 0x292) {
                bool change = false;
                if (first_292) {
                    change = true;
                    first_292 = false;
                } else {
                    if (hasSignificantChange(last_angle_dir, rx_frame.data.u8[0]) ||
                        hasSignificantChange(last_mapa, rx_frame.data.u8[3]) ||
                        hasSignificantChange(last_error, rx_frame.data.u8[4])) {
                        change = true;
                    }
                }
                if (change) {
                    Serial.println("--- Mensaje #2 ECU (0x292) ---");
                    Serial.printf("Ángulo dirección 8-bit: %d bits\n", rx_frame.data.u8[0]);
                    Serial.printf("Mapa seleccionado: %d\n", rx_frame.data.u8[3]);
                    Serial.printf("Código de error: %d\n", rx_frame.data.u8[4]);
                    // Actualizar valores
                    last_angle_dir = rx_frame.data.u8[0];
                    last_mapa = rx_frame.data.u8[3];
                    last_error = rx_frame.data.u8[4];
                    anyChangePrinted = true;
                    printedNoChange = false;  // Se detectó cambio, reiniciamos la bandera
                }
            }
        }
        // Si se procesaron mensajes, pero ninguno mostró cambio, y aún no se imprimió "No hay cambios"
        if (mensajeProcesado && !anyChangePrinted && !printedNoChange) {
            Serial.println("No hay cambios");
            printedNoChange = true;  // Se imprime solo una vez
        }
    }

    // Enviar comando continuamente si está activado
    if (send_command && currentMillis - lastSendTime >= sendInterval) {
        lastSendTime = currentMillis;
        sendSteerToAngle(gain_mode, target_angle);
    }
}
