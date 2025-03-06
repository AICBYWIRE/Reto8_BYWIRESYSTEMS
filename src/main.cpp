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
unsigned long previousMillis = 0;
const int monitorInterval = 1000;  // Monitoreo cada 1 segundo
const int sendInterval = 5;  // Enviar comando cada 5ms
unsigned long lastMonitorTime = 0;
unsigned long lastSendTime = 0;
bool commandSent = false; // Para mostrar solo una vez el mensaje enviado

// 📌 Variables de control
uint8_t gain_mode = 0x04;  // Valor por defecto: ganancia media
uint8_t target_angle = 0x80;  // Valor por defecto: posición central
bool send_command = false;  // Si se debe enviar comandos continuamente
bool monitoring_mode = false;  // Si se debe monitorear los mensajes CAN

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
    tx_frame.data.u8[1] = angle;  // Ángulo objetivo
    for (int i = 2; i < 8; i++) tx_frame.data.u8[i] = 0; // Bytes no usados
    
    ESP32Can.CANWriteFrame(&tx_frame);
    if (!commandSent) {
        Serial.printf("📤 Comando enviado: 298 %02X %02X 00 00 00 00 00 00\n", gain, angle);
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
    unsigned long currentMillis = millis();

    // 📌 Leer comandos desde la terminal
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input == "1") {
            monitoring_mode = true;
            send_command = false;
            Serial.println("📡 Modo monitoreo activado. Mostrando mensajes 290 y 292 cada 1s...");
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

    // 📌 Monitorear mensajes CAN
    if (monitoring_mode && currentMillis - lastMonitorTime >= monitorInterval) {
        lastMonitorTime = currentMillis;
        if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {
            if (rx_frame.MsgID == 0x290) {
                Serial.println("--- Mensaje #1 ECU (0x290) ---");
                Serial.printf("Torque: %d bits\n", rx_frame.data.u8[0]);
                Serial.printf("Motor Duty: %d %%\n", rx_frame.data.u8[1]);
                Serial.printf("Corriente: %d A\n", rx_frame.data.u8[2]);
                Serial.printf("Voltaje: %.1f V\n", rx_frame.data.u8[3] * 0.1);
                Serial.printf("Switch Pos: %d\n", rx_frame.data.u8[4]);
                Serial.printf("Temperatura ECU: %d C\n", rx_frame.data.u8[5]);
            } else if (rx_frame.MsgID == 0x292) {
                Serial.println("--- Mensaje #2 ECU (0x292) ---");
                Serial.printf("Ángulo dirección 8-bit: %d bits\n", rx_frame.data.u8[0]);
                Serial.printf("Mapa seleccionado: %d\n", rx_frame.data.u8[3]);
                Serial.printf("Código de error: %d\n", rx_frame.data.u8[4]);
            }
        } else {
            Serial.println("⌛ Esperando mensajes CAN...");
        }
    }

    // 📌 Enviar comando continuamente si está activado
    if (send_command && currentMillis - lastSendTime >= sendInterval) {
        lastSendTime = currentMillis;
        sendSteerToAngle(gain_mode, target_angle);
    }
}
