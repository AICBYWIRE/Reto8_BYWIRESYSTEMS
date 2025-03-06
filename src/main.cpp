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
const int interval = 1000;  // Intervalo de envío de mensajes (1s)
unsigned long lastPrintTime = 0; // Marca de tiempo para controlar la impresión
const int printInterval = 5000; // Nuevo intervalo de impresión: 5 segundos

// 📌 Función para probar si la tarjeta SD funciona

void SD_test(void) {
    SPI.begin(SD_SCLK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("SDCard MOUNT FAIL");
    } else {
        uint32_t cardSize = SD.cardSize() / (1024 * 1024);
        Serial.printf("✅ SDCard Size: %dMB\n", cardSize);
    }
}

void setup() {
    pinMode(PIN_5V_EN, OUTPUT);
    digitalWrite(PIN_5V_EN, HIGH);

    pinMode(CAN_SE_PIN, OUTPUT);
    digitalWrite(CAN_SE_PIN, LOW);

    Serial.begin(115200);
    SD_test();
    Serial.println("🚀 ESP32 CAN Monitor Iniciado");

    // 📌 Configuración CAN
    CAN_cfg.speed = CAN_SPEED_250KBPS;
    CAN_cfg.tx_pin_id = GPIO_NUM_27;
    CAN_cfg.rx_pin_id = GPIO_NUM_26;
    CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));

    // 📌 Iniciar CAN
    ESP32Can.CANInit();
    Serial.printf("✅ CAN SPEED: %d kbps\n", CAN_cfg.speed);
}

void loop() {
    CAN_frame_t rx_frame;
    unsigned long currentMillis = millis();

    // 📌 Recibir mensajes CAN
    if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {
        if (currentMillis - lastPrintTime >= printInterval) {  // Solo imprimir cada 5 segundos
            lastPrintTime = currentMillis;
            Serial.printf("\n📌 Mensaje recibido - ID: 0x%X\n", rx_frame.MsgID);

            if (rx_frame.MsgID == 0x290) {
                uint8_t torque = rx_frame.data.u8[0];
                uint8_t motor_duty = rx_frame.data.u8[1];
                uint8_t current = rx_frame.data.u8[2];
                float voltage = rx_frame.data.u8[3] * 0.1;
                uint8_t switch_position = rx_frame.data.u8[4];
                int8_t temperature = rx_frame.data.u8[5];

                Serial.println("--- Mensaje #1 ECU ---");
                Serial.printf("Torque: %d bits\n", torque);
                Serial.printf("Motor Duty: %d %%\n", motor_duty);
                Serial.printf("Corriente: %d A\n", current);
                Serial.printf("Voltaje: %.1f V\n", voltage);
                Serial.printf("Switch Pos: %d\n", switch_position);
                Serial.printf("Temperatura ECU: %d C\n", temperature);
            }
            else if (rx_frame.MsgID == 0x292) {
                int angle_8bit = rx_frame.data.u8[0];
                int angle_10bit = (rx_frame.data.u8[1] << 8) | rx_frame.data.u8[2];
                int map_selected = rx_frame.data.u8[3];
                int error_code = rx_frame.data.u8[4];
                int status_flags = rx_frame.data.u8[6];
                int limit_flags = rx_frame.data.u8[7];

                Serial.println("--- Mensaje #2 ECU ---");
                Serial.printf("Ángulo dirección 8-bit: %d bits\n", angle_8bit);
                Serial.printf("Ángulo dirección 10-bit: %d bits\n", angle_10bit);
                Serial.printf("Mapa seleccionado: %d\n", map_selected);
                Serial.printf("Código de error: %d\n", error_code);
                Serial.printf("Estado: 0b%08b\n", status_flags);
                Serial.printf("Límites: 0b%08b\n", limit_flags);
            }
        }
    }

    // 📌 Enviar mensaje CAN de prueba cada segundo (ID: 0x001, sin afectar a la ECU)
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        CAN_frame_t tx_frame;
        tx_frame.FIR.B.FF = CAN_frame_std;
        tx_frame.MsgID = 0x001; // Se mantiene en 0x001 para prueba
        tx_frame.FIR.B.DLC = 8;
        tx_frame.data.u8[0] = 0x10;
        tx_frame.data.u8[1] = 0x30;
        tx_frame.data.u8[2] = 0x30;
        tx_frame.data.u8[3] = 0x40;
        tx_frame.data.u8[4] = 0x50;
        tx_frame.data.u8[5] = 0x60;
        tx_frame.data.u8[6] = 0x70;
        tx_frame.data.u8[7] = 0x80;

        ESP32Can.CANWriteFrame(&tx_frame);
        Serial.println("📤 Mensaje de prueba enviado (ID: 0x001)");
    }
}
// este programa envía un mensaje vía CAN, además lee los mensajes provenientes del PCAN y de la ECU
