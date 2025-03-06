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
    CAN_cfg.speed = CAN_SPEED_1000KBPS;
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
        // 📌 Solo procesar mensajes correctos con `MsgID == 0x290`
        if (rx_frame.MsgID == 0x290)  
        {
            uint8_t torque = rx_frame.data.u8[0];          // D0
            uint8_t motor_duty = rx_frame.data.u8[1];      // D1 (%)
            uint8_t current = rx_frame.data.u8[2];         // D2 (A)
            float voltage = rx_frame.data.u8[3] * 0.1;     // D3 (1 bit = 100mV)
            uint8_t switch_position = rx_frame.data.u8[4]; // D4 (0-15)
            int8_t temperature = rx_frame.data.u8[5];      // D5 (°C)

            // 📌 Solo imprimir los datos en formato CSV y **sin texto adicional**
            Serial.printf("%d,%d,%d,%.1f,%d,%d\n",
                          torque, motor_duty, current, voltage, switch_position, temperature);
        }
    }

    // 📌 Enviar mensaje CAN de prueba cada segundo **sin texto extra**
    if (currentMillis - previousMillis >= interval) 
    {
        previousMillis = currentMillis;
        CAN_frame_t tx_frame;
        tx_frame.FIR.B.FF = CAN_frame_std;
        tx_frame.MsgID = 0x001;
        tx_frame.FIR.B.DLC = 8;
        tx_frame.data.u8[0] = 0x10;
        tx_frame.data.u8[1] = 0x20;
        tx_frame.data.u8[2] = 0x30;
        tx_frame.data.u8[3] = 0x40;
        tx_frame.data.u8[4] = 0x50;
        tx_frame.data.u8[5] = 0x60;
        tx_frame.data.u8[6] = 0x70;
        tx_frame.data.u8[7] = 0x80;

        ESP32Can.CANWriteFrame(&tx_frame);
    }
}
