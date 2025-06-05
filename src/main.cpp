#include <Arduino.h>
#include "config.h"
#include <HardwareSerial.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <SPI.h>
#include <SD.h>
#include <Dynamixel2Arduino.h>

// ============================================================================
// DEFINICIONES Y CONSTANTES
// ============================================================================

// Pines
#define VOLTAJE_HALL     35
#define VOLTAJE_BATERIA  34
#define ENCODER_CLK      32
#define ENCODER_DT       33
#define ENCODER_SW       25
#define RS485_TX_PIN     22
#define RS485_RX_PIN     21
#define RS485_EN_PIN     17
#define RS485_SE_PIN     19
#define PIN_5V_EN        16

// Configuración Dynamixel
#define DXL_PROTOCOL_VERSION 2.0
#define DXL_BAUDRATE         57600
#define DXL_ID               1

// Constantes del sistema
namespace Config {
    const unsigned long CALIBRATION_TIME_MS = 10000;
    const unsigned long MAIN_LOOP_INTERVAL_MS = 3;
    const unsigned long SMOOTH_INTERVAL_MS = 20;
    const unsigned long CAN_TIMEOUT_MS = 100;
    const unsigned long DXL_TIMEOUT_MS = 50;
    
    const float STEERING_RANGE_DEG = 900.0f;
    const float MAX_WHEEL_TORQUE = 8.0f;
    const float MAX_RACK_TORQUE = 40.0f;
    const float VIRTUAL_TORQUE_LIMIT = 5.0f;
    const float TORQUE_ZONE_DEG = 10.0f;
    const float CENTERING_STIFFNESS = 0.05f;
    const float CENTERING_DEADZONE = 3.0f;
    const float DAMPING_COEFFICIENT = 0.02f;
    
    const float CURRENT_SENSITIVITY = 0.0160f;
    const float TORQUE_CURRENT_RATIO = 1.2f;
    const int SPEED_MIN = 0;
    const int SPEED_MAX = 255;
    const int SPEED_STEP = 5;
    
    const uint8_t MEDIAN_KERNEL_SIZE = 3;
    const uint8_t MOVING_AVG_SIZE = 5;
    const float IIR_ALPHA = 0.5f;
    const float SMOOTH_ALPHA = 0.1f;
}

// ============================================================================
// ENUMERACIONES Y ESTRUCTURAS
// ============================================================================

enum SystemState {
    INITIALIZING,
    CALIBRATING,
    NORMAL_OPERATION,
    ERROR_STATE,
    SAFE_MODE
};

struct SensorData {
    float voltage;
    float current;
    float position_deg;
    float angular_velocity;
    uint32_t timestamp;
    bool valid;
    
    SensorData() : voltage(0), current(0), position_deg(0), 
                   angular_velocity(0), timestamp(0), valid(false) {}
};

struct CANData {
    uint8_t duty;
    uint8_t current;
    float voltage;
    uint8_t switch_pos;
    uint8_t temperature;
    uint8_t angle_dir;
    uint8_t map_value;
    uint8_t error_code;
    uint32_t last_update;
    bool valid;
    
    CANData() : duty(0), current(0), voltage(0), switch_pos(0), 
                temperature(0), angle_dir(0), map_value(0), error_code(0),
                last_update(0), valid(false) {}
};

struct ControlState {
    float feedback_torque;
    float centering_torque;
    float virtual_torque;
    float total_torque;
    int16_t dynamixel_current_ma;
    bool torque_enabled;
    bool autocentering_enabled;
    
    ControlState() : feedback_torque(0), centering_torque(0), virtual_torque(0),
                     total_torque(0), dynamixel_current_ma(0), 
                     torque_enabled(true), autocentering_enabled(false) {}
};

// ============================================================================
// CLASES DE FILTRADO
// ============================================================================

class MedianFilter {
private:
    float* buffer;
    uint8_t size;
    uint8_t index;
    
public:
    MedianFilter(uint8_t sz) : size(sz), index(0) {
        buffer = new float[size]();
    }
    
    ~MedianFilter() { delete[] buffer; }
    
    float update(float value) {
        buffer[index] = value;
        index = (index + 1) % size;
        
        // Crear copia temporal y ordenar
        float temp[size];
        memcpy(temp, buffer, size * sizeof(float));
        
        // Bubble sort simple para arrays pequeños
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
};

class MovingAverageFilter {
private:
    float* buffer;
    float sum;
    uint8_t size;
    uint8_t index;
    bool filled;
    
public:
    MovingAverageFilter(uint8_t sz) : size(sz), index(0), sum(0), filled(false) {
        buffer = new float[size]();
    }
    
    ~MovingAverageFilter() { delete[] buffer; }
    
    float update(float value) {
        sum -= buffer[index];
        buffer[index] = value;
        sum += value;
        index = (index + 1) % size;
        
        if (index == 0) filled = true;
        
        return sum / (filled ? size : (index == 0 ? size : index));
    }
};

class IIRFilter {
private:
    float value;
    float alpha;
    bool initialized;
    
public:
    IIRFilter(float a) : alpha(a), value(0), initialized(false) {}
    
    float update(float input) {
        if (!initialized) {
            value = input;
            initialized = true;
        } else {
            value = alpha * input + (1.0f - alpha) * value;
        }
        return value;
    }
    
    float getValue() const { return value; }
};

// ============================================================================
// VARIABLES GLOBALES
// ============================================================================

// Hardware
HardwareSerial DXL_SERIAL(2);
Dynamixel2Arduino dxl(DXL_SERIAL, RS485_EN_PIN);
CAN_device_t CAN_cfg;

// Estado del sistema
SystemState system_state = INITIALIZING;
SensorData sensor_data;
CANData can_data;
ControlState control_state;

// Calibración
float vref = 0.0f;
float vref_sum = 0.0f;
unsigned long vref_samples = 0;
unsigned long calibration_start = 0;

// Filtros
MedianFilter* median_filter = nullptr;
MovingAverageFilter* movavg_filter = nullptr;
IIRFilter* iir_filter = nullptr;
IIRFilter* smooth_torque_filter = nullptr;

// Control de posición
int32_t position_offset = 0;
float previous_position = 0.0f;
unsigned long previous_position_time = 0;

// Encoder y UI
int simulated_speed = 0;
int encoder_clk_last = HIGH;
int button_state = HIGH;
int button_state_last = HIGH;
int button_value = 0;

// Timing
unsigned long last_main_loop = 0;
unsigned long last_smooth_update = 0;
unsigned long last_can_rx = 0;
unsigned long last_dxl_comm = 0;

// Debug y logging
bool debug_enabled = true;
unsigned long debug_counter = 0;

// ============================================================================
// FUNCIONES DE UTILIDAD
// ============================================================================

float constrainf(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

bool isTimeout(unsigned long last_time, unsigned long timeout_ms) {
    return (millis() - last_time) > timeout_ms;
}

float mapf(float value, float in_min, float in_max, float out_min, float out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ============================================================================
// FUNCIONES DE VALIDACIÓN Y SEGURIDAD
// ============================================================================

bool validateSensorData() {
    bool valid = true;

    if (sensor_data.voltage <= 0.1f || sensor_data.voltage >= 3.2f) {
        Serial.print("[ERROR] Voltaje fuera de rango: ");
        Serial.println(sensor_data.voltage, 3);
        valid = false;
    }

    if (abs(sensor_data.current) >= 50.0f) {
        Serial.print("[ERROR] Corriente fuera de rango: ");
        Serial.println(sensor_data.current, 3);
        valid = false;
    }

    if (abs(sensor_data.position_deg) > Config::STEERING_RANGE_DEG / 2.0f) {
        Serial.print("[ERROR] Ángulo fuera de rango: ");
        Serial.println(sensor_data.position_deg, 2);
        valid = false;
    }

    return valid;
}


bool validateCANData() {
    return can_data.valid && !isTimeout(can_data.last_update, Config::CAN_TIMEOUT_MS);
}

void enterSafeMode(const char* reason) {
    system_state = SAFE_MODE;
    control_state.torque_enabled = false;
    dxl.torqueOff(DXL_ID);
    
    if (debug_enabled) {
        Serial.print("⚠️ MODO SEGURO: ");
        Serial.println(reason);
    }
}

void checkSystemHealth() {
    // Verificar timeouts de comunicación
    if (isTimeout(last_can_rx, Config::CAN_TIMEOUT_MS * 3)) {
        enterSafeMode("Timeout CAN");
        return;
    }
    
    if (isTimeout(last_dxl_comm, Config::DXL_TIMEOUT_MS * 3)) {
        enterSafeMode("Timeout Dynamixel");
        return;
    }
    
    // Verificar datos de sensores
    if (!validateSensorData()) {
        enterSafeMode("Datos de sensor inválidos");
        return;
    }
    
    // Si estamos en modo seguro, intentar recuperación
    if (system_state == SAFE_MODE) {
        if (validateSensorData() && validateCANData()) {
            system_state = NORMAL_OPERATION;
            if (debug_enabled) {
                Serial.println("✅ Sistema recuperado del modo seguro");
            }
        }
    }
}

// ============================================================================
// FUNCIONES DE LECTURA DE SENSORES
// ============================================================================

void readCurrentSensor() {
    int adc_reading = analogRead(VOLTAJE_HALL);
    float voltage = (adc_reading / 4095.0f) * 3.3f;
    
    // Aplicar filtros en cascada
    float median_voltage = median_filter->update(voltage);
    float avg_voltage = movavg_filter->update(median_voltage);
    float filtered_voltage = iir_filter->update(avg_voltage);
    
    sensor_data.voltage = filtered_voltage;
    sensor_data.current = (filtered_voltage - vref) / Config::CURRENT_SENSITIVITY;
    sensor_data.timestamp = millis();
    sensor_data.valid = true;
}

void readPositionSensor() {
    int32_t current_pos = dxl.getPresentPosition(DXL_ID);
    if (current_pos == -1) return; // Error de lectura
    
    last_dxl_comm = millis();
    
    int32_t delta_pos = current_pos - position_offset;
    float new_position = (delta_pos / 4096.0f) * 360.0f;
    new_position = constrainf(new_position, -Config::STEERING_RANGE_DEG / 2.0f, 
                                           Config::STEERING_RANGE_DEG / 2.0f);
    
    // Calcular velocidad angular
    unsigned long current_time = millis();
    if (previous_position_time > 0) {
        float dt = (current_time - previous_position_time) / 1000.0f;
        if (dt > 0) {
            sensor_data.angular_velocity = (new_position - previous_position) / dt;
        }
    }
    
    sensor_data.position_deg = new_position;
    previous_position = new_position;
    previous_position_time = current_time;
}

void readEncoder() {
    int current_clk = digitalRead(ENCODER_CLK);
    if (current_clk != encoder_clk_last && current_clk == LOW) {
        if (digitalRead(ENCODER_DT) != current_clk) {
            simulated_speed += Config::SPEED_STEP;
        } else {
            simulated_speed -= Config::SPEED_STEP;
        }
        simulated_speed = constrain(simulated_speed, Config::SPEED_MIN, Config::SPEED_MAX);
    }
    encoder_clk_last = current_clk;
    
    // Leer botón
    button_state = digitalRead(ENCODER_SW);
    if (button_state == LOW && button_state_last == HIGH) {
        button_value = (button_value + 1) % 6;
    }
    button_state_last = button_state;
}

void readCANData() {
    CAN_frame_t rx_frame;
    while (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 0) == pdTRUE) {
        last_can_rx = millis();
        
        if (rx_frame.MsgID == 0x290) {
            can_data.duty = rx_frame.data.u8[1];
            can_data.current = rx_frame.data.u8[2];
            can_data.voltage = rx_frame.data.u8[3] * 0.1f;
            can_data.switch_pos = rx_frame.data.u8[4];
            can_data.temperature = rx_frame.data.u8[5];
            can_data.last_update = millis();
            can_data.valid = true;
        } else if (rx_frame.MsgID == 0x292) {
            can_data.angle_dir = rx_frame.data.u8[0];
            can_data.map_value = rx_frame.data.u8[3];
            can_data.error_code = rx_frame.data.u8[4];
        }
    }
}

// ============================================================================
// FUNCIONES DE CONTROL
// ============================================================================

float calculateFeedbackTorque() {
    // Torque basado en la corriente de la cremallera (resistencia del camino)
    float road_torque = -Config::TORQUE_CURRENT_RATIO * sensor_data.current;
    
    // Escalar al rango del volante
    return constrainf(road_torque * (Config::MAX_WHEEL_TORQUE / Config::MAX_RACK_TORQUE),
                     -Config::MAX_WHEEL_TORQUE, Config::MAX_WHEEL_TORQUE);
}

float calculateCenteringTorque() {
    if (!control_state.autocentering_enabled) return 0.0f;
    
    float abs_position = abs(sensor_data.position_deg);
    if (abs_position <= Config::CENTERING_DEADZONE) return 0.0f;
    
    // Torque de centrado proporcional a la posición
    float centering = -Config::CENTERING_STIFFNESS * sensor_data.position_deg;
    
    // Amortiguación proporcional a la velocidad
    float damping = -Config::DAMPING_COEFFICIENT * sensor_data.angular_velocity;
    
    return centering + damping;
}

float calculateVirtualStops() {
    float half_range = Config::STEERING_RANGE_DEG / 2.0f;
    
    if (sensor_data.position_deg > half_range - Config::TORQUE_ZONE_DEG) {
        // Tope derecho
        float penetration = sensor_data.position_deg - (half_range - Config::TORQUE_ZONE_DEG);
        return -Config::VIRTUAL_TORQUE_LIMIT * (penetration / Config::TORQUE_ZONE_DEG);
    } else if (sensor_data.position_deg < -half_range + Config::TORQUE_ZONE_DEG) {
        // Tope izquierdo
        float penetration = abs(sensor_data.position_deg) - (half_range - Config::TORQUE_ZONE_DEG);
        return Config::VIRTUAL_TORQUE_LIMIT * (penetration / Config::TORQUE_ZONE_DEG);
    }
    
    return 0.0f;
}

void updateControlSystem() {
    if (system_state != NORMAL_OPERATION) return;
    
    // Calcular componentes de torque
    control_state.feedback_torque = calculateFeedbackTorque();
    control_state.centering_torque = calculateCenteringTorque();
    control_state.virtual_torque = calculateVirtualStops();
    
    // Sumar todos los componentes
    control_state.total_torque = control_state.feedback_torque + 
                                control_state.centering_torque + 
                                control_state.virtual_torque;
    
    // Aplicar suavizado
    if (millis() - last_smooth_update >= Config::SMOOTH_INTERVAL_MS) {
        control_state.total_torque = smooth_torque_filter->update(control_state.total_torque);
        last_smooth_update = millis();
    }
    
    // Convertir a corriente del Dynamixel
    control_state.dynamixel_current_ma = 
        (control_state.total_torque / Config::MAX_WHEEL_TORQUE) * 1193;
    control_state.dynamixel_current_ma = 
        constrain(control_state.dynamixel_current_ma, -1193, 1193);
}

void applyTorqueControl() {
    if (!control_state.torque_enabled || system_state != NORMAL_OPERATION) {
        dxl.torqueOff(DXL_ID);
        return;
    }
    
    dxl.torqueOn(DXL_ID);
    dxl.setGoalCurrent(DXL_ID, control_state.dynamixel_current_ma, UNIT_MILLI_AMPERE);
    last_dxl_comm = millis();
}

// ============================================================================
// FUNCIONES DE COMUNICACIÓN
// ============================================================================

void sendCANData() {
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = 0x298;
    tx_frame.FIR.B.DLC = 8;
    
    // Convertir posición a bits de dirección
    float direction_bits_f = mapf(sensor_data.position_deg + Config::STEERING_RANGE_DEG / 2.0f,
                                 0, Config::STEERING_RANGE_DEG, 10.0f, 250.0f);
    uint8_t direction_bits = static_cast<uint8_t>(constrainf(direction_bits_f, 10.0f, 250.0f));
    
    uint8_t centered_bit = (abs(sensor_data.position_deg) < 5.0f) ? 1 : 0;
    
    tx_frame.data.u8[0] = 5;
    tx_frame.data.u8[1] = direction_bits;
    tx_frame.data.u8[2] = centered_bit;
    tx_frame.data.u8[3] = static_cast<uint8_t>(simulated_speed);
    tx_frame.data.u8[4] = 0x00;
    tx_frame.data.u8[5] = 0x00;
    tx_frame.data.u8[6] = 0x00;
    tx_frame.data.u8[7] = 0x00;
    
    ESP32Can.CANWriteFrame(&tx_frame);
}

void processSerialCommands() {
    while (Serial.available()) {
        char c = Serial.read();
        switch (c) {
            case '1':
                control_state.torque_enabled = !control_state.torque_enabled;
                Serial.print("🔧 Torque ");
                Serial.println(control_state.torque_enabled ? "ACTIVADO" : "DESACTIVADO");
                break;
                
            case '2':
                control_state.autocentering_enabled = !control_state.autocentering_enabled;
                Serial.print("📐 Autocentrado ");
                Serial.println(control_state.autocentering_enabled ? "ACTIVADO" : "DESACTIVADO");
                break;
                
            case 'd':
                debug_enabled = !debug_enabled;
                Serial.print("🐛 Debug ");
                Serial.println(debug_enabled ? "ACTIVADO" : "DESACTIVADO");
                break;
                
            case 'r':
                if (system_state == SAFE_MODE) {
                    system_state = NORMAL_OPERATION;
                    Serial.println("🔄 Sistema reiniciado");
                }
                break;
        }
    }
}

void printDebugInfo() {
    if (!debug_enabled || (debug_counter++ % 100 != 0)) return;
    
    Serial.print(sensor_data.voltage, 4); Serial.print(",");
    Serial.print(can_data.duty); Serial.print(",");
    Serial.print(can_data.current); Serial.print(",");
    Serial.print(can_data.voltage, 1); Serial.print(",");
    Serial.print(can_data.switch_pos); Serial.print(",");
    Serial.print(can_data.temperature); Serial.print(",");
    Serial.print(can_data.angle_dir); Serial.print(",");
    Serial.print(can_data.map_value); Serial.print(",");
    Serial.print(can_data.error_code); Serial.print(",");
    Serial.print(sensor_data.position_deg, 2); Serial.print(",");
    Serial.print(button_value); Serial.print(",");
    Serial.print(sensor_data.current, 4); Serial.print(",");
    Serial.print(control_state.feedback_torque, 4); Serial.print(",");
    Serial.print(control_state.centering_torque, 4); Serial.print(",");
    Serial.print(control_state.virtual_torque, 4); Serial.print(",");
    Serial.print(control_state.total_torque, 4); Serial.print(",");
    Serial.print(control_state.dynamixel_current_ma); Serial.print(",");
    Serial.println(static_cast<int>(system_state));
}

// ============================================================================
// FUNCIONES DE CALIBRACIÓN E INICIALIZACIÓN
// ============================================================================

void performCalibration() {
    int adc_reading = analogRead(VOLTAJE_HALL);
    float voltage = (adc_reading / 4095.0f) * 3.3f;
    vref_sum += voltage;
    vref_samples++;
    
    if (millis() - calibration_start >= Config::CALIBRATION_TIME_MS) {
        vref = vref_sum / vref_samples;
        system_state = NORMAL_OPERATION;
        
        Serial.print("✅ Calibración completada. VREF = ");
        Serial.print(vref, 4);
        Serial.println(" V");
        
        if (debug_enabled) {
            Serial.println("V_hall,MotorDuty,I_CAN,V_CAN,Switch,Temp,DirAngle,Mapa,Error,Pos_deg,Button,I_final,T_feedback,T_centering,T_virtual,T_total,I_dxl_mA,State");
        }
    }
}

bool initializeHardware() {
    // Configurar pines
    pinMode(PIN_5V_EN, OUTPUT);
    digitalWrite(PIN_5V_EN, HIGH);
    pinMode(RS485_EN_PIN, OUTPUT);
    digitalWrite(RS485_EN_PIN, LOW);
    pinMode(RS485_SE_PIN, OUTPUT);
    digitalWrite(RS485_SE_PIN, HIGH);
    pinMode(CAN_SE_PIN, OUTPUT);
    digitalWrite(CAN_SE_PIN, LOW);
    pinMode(ENCODER_CLK, INPUT);
    pinMode(ENCODER_DT, INPUT);
    pinMode(ENCODER_SW, INPUT_PULLUP);
    
    delay(100);
    
    // Inicializar Serial
    Serial.begin(921600);
    analogReadResolution(12);
    
    // Inicializar Dynamixel
    DXL_SERIAL.begin(DXL_BAUDRATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
    dxl.begin();
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
    
    if (dxl.ping(DXL_ID)) {
        Serial.println("✅ Dynamixel detectado.");
        dxl.torqueOff(DXL_ID);
        position_offset = dxl.getPresentPosition(DXL_ID);
        Serial.print("📍 Offset posición inicial: ");
        Serial.println(position_offset);
    } else {
        Serial.println("❌ Dynamixel no responde.");
        return false;
    }
    
    // Inicializar CAN
    CAN_cfg.speed = CAN_SPEED_250KBPS;
    CAN_cfg.tx_pin_id = GPIO_NUM_27;
    CAN_cfg.rx_pin_id = GPIO_NUM_26;
    CAN_cfg.rx_queue = xQueueCreate(10, sizeof(CAN_frame_t));
    ESP32Can.CANInit();
    
    // Inicializar filtros
    median_filter = new MedianFilter(Config::MEDIAN_KERNEL_SIZE);
    movavg_filter = new MovingAverageFilter(Config::MOVING_AVG_SIZE);
    iir_filter = new IIRFilter(Config::IIR_ALPHA);
    smooth_torque_filter = new IIRFilter(Config::SMOOTH_ALPHA);
    
    encoder_clk_last = digitalRead(ENCODER_CLK);
    
    return true;
}

// ============================================================================
// FUNCIONES PRINCIPALES
// ============================================================================

void setup() {
    system_state = INITIALIZING;
    
    if (!initializeHardware()) {
        system_state = ERROR_STATE;
        while (true) {
            Serial.println("❌ Error de inicialización. Sistema detenido.");
            delay(5000);
        }
    }
    
    // Iniciar calibración
    system_state = CALIBRATING;
    calibration_start = millis();
    Serial.println("⚙️ Calibrando VREF durante 10 segundos, mantén el motor en idle...");
}

void loop() {
    unsigned long current_time = millis();
    
    // Procesamiento según el estado del sistema
    switch (system_state) {
        case CALIBRATING:
            performCalibration();
            return;
            
        case ERROR_STATE:
        case SAFE_MODE:
            processSerialCommands();
            delay(100);
            return;
            
        default:
            break;
    }
    
    // Loop principal - ejecutar a intervalos regulares
    if (current_time - last_main_loop >= Config::MAIN_LOOP_INTERVAL_MS) {
        last_main_loop = current_time;
        
        // Leer sensores y entradas
        readCANData();
        readEncoder();
        readCurrentSensor();
        readPositionSensor();
        
        // Verificar salud del sistema
        checkSystemHealth();
        
        if (system_state == NORMAL_OPERATION) {
            // Actualizar sistema de control
            updateControlSystem();
            applyTorqueControl();
            sendCANData();
        }
        
        // Debug y comunicación serie
        printDebugInfo();
    }
    
    // Procesar comandos serie (sin bloqueo)
    processSerialCommands();
}