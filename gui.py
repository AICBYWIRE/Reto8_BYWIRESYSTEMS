import sys
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QComboBox, QGridLayout
from PyQt5.QtCore import QThread, pyqtSignal

class SerialReader(QThread):
    data_received = pyqtSignal(dict)  # Enviar datos como diccionario

    def __init__(self, puerto, baudrate=115200):
        super().__init__()
        self.puerto = puerto
        self.baudrate = baudrate
        self.esp32 = None
        self.running = False  # Control del hilo

    def run(self):
        """Lee datos del ESP32 y los envía en un diccionario"""
        try:
            self.esp32 = serial.Serial(self.puerto, self.baudrate, timeout=1)
            self.running = True
            while self.running:
                if self.esp32.in_waiting:
                    data = self.esp32.readline().decode('utf-8').strip()
                    if data:
                        valores = self.procesar_datos(data)
                        self.data_received.emit(valores)  # Enviar datos procesados
        except serial.SerialException as e:
            self.data_received.emit({"Error": str(e)})

    def procesar_datos(self, data):
        """Convierte los datos en un diccionario"""
        try:
            partes = data.split(",")  # Suponiendo formato: "10,50,2,12.5,3,25"
            return {
                "Torque": int(partes[0]),
                "Motor Duty": int(partes[1]),
                "Corriente": int(partes[2]),
                "Voltaje": float(partes[3]),
                "Switch Pos": int(partes[4]),
                "Temperatura": int(partes[5])
            }
        except:
            return {"Error": "Formato inválido"}

    def stop(self):
        """Detiene el hilo y cierra el puerto serie"""
        self.running = False
        if self.esp32 and self.esp32.is_open:
            self.esp32.close()
        self.quit()
        self.wait()

class ESP32_CAN_GUI(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.serial_thread = None

    def initUI(self):
        """Diseño de la GUI con etiquetas fijas y datos dinámicos"""
        self.setWindowTitle('ESP32 CAN Monitor')
        self.setGeometry(100, 100, 400, 300)

        layout = QVBoxLayout()
        self.label_estado = QLabel("Estado: Desconectado")
        layout.addWidget(self.label_estado)

        self.cmbPuertos = QComboBox()
        self.cargar_puertos()
        layout.addWidget(self.cmbPuertos)

        self.connect_button = QPushButton("Conectar a ESP32")
        self.connect_button.clicked.connect(self.conectar_esp32)
        layout.addWidget(self.connect_button)

        # 📌 GRID Layout para mostrar los datos de forma ordenada
        self.grid = QGridLayout()
        layout.addLayout(self.grid)

        # 📌 Diccionario de etiquetas (texto fijo y valores dinámicos)
        self.labels = {
            "Torque": QLabel("🔩 Torque:"),
            "Motor Duty": QLabel("⚡ Motor Duty:"),
            "Corriente": QLabel("🔋 Corriente:"),
            "Voltaje": QLabel("⚡ Voltaje:"),
            "Switch Pos": QLabel("🔘 Switch Pos:"),
            "Temperatura": QLabel("🌡️  Temperatura:")
        }

        self.values = {
            "Torque": QLabel("[ 0 Nm ]"),
            "Motor Duty": QLabel("[ 0 % ]"),
            "Corriente": QLabel("[ 0 A ]"),
            "Voltaje": QLabel("[ 0.0 V ]"),
            "Switch Pos": QLabel("[ 0 ]"),
            "Temperatura": QLabel("[ 0 °C ]")
        }

        # 📌 Agregar etiquetas y valores al grid
        row = 0
        for key in self.labels:
            self.grid.addWidget(self.labels[key], row, 0)   # Texto fijo
            self.grid.addWidget(self.values[key], row, 1)   # Valor dinámico
            row += 1

        self.setLayout(layout)

    def cargar_puertos(self):
        """Carga los puertos disponibles en el ComboBox"""
        puertos = [port.device for port in serial.tools.list_ports.comports()]
        self.cmbPuertos.addItems(puertos)

    def conectar_esp32(self):
        """Conectar al ESP32 y empezar a recibir datos"""
        puerto = self.cmbPuertos.currentText()
        if puerto:
            if self.serial_thread and self.serial_thread.isRunning():
                self.serial_thread.stop()
                self.serial_thread.wait()

            self.serial_thread = SerialReader(puerto)
            self.serial_thread.data_received.connect(self.actualizar_datos)
            self.serial_thread.start()

            self.label_estado.setText(f"✅ Conectado a {puerto}")

    def actualizar_datos(self, datos):
        """Actualiza los valores en la GUI"""
        if "Error" in datos:
            self.label_estado.setText(f"❌ {datos['Error']}")
            return

        for key in self.values:
            self.values[key].setText(f"[ {datos[key]} ]")

    def closeEvent(self, event):
        """Cerrar el hilo de lectura al salir de la GUI"""
        if self.serial_thread and self.serial_thread.isRunning():
            self.serial_thread.stop()
            self.serial_thread.wait()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = ESP32_CAN_GUI()
    gui.show()
    sys.exit(app.exec_())
