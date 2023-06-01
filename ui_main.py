#Codigo para conectar con interfaz. 
#Se crea interfaz para permitir comunicación serial y con adafruit IO a través de la nube. 
#La comunicación serial se pretende para usar con el PIC16F887  aun atasas de 9600

import sys
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import QApplication, QMainWindow, QLabel, QSlider, QVBoxLayout, QWidget, QPushButton, QComboBox
from Adafruit_IO import Client, Feed, RequestError
import serial

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        #configuración inicial de la ventana
        self.setWindowTitle("Proyecto 2")
        self.resize(600, 400)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        #Layout central
        layout = QVBoxLayout(central_widget)

        #Labels
        label1 = QLabel("Cabeza")
        label2 = QLabel("Cuello")
        label3 = QLabel("Femur")
        label4 = QLabel("Antebrazo")

        #Sliders
        self.slider1 = QSlider(Qt.Horizontal)
        self.slider2 = QSlider(Qt.Horizontal)
        self.slider3 = QSlider(Qt.Horizontal)
        self.slider4 = QSlider(Qt.Horizontal)

        # valores máximos y mínimos
        self.slider1.setMinimum(0)
        self.slider2.setMinimum(0)
        self.slider3.setMinimum(0)
        self.slider4.setMinimum(0)

        self.slider1.setMaximum(255)
        self.slider2.setMaximum(255)
        self.slider3.setMaximum(255)
        self.slider4.setMaximum(255)

        # Despliegue de sus valores
        self.value_label1 = QLabel("0")
        self.value_label2 = QLabel("0")
        self.value_label3 = QLabel("0")
        self.value_label4 = QLabel("0")

        # Se eagregan widgets adiciones para cada sliderr con su etiqueta. 
        slider_widget1 = QWidget()
        slider_widget2 = QWidget()
        slider_widget3 = QWidget()
        slider_widget4 = QWidget()

        # Cada etiqueta tiene su propio layout
        slider_layout1 = QVBoxLayout(slider_widget1)
        slider_layout2 = QVBoxLayout(slider_widget2)
        slider_layout3 = QVBoxLayout(slider_widget3)
        slider_layout4 = QVBoxLayout(slider_widget4)

        slider_layout1.addWidget(label1)
        slider_layout1.addWidget(self.slider1)
        slider_layout1.addWidget(self.value_label1)

        slider_layout2.addWidget(label2)
        slider_layout2.addWidget(self.slider2)
        slider_layout2.addWidget(self.value_label2)

        slider_layout3.addWidget(label3)
        slider_layout3.addWidget(self.slider3)
        slider_layout3.addWidget(self.value_label3)

        slider_layout4.addWidget(label4)
        slider_layout4.addWidget(self.slider4)
        slider_layout4.addWidget(self.value_label4)

        # La señal de released se conecta a cada objeto
        self.slider1.sliderReleased.connect(lambda: self.slider_released(self.slider1, self.value_label1))
        self.slider2.sliderReleased.connect(lambda: self.slider_released(self.slider2, self.value_label2))
        self.slider3.sliderReleased.connect(lambda: self.slider_released(self.slider3, self.value_label3))
        self.slider4.sliderReleased.connect(lambda: self.slider_released(self.slider4, self.value_label4))

        layout.addWidget(slider_widget1)
        layout.addWidget(slider_widget2)
        layout.addWidget(slider_widget3)
        layout.addWidget(slider_widget4)

        # Prev values
        self.prev_values = {
            self.slider1: self.slider1.value(),
            self.slider2: self.slider2.value(),
            self.slider3: self.slider3.value(),
            self.slider4: self.slider4.value()
        }

        # Elementos adicionales para comunicación serial y Adafruit
        self.pushButton = QPushButton("CONNECT")
        self.adafruit_mode_button = QPushButton("Adafruit Mode")
        self.comboBox = QComboBox()
        self.comboBox.addItems(["COM0", "COM1", "COM2", "COM3", "COM4", "COM5", "COM6", "COM7", "COM8", "COM9", "COM10"])
        self.pushButton_3 = QPushButton("CLOSE")

        layout.addWidget(self.pushButton)
        layout.addWidget(self.comboBox)
        layout.addWidget(self.adafruit_mode_button)
        layout.addWidget(self.pushButton_3)

        self.serial_port = None
        self.pushButton.clicked.connect(self.connect_port)
        self.pushButton_3.clicked.connect(self.close_port)
        self.adafruit_mode_button.clicked.connect(self.toggle_adafruit_mode)
        
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.receive_data)
        self.timer.start(250)
        ADAFRUIT_IO_KEY = 'aio_rzPb15do9vEbkISElKT6WJqUC0OA'

        # Set to your Adafruit IO username.
        # (go to https://accounts.adafruit.com to find your username)
        ADAFRUIT_IO_USERNAME = 'estuardo02'
                # Create an instance of the REST client.
        self.aio = Client(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)
        self.lectura_prev = 0
        self.adafruit_mode = False
        try: # analog feed de salida de PC, entradad para el pic
            self.feed_01 = self.aio.feeds('cabeza-gato')
        except RequestError: # create an `analog` feed
            feed = Feed(name='cabeza-gato')
            self.feed_01 = self.aio.create_feed(feed)
        try: # analog feed de salida de PC, entradad para el pic
            self.feed_02 = self.aio.feeds('cuello-gato')
        except RequestError: # create an `analog` feed
            feed = Feed(name='cuello-gato')
            self.feed_02 = self.aio.create_feed(feed)
        try: # analog feed de salida de PC, entradad para el pic
            self.feed_03 = self.aio.feeds('femur-gato')
        except RequestError: # create an `analog` feed
            feed = Feed(name='femur-gato')
            self.feed_03 = self.aio.create_feed(feed) 
        try: # analog feed de salida de PC, entradad para el pic
            self.feed_04 = self.aio.feeds('antebrazo-gato')
        except RequestError: # create an `analog` feed
            feed = Feed(name='antebrazo-gato')
            self.feed_04 = self.aio.create_feed(feed) 



    def toggle_adafruit_mode(self):
        self.adafruit_mode = not self.adafruit_mode
        if self.adafruit_mode:
            self.slider1.setEnabled(False)
            self.slider2.setEnabled(False)
            self.slider3.setEnabled(False)
            self.slider4.setEnabled(False)
            self.connect_adafruit(1)
        else:
            self.slider1.setEnabled(True)
            self.slider2.setEnabled(True)
            self.slider3.setEnabled(True)
            self.slider4.setEnabled(True)

    def connect_adafruit(self, feed_name):
        if feed_name == 1:
            lectura = self.aio.receive(self.feed_01.key)
        if feed_name == 2:
            lectura = self.aio.receive(self.feed_02.key)
        if feed_name == 3:
            lectura = self.aio.receive(self.feed_03.key)
        if feed_name == 4:
            lectura = self.aio.receive(self.feed_04.key)
        else:
            pass
        #escritura en consola
        if (lectura.value != self.lectura_prev):
            pass
            #print('received <- ', lectura.value)
        self.lectura_prev = lectura.value
        return lectura.value

    def connect_port(self):
        port_name = self.comboBox.currentText()
        try:
            self.serial_port = serial.Serial(port_name, 9600)
            print(f"Port {port_name} opened")
        except serial.SerialException as e:
            print("Access denied")

    def send_data(self):
        if self.serial_port is not None:
            # Revisar si cambio 
            #if any(slider.value() != self.prev_values[slider] for slider in self.prev_values.keys()):
                # array de 16 bits con la estructura aaax donde a es el valor del slider y x hace referencia al slider. 
                data = f"{self.slider1.value():03d}1{self.slider2.value():03d}2{self.slider3.value():03d}3{self.slider4.value():03d}4"
                # Send the data through the serial port 
                self.serial_port.write(data.encode(encoding="ascii", errors="replace"))

                # Actualizar los valores del diccionario 
                for slider in self.prev_values.keys():
                    self.prev_values[slider] = slider.value()
    #funcion para cerrar puerto serial 
    def close_port(self):
        if self.serial_port is not None:
            self.serial_port.close()
            self.serial_port = None
    #recepción de datos de Adafruit o serial
    def receive_data(self):
        if self.serial_port is not None:
            if self.adafruit_mode:
                data = self.connect_adafruit(1)
                self.slider1.setValue(int(data)) #Se copia el valor del feed y castea a int para que lo interprete el slider. 
                self.slider_released(self.slider1, self.value_label1)
                data = self.connect_adafruit(2)
                self.slider2.setValue(int(data))
                self.slider_released(self.slider2, self.value_label2)
                data = self.connect_adafruit(3)
                self.slider3.setValue(int(data))
                self.slider_released(self.slider3, self.value_label3)
                data = self.connect_adafruit(4)
                self.slider4.setValue(int(data))
                self.slider_released(self.slider4, self.value_label4)
            else:
                data = self.serial_port.read_all().decode(encoding="ascii", errors="replace")
                if len(data) == 20:
                    for i in range(0, 16, 5):
                        if i == 0: #se selecciona cual slider se está escribiendo. 
                            self.slider1.setValue(int(data[1:4])) #se toman los 3 valores relevantes para el slider 
                            self.slider_released(self.slider1, self.value_label1)
                        elif i == 5:
                            self.slider2.setValue(int(data[6:9]))
                            self.slider_released(self.slider2, self.value_label2)
                        elif i == 10:
                            self.slider3.setValue(int(data[11:14]))
                            self.slider_released(self.slider3, self.value_label3)
                        elif i == 15:
                            self.slider4.setValue(int(data[16:19]))
                            self.slider_released(self.slider4, self.value_label4)

    def slider_released(self, slider, label):
        value = slider.value()
        label.setText(str(value))
        if slider in self.prev_values.keys():
            prev_value = self.prev_values[slider]
            if value != prev_value:
                slider_name = slider.objectName()
                #print(f"Slider value changed: {slider_name}, New value: {value}")
                self.prev_values[slider] = value
                self.send_data()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
