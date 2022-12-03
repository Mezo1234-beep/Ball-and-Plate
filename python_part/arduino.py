import serial
import serial.tools.list_ports
import time


class Arduino:
    """
    A class that handles the connection with an Arduino board
    """

    def __init__(self):
        self.arduinoIsConnected = False
        self.ser = None
        self.status_label = None

    def set_label(self, label) -> None:
        self.status_label = label

    def connectArduino(self) -> None:
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            if "Arduino" or "CH340" in port.description:
                self.ser = serial.Serial(port[0], 19200, timeout=1)
                time.sleep(1)
                self.status_label.configure(text=f"{port}", fg="#36db8b")
                self.arduinoIsConnected = True
