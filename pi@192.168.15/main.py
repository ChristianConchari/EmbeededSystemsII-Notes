import sys
from PyQt5 import QtCore, QtGui, QtWidgets, QtSerialPort
from PyQt5 import uic


class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi("mainwindow.ui", self)
        self.setFixedSize(self.size())
        self.connectBox.stateChanged.connect(self.on_toggled)

        self.portText.setPlainText('ttyACM0')
        home_msg = 'home\n'
        stop_msg = 'stop\n'
        # Pressed
        forward_p1 = 'm1fr\n'
        forward_p2 = 'm2fr\n'
        backward_p1 = 'm1br\n'
        backward_p2 = 'm2br\n'
        # Released
        forward_r1 = 'm1fs\n'
        forward_r2 = 'm2fs\n'
        backward_r1 = 'm1bs\n'
        backward_r2 = 'm2bs\n'

        self.homeButton.clicked.connect(lambda: self.send(home_msg))
        self.stopButton.clicked.connect(lambda: self.send(stop_msg))
        self.fwVel1.pressed.connect(lambda: self.send(forward_p1))
        self.fwVel2.pressed.connect(lambda: self.send(forward_p2))
        self.bwVel1.pressed.connect(lambda: self.send(backward_p1))
        self.bwVel2.pressed.connect(lambda: self.send(backward_p2))
        
        self.vel1Slider.valueChanged.connect(self.slider1Report)
        self.vel2Slider.valueChanged.connect(self.slider2Report)

        
        self.fwVel1.released.connect(lambda: self.send(forward_r1))
        self.fwVel2.released.connect(lambda: self.send(forward_r2))
        self.bwVel1.released.connect(lambda: self.send(backward_r1))
        self.bwVel2.released.connect(lambda: self.send(backward_r2))

        self.serial = QtSerialPort.QSerialPort()
        self.serial.setBaudRate(115200)
        self.serial.readyRead.connect(self.receive)

    def slider1Report(self):
        self.vel1Status.setText(str(self.vel1Slider.value()))
        if (self.vel1Slider.value()<=9):
            text = "m1_00"+str(self.vel1Slider.value())+'\n'
        
        if (self.vel1Slider.value()<=99 and self.vel1Slider.value()>=10):
            text = "m1_0"+str(self.vel1Slider.value())+'\n'
        
        if (self.vel1Slider.value()==100):
            text = "m1_"+str(self.vel1Slider.value())+'\n'

        self.send(text)

    def slider2Report(self):
        self.vel2Status.setText(str(self.vel2Slider.value()))
        text = "m2_"+str(self.vel2Slider.value())+'\n'
        self.send(text)


    def send(self, command):
        if self.connectBox.isChecked():
            try: 
                self.serial.write(command.encode())
                return 0
            except:
                print('Error at writing')
                return -1
        else:
            self.setAppStatus('The device is offline')

    def receive(self):
        while self.serial.canReadLine():
            text = self.serial.readLine().data().decode()
            text = text.rstrip('\r\n')
            self.setAppStatus(text)

    def setAppStatus(self, text):
        self.statusText.setText(text)

    def on_toggled(self, checked):   
        try:     
            self.serial.setPortName(self.portText.toPlainText())
            if self.connectBox.isChecked():
                if not self.serial.isOpen():
                    if not self.serial.open(QtCore.QIODevice.ReadWrite):
                        self.setAppStatus("The port couldn't be opened.")
                        self.connectBox.setChecked(False)                        
                    else:
                        self.setAppStatus('Connected.')   
                        self.connectBox.setChecked(True)                     
                else:
                    self.setAppStatus('Port is already opened. Please, check availability.')
                    self.connectBox.setChecked(False)
            else:            
                self.setAppStatus('Disconnected.')
                self.connectBox.setChecked(False)
                self.serial.close()
        except Exception as e:
            self.connectBox.setChecked(False)
            self.setAppStatus(str(e))
            print(e)


        


app = QtWidgets.QApplication(sys.argv)
window = MainWindow()
window.show()
app.exec_()

'''
class Widget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(Widget, self).__init__(parent)
        self.message_le = QtWidgets.QLineEdit()
        self.send_btn = QtWidgets.QPushButton(
            text="Send",
            clicked=self.send
        )
        self.output_te = QtWidgets.QTextEdit(readOnly=True)
        self.button = QtWidgets.QPushButton(
            text="Connect",
            checkable=True,
            toggled=self.on_toggled
        )
        lay = QtWidgets.QVBoxLayout(self)
        hlay = QtWidgets.QHBoxLayout()
        hlay.addWidget(self.message_le)
        hlay.addWidget(self.send_btn)
        lay.addLayout(hlay)
        lay.addWidget(self.output_te)
        lay.addWidget(self.button)

        self.serial = QtSerialPort.QSerialPort(
            '/dev/tty.usbmodem14201',
            baudRate=QtSerialPort.QSerialPort.Baud9600,
            readyRead=self.receive
        )

    @QtCore.pyqtSlot()
    def receive(self):
        while self.serial.canReadLine():
            text = self.serial.readLine().data().decode()
            text = text.rstrip('\r\n')
            self.output_te.append(text)

    @QtCore.pyqtSlot()
    def send(self):
        self.serial.write(self.message_le.text().encode())

    @QtCore.pyqtSlot(bool)
    def on_toggled(self, checked):
        self.button.setText("Disconnect" if checked else "Connect")
        if checked:
            if not self.serial.isOpen():
                if not self.serial.open(QtCore.QIODevice.ReadWrite):
                    self.button.setChecked(False)
        else:
            self.serial.close()

if __name__ == '__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    w = Widget()
    w.show()
    sys.exit(app.exec_())
'''