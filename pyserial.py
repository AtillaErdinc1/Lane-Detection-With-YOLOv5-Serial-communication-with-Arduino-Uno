import serial
import time

arduino_port = 'COM3'  # Arduino'nun bağlı olduğu seri portun adı
baud_rate = 9600  # Arduino'nun seri port hızı
ser = serial.Serial(arduino_port, baud_rate)
i = 0
while True:
    i += 1
    if i % 2 == 0:
        ser.write(b'0')  # Arduino'ya 0 gönder
    else:
        ser.write(b'1')  # Arduino'ya 1 gönder
    time.sleep(1)  # 1 saniye bekle
