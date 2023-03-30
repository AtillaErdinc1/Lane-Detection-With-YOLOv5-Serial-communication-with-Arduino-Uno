# Lane-Detection-With-YOLOv5-Serial-communication-with-Arduino-Uno

Lane Detection Algorithm Communication with Arduino Board:

There are several different methods to send the outputs of an algorithm prepared in Python language to the Arduino board. The most common methods are:
1. Serial Communication: Python program can send data via a serial port connected to Arduino board via USB. A pre-prepared code for the serial port must be installed on the Arduino board. In this way, the Arduino board can receive and process data from the Python program.
2. Firmata Protocol: The Firmata protocol allows the Arduino board to be controlled by the Python program. In this method, the Python program sends commands to the Arduino board using the Firmata protocol, and the Arduino board executes these commands. In this method, the Firmata library must be installed on the Arduino board.
3. Wi-Fi or Bluetooth: Arduino boards can be equipped with Wi-Fi or Bluetooth modules. In this way, the Python program can send data to the Arduino board wirelessly. This method is useful in applications that require wireless data transmission.

Which method to use depends on the project's requirements and hardware capabilities. In any case, a compatible communication protocol must be used between the Python program and the Arduino board.

Şerit Tespiti Algoritmasının Arduino Kart İle Haberleşmesi:

Python diliyle hazırlanmış bir algoritmanın çıktılarını Arduino kartına göndermek için birkaç farklı yöntem vardır. En yaygın yöntemler şunlardır:
1.	Seri Haberleşme (Serial Communication): Python programı, USB üzerinden Arduino kartına bağlı olan bir seri port aracılığıyla veri gönderebilir. Ardunio kartında seri port için önceden hazırlanmış bir kod yüklü olması gerekiyor. Bu sayede Arduino kartı, Python programından gelen verileri alabilir ve işleyebilir.
2.	Firmata Protokolü: Firmata protokolü, Arduino kartının Python programı tarafından kontrol edilmesini sağlar. Bu yöntemde, Python programı Firmata protokolünü kullanarak Arduino kartına komut gönderir ve Arduino kartı bu komutları çalıştırır. Bu yöntemde, Arduino kartında Firmata kütüphanesinin yüklü olması gerekiyor.
3.	Wi-Fi veya Bluetooth: Arduino kartları, Wi-Fi veya Bluetooth modülleri ile donatılabilir. Bu sayede, Python programı Arduino kartına kablosuz olarak veri gönderebilir. Bu yöntem, kablosuz veri iletimi gerektiren uygulamalarda kullanışlıdır.

Hangi yöntemin kullanılacağı, projenin gereksinimlerine ve donanım özelliklerine bağlıdır. Her durumda, Python programı ve Arduino kartı arasında uyumlu bir iletişim protokolü kullanılmalıdır.

Kullanılacak seri haberleşme (Serial Communication) algoritması için örnek:
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

Arduino kart üzerinde buzzer kontrolü yapabilmek için, öncelikle buzzerın Arduino'ya bağlı olduğu pin numarasını bilmemiz gerekiyor. Bu örnekte buzzer, 8. pin (Arduino Uno'da) üzerine bağlı olduğunu varsayalım.
Ardından, Pyserial kütüphanesi ile Python programından gönderilen veriyi Arduino kartında okuyarak, 1 değeri geldiğinde buzzerın ötmesi, 0 değeri geldiğinde ise susması sağlanabilir. 

In order to control the buzzer on the Arduino board, we first need to know the pin number of the buzzer connected to the Arduino. In this example, let's say the buzzer is connected to pin 8 (on Arduino Uno).
Then, by reading the data sent from the Python program with the Pyserial library on the Arduino board, the buzzer can beep when the value is 1, and silence when the value is 0.

int buzzer_pin = 8; // Buzzer pin numarası
char data; // Seri porttan okunan veri

void setup() {
  pinMode(buzzer_pin, OUTPUT);
  Serial.begin(9600); // Seri haberleşme hızı
}

void loop() {
  if (Serial.available() > 0) {
    data = Serial.read(); // Seri porttan gelen veriyi oku
    if (data == '1') {
      digitalWrite(buzzer_pin, HIGH); // Buzzer ötür
    } else {
      digitalWrite(buzzer_pin, LOW); // Buzzerı kapat
    }
  }
}
