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
