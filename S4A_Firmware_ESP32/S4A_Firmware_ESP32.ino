#include <Arduino.h>
#include <ESP32Servo.h>

unsigned long lastDataReceivedTime = millis();
Servo servos[3];

int analogPin[6] = {
  33,  //PinArduino = A0 -> Analog0 OK
  32,  //PinArduino = A1 -> Analog1 OK
  35,  //PinArduino = A2 -> Analog2 OK
  34,  //PinArduino = A3 -> Analog3 OK
  39,  //PinArduino = A4 -> Analog4 OK
  36   //PinArduino = A5 -> Analog5 OK
};

int digitalPin[14] = {
  26,   //PinArduino = 0  -> Non usato
  25,   //PinArduino = 1  -> Non usato
  4,   //PinArduino = 2  -> Lettura digitale OK
  13,  //PinArduino = 3  -> Lettura digitale OK
  15,  //PinArduino = 4  -> Servomotore
  18,  //PinArduino = 5  -> PWM OK
  19,  //PinArduino = 6  -> PWM OK
  21,  //PinArduino = 7  -> Servomotore
  22,  //PinArduino = 8  -> Servomotore
  23,  //PinArduino = 9  -> PWM OK
  5,   //PinArduino = 10 -> Uscita digitale OK
  12,  //PinArduino = 11 -> Uscita digitale OK
  14,  //PinArduino = 12 -> Uscita digitale OK
  27   //PinArduino = 13 -> Uscita digitale OK
};

//Pin servomotore
int servoPin[3] = {
  digitalPin[4],
  digitalPin[7],
  digitalPin[8]
};
int extractPin(int pin) {
  for (int num = 0; num < 14; num++) {
    if (pin == digitalPin[num]) {
      return num;
    }
  }
}
int servoPinIndex[3] = {
  extractPin(digitalPin[4]),
  extractPin(digitalPin[7]),
  extractPin(digitalPin[8])
};

int statePin[14];  // Array per gli stati dei pin

void setup() {
  Serial.begin(38400);
  Serial.flush();
  resetPins();
  for (int i = 0; i < 3; i++) {
    servos[i].attach(servoPin[i]);
  }
}

void loop() {
  static unsigned long timerCheckUpdate = millis();
  if (millis() - timerCheckUpdate >= 20) {
    sendUpdateServomotors();
    sendSensorValues();
    timerCheckUpdate = millis();
  }
  readSerialPort();
}

void resetPins() {
  for (int a = 5; a < 14; a++) {
    pinMode(digitalPin[a], OUTPUT);
    if (digitalPin[a] == digitalPin[4] || digitalPin[a] == digitalPin[7] || digitalPin[a] == digitalPin[8]) {
      servo(digitalPin[a], 255);
      statePin[a] = 255;
    } else if(digitalPin[a] == digitalPin[2] || digitalPin[a] == digitalPin[3]){
      pinMode(digitalPin[a], INPUT);
    } else {
      digitalWrite(digitalPin[a], LOW);
      statePin[a] = 0;
    }
  }
}

void sendSensorValues() {
  unsigned int sensorValues[6], readings[5];
  for (int b = 0; b < 6; b++) {
    for (byte c = 0; c < 5; c++) {
      readings[c] = analogRead(analogPin[b]);
    }
    insertionSort(readings, 5);
    sensorValues[b] = readings[2];
  }
  for (int d = 0; d < 6; d++) {
    ScratchBoardSensorReport(d, sensorValues[d]);
  }
  if(analogRead(digitalPin[2]) > 4000){
    ScratchBoardSensorReport(6, 255);
  } else {
    ScratchBoardSensorReport(6, 0);
  }
  if(analogRead(digitalPin[3]) > 4000){
    ScratchBoardSensorReport(7, 255);
  } else {
    ScratchBoardSensorReport(7, 0);
  }
}

void insertionSort(unsigned int* array, unsigned int n) {
  for (int i = 1; i < n; i++)
    for (int j = i; (j > 0) && (array[j] < array[j - 1]); j--)
      swap(array, j, j - 1);
}

void swap(unsigned int* array, unsigned int a, unsigned int b) {
  unsigned int temp = array[a];
  array[a] = array[b];
  array[b] = temp;
}

void ScratchBoardSensorReport(byte sensor, int value) {
  Serial.write(B10000000 | ((sensor & B1111) << 3) | ((value >> 7) & B111));
  Serial.write(value & B1111111);
}

void readSerialPort() {
  byte pin;
  int newVal;
  static byte actuatorHighByte, actuatorLowByte;
  static byte readingSM = 0;
  if (Serial.available()) {
    if (readingSM == 0) {
      actuatorHighByte = Serial.read();
      if (actuatorHighByte >= 128) readingSM = 1;
    } else if (readingSM == 1) {
      actuatorLowByte = Serial.read();
      if (actuatorLowByte < 128) readingSM = 2;
      else readingSM = 0;
    }
    if (readingSM == 2) {
      lastDataReceivedTime = millis();
      pin = ((actuatorHighByte >> 3) & 0x0F);
      newVal = ((actuatorHighByte & 0x07) << 7) | (actuatorLowByte & 0x7F);
      if (statePin[pin] != newVal) {
        statePin[pin] = newVal;
        updateActuator(pin);
      }
      readingSM = 0;
    }
  } else checkScratchDisconnection();
}

void reset() {
  resetPins();
  sendSensorValues();
  lastDataReceivedTime = millis();
}

void updateActuator(byte pinNumber) {
  int pin = (int)pinNumber;
  if (digitalPin[pin] == digitalPin[13] || digitalPin[pin] == digitalPin[12] || digitalPin[pin] == digitalPin[11] || digitalPin[pin] == digitalPin[10]) {
    digitalWrite(digitalPin[pin], statePin[pin]);
  } else if (digitalPin[pin] == digitalPin[5] || digitalPin[pin] == digitalPin[6] || digitalPin[pin] == digitalPin[9]) {
    ledcAttach(digitalPin[pin], 5000, 8);
    ledcWrite(digitalPin[pin], statePin[pin]);
  }
}

void sendUpdateServomotors() {
  for (int f = 0; f < 3; f++) {
    servo(f, statePin[servoPinIndex[f]]);
  }
}


void servo(byte pinNumber, byte angle) {
  if (angle != 255) {
    servos[pinNumber].write(angle);  // Usa la libreria ESP32Servo
  }
}

void checkScratchDisconnection() {
  if (millis() - lastDataReceivedTime > 1000) reset();
}
