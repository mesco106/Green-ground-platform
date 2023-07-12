#ifndef __GPIO_H__
#define __GPIO_H__

class Gpio {
public:
  Gpio(int pin) : m_pin(pin) { }
  void setup() { pinMode(m_pin, OUTPUT); }
  void on() { digitalWrite(m_pin, HIGH); }
  void off() { digitalWrite(m_pin, LOW); }
  void blink(unsigned long t){
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= t) {
      previousMillis = currentMillis;

      if (ledState == LOW) {
        ledState = HIGH;
      } else {
        ledState = LOW;
      }
      digitalWrite(m_pin, ledState);
      }
  }

private:
  int m_pin;
  int ledState = LOW;
  unsigned long previousMillis = 0;
};

#endif