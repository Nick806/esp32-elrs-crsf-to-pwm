#include <Arduino.h>
#include "crsf.h"

#define RXD2 16
#define TXD2 17
#define SBUS_BUFFER_SIZE 25
uint8_t _rcs_buf[25]{};
uint16_t _raw_rc_values[RC_INPUT_MAX_CHANNELS]{};
uint16_t _raw_rc_count{};
uint64_t _last_rc_update = 0;

// ---------------------------------- Pins Settings ----------------------------------
// left stick pins
const uint8_t left_stick_up = 18;
const uint8_t left_stick_down = 19;

// right stick pins
const uint8_t right_stick_left = 21;
const uint8_t right_stick_right = 22;

// switch E pins
const uint8_t switch_e_down = NULL;
const uint8_t switch_e_up = 25;

// switch F pins
const uint8_t switch_f_down = NULL;
const uint8_t switch_f_up = 23;

// switch B pins
const uint8_t switch_b_forward = 26;
const uint8_t switch_b_backward = 27;

// switch C pins
const uint8_t switch_c_forward = 4;
const uint8_t switch_c_backward = 32;

// button D pin
const uint8_t button_d = 33;

// button A pin
const uint8_t button_a = 13;

// last pin on
uint8_t last_pin_on = 0;

// ---------------------------------- Functions ----------------------------------

void secureDigitalWrite(uint8_t pin, bool value)
{
  if (pin != NULL)
  {
    digitalWrite(pin, value);
    if (value && last_pin_on != pin)
    {
      Serial.println("pin " + String(pin) + " set to HIGH");
      last_pin_on = pin;
    }
  }
}

void disable_speed()
{
  secureDigitalWrite(switch_e_up, LOW);
  secureDigitalWrite(switch_b_forward, LOW);
  secureDigitalWrite(switch_b_backward, LOW);
}

void handle_change(uint16_t value, uint8_t high, uint8_t low)
{
  if (value > 1700)
  {
    secureDigitalWrite(high, HIGH);
    secureDigitalWrite(low, LOW);
  }
  else if (value < 1300)
  {
    secureDigitalWrite(low, HIGH);
    secureDigitalWrite(high, LOW);
  }
  else
  {
    secureDigitalWrite(high, LOW);
    secureDigitalWrite(low, LOW);
  }
}

void handle_acceleration(uint16_t value, uint16_t b_switch, uint16_t e_switch)
{
  handle_change(value, left_stick_up, left_stick_down);

  if (value > 1700 || value < 1300)
  {
    handle_change(b_switch, switch_b_backward, switch_b_forward);
    handle_change(e_switch, switch_e_up, switch_e_down);
  }
  else
  {
    disable_speed();
  }
}

void setup()
{
  // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  Serial.begin(115200);
  Serial2.begin(420000, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Serial Txd is on pin: " + String(TX));
  Serial.println("Serial Rxd is on pin: " + String(RX));

  pinMode(left_stick_up, OUTPUT);
  digitalWrite(left_stick_up, LOW);
  pinMode(left_stick_down, OUTPUT);
  digitalWrite(left_stick_down, LOW);

  pinMode(right_stick_left, OUTPUT);
  digitalWrite(right_stick_left, LOW);
  pinMode(right_stick_right, OUTPUT);
  digitalWrite(right_stick_right, LOW);

  pinMode(switch_e_down, OUTPUT);
  digitalWrite(switch_e_down, LOW);
  pinMode(switch_e_up, OUTPUT);
  digitalWrite(switch_e_up, LOW);

  pinMode(switch_f_down, OUTPUT);
  digitalWrite(switch_f_down, LOW);
  pinMode(switch_f_up, OUTPUT);
  digitalWrite(switch_f_up, LOW);

  pinMode(switch_b_forward, OUTPUT);
  digitalWrite(switch_b_forward, LOW);
  pinMode(switch_b_backward, OUTPUT);
  digitalWrite(switch_b_backward, LOW);

  pinMode(switch_c_forward, OUTPUT);
  digitalWrite(switch_c_forward, LOW);
  pinMode(switch_c_backward, OUTPUT);
  digitalWrite(switch_c_backward, LOW);

  pinMode(button_d, OUTPUT);
  digitalWrite(button_d, LOW);

  pinMode(button_a, OUTPUT);
  digitalWrite(button_a, LOW);
}

void loop()
{
  if (millis() - _last_rc_update > 300)
  {
    Serial.println("No RC signal detected");

    secureDigitalWrite(left_stick_up, LOW);
    secureDigitalWrite(left_stick_down, LOW);

    secureDigitalWrite(right_stick_left, LOW);
    secureDigitalWrite(right_stick_right, LOW);

    secureDigitalWrite(switch_e_down, LOW);
    secureDigitalWrite(switch_e_up, LOW);

    secureDigitalWrite(switch_f_down, LOW);
    secureDigitalWrite(switch_f_up, LOW);

    secureDigitalWrite(switch_b_forward, LOW);
    secureDigitalWrite(switch_b_backward, LOW);

    secureDigitalWrite(switch_c_forward, LOW);
    secureDigitalWrite(switch_c_backward, LOW);

    secureDigitalWrite(button_d, LOW);

    secureDigitalWrite(button_a, LOW);
  }

  while (Serial2.available())
  {
    size_t numBytesRead = Serial2.readBytes(_rcs_buf, SBUS_BUFFER_SIZE);
    if (numBytesRead > 0)
    {
      _last_rc_update = millis();
      crsf_parse(&_rcs_buf[0], SBUS_BUFFER_SIZE, &_raw_rc_values[0], &_raw_rc_count, RC_INPUT_MAX_CHANNELS);
      if (_raw_rc_values[0] == 0)
      {
        return;
      }
      // Serial.print("Ch 1: ");
      // Serial.print(_raw_rc_values[0]);
      handle_change(_raw_rc_values[0], right_stick_left, right_stick_right); // right stick left/right

      // Serial.print("  Ch 2: ");
      // Serial.print(_raw_rc_values[1]);
      // right stick up/down (ignored)

      // Serial.print("  Ch 3: ");
      // Serial.print(_raw_rc_values[2]);
      handle_acceleration(_raw_rc_values[2], _raw_rc_values[5], _raw_rc_values[4]); // left stick up/down

      // Serial.print("  Ch 4: ");
      // Serial.print(_raw_rc_values[3]);
      // left stick left/right (ignored)

      // Serial.print("  Ch 5: ");
      // Serial.print(_raw_rc_values[4]);
      // handle_change(_raw_rc_values[4], switch_e_up, switch_e_down); // switch E up/down

      // Serial.print("  Ch 6: ");
      // Serial.print(_raw_rc_values[5]);
      // handle_change(_raw_rc_values[5], switch_b_backward, switch_b_forward); // switch B forward/backward

      // Serial.print("  Ch 7: ");
      // Serial.print(_raw_rc_values[6]);
      handle_change(_raw_rc_values[6], switch_f_up, switch_f_down); // switch F up/down (ignored)

      // Serial.print("  Ch 8: ");
      // Serial.print(_raw_rc_values[7]);
      handle_change(_raw_rc_values[7], switch_c_backward, switch_c_forward); // switch C forward/backward

      // Serial.print("  Ch 9: ");
      // Serial.print(_raw_rc_values[8]);
      handle_change(_raw_rc_values[8], button_d, NULL); // button D

      // Serial.print("  Ch 10: ");
      // Serial.print(_raw_rc_values[9]);
      handle_change(_raw_rc_values[9], button_a, NULL); // button A
      // Serial.println("");
    }
  }
}