#include <Arduino.h>
#include "crsf.h"

#define RXD2 16
#define TXD2 17
#define SBUS_BUFFER_SIZE 25

const uint16_t trashhold = 30; // trashhold for sticks
uint8_t _rcs_buf[25]{};
uint16_t _raw_rc_values[RC_INPUT_MAX_CHANNELS]{};
uint16_t _raw_rc_count{};
uint64_t _last_rc_update = 0;

// ---------------------------------- Pins Settings ----------------------------------
// left stick pins
const uint8_t left_stick_left = 18;
// const uint8_t left_stick_left_pwm = 19;

const uint8_t left_stick_right = 21;
// const uint8_t left_stick_right_pwm = 22;

// right stick pins
const uint8_t right_stick_forward = 25;
const uint8_t right_stick_forward_pwm = 26;
const uint8_t right_stick_backward = 32;
const uint8_t right_stick_backward_pwm = 33;

// switch E pins
const uint8_t switch_e_down = NULL;
const uint8_t switch_e_up = 4;

// switch F pins
const uint8_t switch_f_down = NULL;
const uint8_t switch_f_up = 19;

// switch B pins
const uint8_t switch_b_forward = NULL;
const uint8_t switch_b_backward = 13;

// switch C pins
const uint8_t switch_c_forward = NULL;
const uint8_t switch_c_backward = 23;

// button D pin
const uint8_t button_d = 27;

// button A pin
const uint8_t button_a = 22;

// last pin on
uint8_t last_pin_on = 0;

// pwm settings
const uint8_t left_stick_left_channel = 0;
const uint8_t left_stick_right_channel = 1;
const uint8_t right_stick_forward_channel = 2;
const uint8_t right_stick_backward_channel = 3;

// ---------------------------------- Functions ----------------------------------

void secureDigitalWrite(uint8_t pin, bool value)
{
  if (pin != NULL)
  {
    digitalWrite(pin, value);
    if (value && last_pin_on != pin)
    {
      // Serial.println("pin " + String(pin) + " set to HIGH");
      last_pin_on = pin;
    }
  }
}

void disable_speed()
{
  secureDigitalWrite(switch_e_up, LOW);
  secureDigitalWrite(switch_e_down, LOW);
  secureDigitalWrite(switch_b_forward, LOW);
  secureDigitalWrite(switch_b_backward, LOW);
}

void handle_change(uint16_t value, uint8_t high, uint8_t low)
{
  if (value > 1650)
  {
    secureDigitalWrite(high, HIGH);
    secureDigitalWrite(low, LOW);
  }
  else if (value < 1350)
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

bool handle_change_with_pwm(uint16_t value, uint8_t high, uint8_t low, uint8_t pwm1, uint8_t pwm2)
{

  uint16_t value_fixed = constrain(value, 1000, 2000);

  bool stick_active = true;
  if (value > 1650)
  {
    ledcWrite(pwm1, map(value_fixed, 1650, 2000, 0, 255));
    secureDigitalWrite(high, HIGH);
    secureDigitalWrite(low, LOW);
  }
  else if (value < 1350)
  {
    ledcWrite(pwm2, map(value_fixed, 1350, 1000, 0, 255));
    secureDigitalWrite(low, HIGH);
    secureDigitalWrite(high, LOW);
  }
  else
  {
    secureDigitalWrite(high, LOW);
    secureDigitalWrite(low, LOW);
    ledcWrite(pwm1, 0);
    ledcWrite(pwm2, 0);

    stick_active = false;
  }

  return stick_active;
}

void handle_acceleration(uint16_t value, uint16_t b_switch, uint16_t e_switch)
{
  bool stick_active = handle_change_with_pwm(value, right_stick_forward, right_stick_backward, right_stick_forward_channel, right_stick_backward_channel);

  if (stick_active)
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

  // pwm settings
  ledcSetup(left_stick_left_channel, 5000, 8);
  ledcSetup(left_stick_right_channel, 5000, 8);
  ledcSetup(right_stick_backward_channel, 5000, 8);
  ledcSetup(right_stick_forward_channel, 5000, 8);

  // ledcAttachPin(left_stick_left_pwm, left_stick_left_channel);
  // ledcAttachPin(left_stick_right_pwm, left_stick_right_channel);
  ledcAttachPin(right_stick_backward_pwm, right_stick_backward_channel);
  ledcAttachPin(right_stick_forward_pwm, right_stick_forward_channel);

  pinMode(left_stick_left, OUTPUT);
  digitalWrite(left_stick_left, LOW);
  pinMode(left_stick_right, OUTPUT);
  digitalWrite(left_stick_right, LOW);

  pinMode(right_stick_forward, OUTPUT);
  digitalWrite(right_stick_forward, LOW);
  pinMode(right_stick_backward, OUTPUT);
  digitalWrite(right_stick_backward, LOW);

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

    secureDigitalWrite(left_stick_left, LOW);
    secureDigitalWrite(left_stick_right, LOW);

    secureDigitalWrite(right_stick_forward, LOW);
    secureDigitalWrite(right_stick_backward, LOW);

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
      // handle_change(_raw_rc_values[0], right_stick_left, right_stick_right); // right stick left/right (ignored)

      // Serial.print("  Ch 2: ");
      // Serial.print(_raw_rc_values[1]);
      handle_acceleration(_raw_rc_values[1], _raw_rc_values[5], _raw_rc_values[4]); // right stick up/down (ignored)

      // Serial.print("  Ch 3: ");
      // Serial.print(_raw_rc_values[2]);
      // left stick up/down  (ignored)

      // Serial.print("  Ch 4: ");
      // Serial.print(_raw_rc_values[3]);
      handle_change(_raw_rc_values[3], left_stick_left, left_stick_right); // left stick left/right

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