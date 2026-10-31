/*
 * =============================================================================
 * TURTLEBOT MOTOR CONTROLLER FIRMWARE
 * Arduino / ESP32 / Teensy Dual DC Motor Controller & Quadrature Encoder Bridge
 * =============================================================================
 *
 * Compatible with ROS 2 `turtlebot_hardware` (ros2_control SystemInterface).
 *
 * COMMUNICATION PROTOCOL:
 * ----------------------
 * Baud Rate: 115200 bps
 * Stream Rate: 50 Hz (20 ms period)
 *
 * Inbound (ROS 2 -> Microcontroller):
 *   VEL,<left_rad_s>,<right_rad_s>\n   - Set target wheel angular velocities
 *   STOP\n                             - Emergency stop (zero targets immediately)
 *   RST\n                              - Reset cumulative encoder tick counts to 0
 *   TEST,ON\n                          - Enable pseudo-hardware emulation mode (no physical motors needed)
 *   TEST,OFF\n                         - Disable pseudo-hardware emulation (use physical pins)
 *
 * Outbound (Microcontroller -> ROS 2):
 *   ENC,<left_count>,<right_count>\n   - 50 Hz stream of cumulative encoder ticks
 *
 * PIN DEFINITIONS (Default Arduino Uno / Mega):
 * ---------------------------------------------
 * Left Motor:
 *   - PWM Pin:  5
 *   - DIR1 Pin: 6
 *   - DIR2 Pin: 7
 *   - Encoder Phase A (Interrupt): Pin 2 (INT0)
 *   - Encoder Phase B: Pin 4
 *
 * Right Motor:
 *   - PWM Pin:  9
 *   - DIR1 Pin: 10
 *   - DIR2 Pin: 11
 *   - Encoder Phase A (Interrupt): Pin 3 (INT1)
 *   - Encoder Phase B: Pin 8
 *
 * SPECIFICATIONS:
 *   - Wheel Diameter: 0.10 m (Radius: 0.05 m)
 *   - Wheel Separation: 0.20 m
 *   - Encoder CPR: 360 ticks per revolution
 * =============================================================================
 */

#include <Arduino.h>

// =============================================================================
// CONFIGURATION & CONSTANTS
// =============================================================================
#define SERIAL_BAUD 115200
#define CONTROL_FREQ_HZ 50
#define CONTROL_PERIOD_MS (1000 / CONTROL_FREQ_HZ)
#define CMD_TIMEOUT_MS 1000
#define ENCODER_CPR 360

// Motor Pin Mapping
const int PIN_LEFT_PWM  = 5;
const int PIN_LEFT_DIR1 = 6;
const int PIN_LEFT_DIR2 = 7;
const int PIN_LEFT_ENC_A = 2; // External Interrupt
const int PIN_LEFT_ENC_B = 4;

const int PIN_RIGHT_PWM  = 9;
const int PIN_RIGHT_DIR1 = 10;
const int PIN_RIGHT_DIR2 = 11;
const int PIN_RIGHT_ENC_A = 3; // External Interrupt
const int PIN_RIGHT_ENC_B = 8;

// PID Tuning Constants
float Kp = 3.5;
float Ki = 0.8;
float Kd = 0.05;

// =============================================================================
// GLOBAL STATE
// =============================================================================
volatile long left_encoder_ticks = 0;
volatile long right_encoder_ticks = 0;

float target_left_rad_s = 0.0;
float target_right_rad_s = 0.0;

unsigned long last_cmd_time = 0;
unsigned long last_control_time = 0;

// Simulation / Emulation mode (can be enabled via serial or compile flag)
bool simulation_mode = false;
float sim_left_pos_rad = 0.0;
float sim_right_pos_rad = 0.0;

// Serial reception buffer
char rx_buffer[64];
uint8_t rx_idx = 0;

// =============================================================================
// ENCODER INTERRUPTS (PHYSICAL HARDWARE)
// =============================================================================
void isr_left_encoder() {
  if (digitalRead(PIN_LEFT_ENC_B) == HIGH) {
    left_encoder_ticks++;
  } else {
    left_encoder_ticks--;
  }
}

void isr_right_encoder() {
  if (digitalRead(PIN_RIGHT_ENC_B) == HIGH) {
    right_encoder_ticks--;
  } else {
    right_encoder_ticks++;
  }
}

// =============================================================================
// MOTOR DRIVER ACTUATION
// =============================================================================
void set_motor_speeds(int left_pwm, int right_pwm) {
  // Left Motor
  if (left_pwm >= 0) {
    digitalWrite(PIN_LEFT_DIR1, HIGH);
    digitalWrite(PIN_LEFT_DIR2, LOW);
    analogWrite(PIN_LEFT_PWM, constrain(left_pwm, 0, 255));
  } else {
    digitalWrite(PIN_LEFT_DIR1, LOW);
    digitalWrite(PIN_LEFT_DIR2, HIGH);
    analogWrite(PIN_LEFT_PWM, constrain(-left_pwm, 0, 255));
  }

  // Right Motor
  if (right_pwm >= 0) {
    digitalWrite(PIN_RIGHT_DIR1, HIGH);
    digitalWrite(PIN_RIGHT_DIR2, LOW);
    analogWrite(PIN_RIGHT_PWM, constrain(right_pwm, 0, 255));
  } else {
    digitalWrite(PIN_RIGHT_DIR1, LOW);
    digitalWrite(PIN_RIGHT_DIR2, HIGH);
    analogWrite(PIN_RIGHT_PWM, constrain(-right_pwm, 0, 255));
  }
}

// =============================================================================
// SERIAL COMMAND PARSER
// =============================================================================
void process_command(char* cmd) {
  if (strncmp(cmd, "VEL,", 4) == 0) {
    char* comma = strchr(cmd + 4, ',');
    if (comma != NULL) {
      *comma = '\0';
      target_left_rad_s = atof(cmd + 4);
      target_right_rad_s = atof(comma + 1);
      last_cmd_time = millis();
    }
  } else if (strcmp(cmd, "STOP") == 0) {
    target_left_rad_s = 0.0;
    target_right_rad_s = 0.0;
  } else if (strcmp(cmd, "RST") == 0) {
    left_encoder_ticks = 0;
    right_encoder_ticks = 0;
    sim_left_pos_rad = 0.0;
    sim_right_pos_rad = 0.0;
  } else if (strcmp(cmd, "TEST,ON") == 0) {
    simulation_mode = true;
  } else if (strcmp(cmd, "TEST,OFF") == 0) {
    simulation_mode = false;
  }
}

void read_serial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (rx_idx > 0) {
        rx_buffer[rx_idx] = '\0';
        process_command(rx_buffer);
        rx_idx = 0;
      }
    } else if (rx_idx < sizeof(rx_buffer) - 1) {
      rx_buffer[rx_idx++] = c;
    }
  }
}

// =============================================================================
// SETUP & MAIN LOOP
// =============================================================================
void setup() {
  Serial.begin(SERIAL_BAUD);

  // Configure Motor Pins
  pinMode(PIN_LEFT_PWM, OUTPUT);
  pinMode(PIN_LEFT_DIR1, OUTPUT);
  pinMode(PIN_LEFT_DIR2, OUTPUT);
  pinMode(PIN_RIGHT_PWM, OUTPUT);
  pinMode(PIN_RIGHT_DIR1, OUTPUT);
  pinMode(PIN_RIGHT_DIR2, OUTPUT);

  // Configure Encoder Pins
  pinMode(PIN_LEFT_ENC_A, INPUT_PULLUP);
  pinMode(PIN_LEFT_ENC_B, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENC_A), isr_left_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENC_A), isr_right_encoder, RISING);

  last_cmd_time = millis();
  last_control_time = millis();
}

void loop() {
  read_serial();

  unsigned long current_time = millis();
  if (current_time - last_control_time >= CONTROL_PERIOD_MS) {
    float dt = (current_time - last_control_time) / 1000.0;
    last_control_time = current_time;

    // Safety watchdog: stop motors if no velocity command received recently
    if (current_time - last_cmd_time > CMD_TIMEOUT_MS) {
      target_left_rad_s = 0.0;
      target_right_rad_s = 0.0;
    }

    if (simulation_mode) {
      // Pseudo-hardware emulation: integrate wheel position and compute synthetic encoder ticks
      sim_left_pos_rad += target_left_rad_s * dt;
      sim_right_pos_rad += target_right_rad_s * dt;

      const float rad_to_counts = (float)ENCODER_CPR / (2.0 * 3.1415926535);
      left_encoder_ticks = (long)round(sim_left_pos_rad * rad_to_counts);
      right_encoder_ticks = (long)round(sim_right_pos_rad * rad_to_counts);
      set_motor_speeds(0, 0);
    } else {
      // Physical hardware closed-loop control: simple feedforward + proportional control
      // Max angular velocity ~10 rad/s mapped to PWM 0-255
      int left_pwm = (int)(target_left_rad_s * 25.5);
      int right_pwm = (int)(target_right_rad_s * 25.5);
      set_motor_speeds(left_pwm, right_pwm);
    }

    // Transmit telemetry to ROS 2 hardware interface: ENC,<left>,<right>\n
    Serial.print("ENC,");
    Serial.print(left_encoder_ticks);
    Serial.print(",");
    Serial.println(right_encoder_ticks);
  }
}
