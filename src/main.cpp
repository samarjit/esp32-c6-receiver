#include <Arduino.h>
#include "driver/mcpwm.h"

#define PWM_FREQUENCY_HZ 50  // 1 kHz PWM frequency
#define GPIO_ENABLE 20         // H-bridge chip enable pin
#define GPIO_PWM_HIGH RGB_BUILTIN        // High-side GPIO for motor
#define GPIO_PWM_LOW 9         // Low-side GPIO for motor

void setup_motor_pwm() {
    // Configure MCPWM unit
    mcpwm_config_t pwm_config;
    pwm_config.frequency = PWM_FREQUENCY_HZ;  // PWM frequency
    pwm_config.cmpr_a = 0.0;                  // Initial duty cycle for A
    pwm_config.cmpr_b = 0.0;                  // Initial duty cycle for B
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    // Initialize MCPWM unit 0, timer 0
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM_HIGH);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM_LOW);

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
}

void set_motor_speed(float duty_cycle, bool forward) {
    if (forward) {
        // Forward: GPIO 6 active with PWM, GPIO 7 inactive (LOW)
        mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, fabs(duty_cycle * 100));
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    } else {
        // Reverse: GPIO 7 active with PWM, GPIO 6 inactive (LOW)
        mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, fabs(duty_cycle * 100));
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(GPIO_ENABLE, OUTPUT);
    digitalWrite(GPIO_ENABLE, HIGH);  // Enable the H-bridge
    pinMode(GPIO_PWM_HIGH, OUTPUT);
    pinMode(GPIO_PWM_LOW, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);
 digitalWrite(6, HIGH);
  digitalWrite(7, HIGH);
    digitalWrite(GPIO_PWM_HIGH, LOW);
    setup_motor_pwm();

    // Test forward motion
    for (float duty_cycle = 0.0; duty_cycle <= 1.0; duty_cycle += 0.1) {
        printf("Forward Speed: %.2f\n", duty_cycle);
        set_motor_speed(duty_cycle, true);
        delay(500);
    }

    delay(1000);

    // Test reverse motion
    printf("Reversing Motor\n");
    set_motor_speed(1.0, false);
    delay(2000);
}

void loop() {
    for (float duty_cycle = 0.0; duty_cycle <= 1.0; duty_cycle += 0.1) {
        printf("Forward Speed: %.2f\n", duty_cycle);
        set_motor_speed(duty_cycle, true);
        delay(500);
    }

    delay(1000);

    // Test reverse motion
    printf("Reversing Motor\n");
    for (float duty_cycle = 0.0; duty_cycle <= 1.0; duty_cycle += 0.1) {
        printf("Reversing Speed: %.2f\n", duty_cycle);
        set_motor_speed(duty_cycle, false);
        delay(500);
    }
    delay(2000);
}
