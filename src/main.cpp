#include <Arduino.h>
#include "driver/mcpwm.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include "lwip/netif.h"
#include <esp_now.h>

extern "C" int lwip_hook_ip6_input(struct pbuf *p, struct netif *inp) __attribute__((weak));
extern "C" int lwip_hook_ip6_input(struct pbuf *p, struct netif *inp)
{
    if (ip6_addr_isany_val(inp->ip6_addr[0].u_addr.ip6))
    {
        // We don't have an LL address -> eat this packet here, so it won't get accepted on input netif
        pbuf_free(p);
        return 1;
    }
    return 0;
}

//////////// esp now start //////////////
typedef struct struct_message
{
    uint16_t throttle;
    uint16_t rudder;
    uint16_t elevator;
    uint16_t aileron;
    uint16_t mode;
    uint16_t panic;
} struct_message;

// Create a struct_message called myData
struct_message espNowData;
esp_now_peer_info_t peerInfo;

/////////// esp now end ////////////////
#define PWM_FREQUENCY_HZ 50 // 1 kHz PWM frequency
#define GPIO_ENABLE 4       // H-bridge chip enable pin
#define GPIO_PWM_HIGH 8     // High-side GPIO for motor
#define GPIO_PWM_LOW 9      // Low-side GPIO for motor

void setup_motor_pwm()
{
    // Configure MCPWM unit
    mcpwm_config_t pwm_config;
    pwm_config.frequency = PWM_FREQUENCY_HZ; // PWM frequency
    pwm_config.cmpr_a = 0.0;                 // Initial duty cycle for A
    pwm_config.cmpr_b = 0.0;                 // Initial duty cycle for B
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    // Initialize MCPWM unit 0, timer 0
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM_HIGH);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM_LOW);

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
}

void set_motor_speed(float duty_cycle, bool forward)
{
    if (forward)
    {
        // Forward: GPIO 6 active with PWM, GPIO 7 inactive (LOW)
        mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, (duty_cycle * 100));
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    }
    else
    {
        // Reverse: GPIO 7 active with PWM, GPIO 6 inactive (LOW)
        mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, (duty_cycle * 100));
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    }
}

void readMacAddress()
{
    uint8_t baseMac[6];
    esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
    if (ret == ESP_OK)
    {
        printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
               baseMac[0], baseMac[1], baseMac[2],
               baseMac[3], baseMac[4], baseMac[5]);
    }
    else
    {
        printf("Failed to read MAC address");
    }
}

int watchdog = 100;

// esp-now sent event
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    memcpy(&espNowData, incomingData, sizeof(espNowData));
    // printf("Bytes received: %d\n", len);
    // printf("Throttle: %d Rudder: %d Elevator: %d Aileron: %d Mode: %d Panic: %d\n",
    //        espNowData.throttle, espNowData.rudder, espNowData.elevator, espNowData.aileron, espNowData.mode, espNowData.panic);
    uint8_t throttle = map(espNowData.throttle, 0, 32737, 0, 255);
    uint8_t airelon = map(espNowData.aileron, 0, 32737, 0, 255);
    uint8_t rudder = map(espNowData.rudder, 0, 32737, 0, 255);
    uint8_t elevator = map(espNowData.elevator, 0, 32737, 0, 255);
    // printf("throttle: %d airelon: %d rudder: %d elevator: %d\n", throttle, airelon, rudder, elevator);
    int leftMotor = throttle + 0.25 * (airelon - 111);
    int rightMotor = throttle - 0.25 * (airelon - 111);
    printf("throttle: %d airelon: %d , elevator: %d leftMotor: %d rightMotor: %d\n",throttle, airelon, elevator, leftMotor, rightMotor);
    if (leftMotor < 0) {
        leftMotor = 0;
    }
    if (rightMotor < 0) {
        rightMotor = 0;
    }
    if (leftMotor > 255) {
        leftMotor = 255;
    }
    if (rightMotor > 255) {
        rightMotor = 255;
    }
    analogWrite(7, leftMotor);
    analogWrite(4, rightMotor);

    // if (elevator > 127) {
    //     analogWrite(9, (elevator - 127) * 2);
    //     digitalWrite(8, LOW);
    // } else {
    //     analogWrite(8, elevator * 2);
    //     digitalWrite(9, LOW);
    // }
    // set_motor_speed((float)throttle / 100, true);
    // set_motor_speed((float)rudder / 100, true);
    // set_motor_speed((float)elevator / 100, true);
    // set_motor_speed((float)airelon / 100, true);
    watchdog = 100;
}

void setup()
{
    Serial.begin(115200);
    WiFi.mode(WIFI_MODE_STA);
    printf("%s\n", WiFi.macAddress());
    pinMode(4, OUTPUT);
    pinMode(GPIO_PWM_HIGH, OUTPUT);
    pinMode(GPIO_PWM_LOW, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);

    // digitalWrite(4, HIGH); // Enable the H-bridge
    // digitalWrite(9, HIGH);
    // digitalWrite(8, LOW);
    // digitalWrite(GPIO_PWM_HIGH, LOW);
    // setup_motor_pwm();

    // Test forward motion
    // for (float duty_cycle = 0.0; duty_cycle <= 1.0; duty_cycle += 0.1)
    // {
    //     printf("Forward Speed: %.2f\n", duty_cycle);
    //     set_motor_speed(duty_cycle, true);
    //     delay(500);
    // }

    delay(1000);

    // Test reverse motion
    printf("Reversing Motor\n");
    // set_motor_speed(1.0, false);
    delay(2000);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        printf("Error initializing ESP-NOW");
        return;
    }

    // Once ESPNow is successfully Init, we will register for recv CB to
    // get recv packer info
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop()
{
    // digitalWrite(4, HIGH);
    // digitalWrite(5, HIGH);
    // readMacAddress();
    watchdog--;
    delay(10);
    if (watchdog <= 0) {
        analogWrite(7, LOW);
        analogWrite(4, LOW);
        printf("Watchdog stopped\n");
    }
    
}
