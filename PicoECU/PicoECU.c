#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "ssd1306.h"

#define I2C_PORT    i2c0
#define I2C_SDA     0
#define I2C_SCL     1

#define INJECTOR_PIN    21
#define HALL_PIN        26 
#define ADC_MAP_PIN     27 
#define ADC_TEMP_PIN    28 

//#define ADC_HALL_CHANNEL    0
#define ADC_MAP_CHANNEL     1 
#define ADC_TEMP_CHANNEL    2 

#define HALL_THRESH     2500
#define HYSTERESIS      200
#define TEMP_BETA       3950
#define TEMP_ROOM_RES   10000
#define COLD_RATIO      1.3f
#define INTAKE_THRESH   600
#define INJ_DUR_MAX     5000
#define INJ_DUR_MIN     500
#define CYCLE_DELAY     10     // 30 gives a max rpm of ~20k. we can only dream.
#define MIN_CYCLE       300

volatile uint32_t rpm = 0;
volatile uint32_t rpm_max = 0;
volatile uint16_t map = 0;
volatile float injector_us = 0;
volatile float temp_c = 25.0;
absolute_time_t last_cycle_time;
volatile bool prev_state = false;

ssd1306_t disp;

// prototypes
void prep_injector();
void injector_pulse(float pulse_us);
void display_info();
float get_temp();


// fire injector for pulse_us - PWM for 
void injector_pulse(float pulse_us) {
    gpio_put(INJECTOR_PIN, 1);
    sleep_us((int)pulse_us);
    gpio_put(INJECTOR_PIN, 0);
}

// don't worry about the math here, basically just find the voltage based on the ADC values and convert to a resistance
// resistance can then be used to either look up or calculate (Steinhart–Hart equation thingy) the resulting temp in kelvin and then to celsius
float get_temp() {
    adc_select_input(ADC_TEMP_CHANNEL);
    uint16_t temp_raw = adc_read();

    // float thermistor_resistance = (3.3f * 10000 / (temp_raw * 3.3f / 4095.0f)) - 10000;
    // resistance = (adc_voltage * fixed_resistor) / (supply_voltage - adc_voltage)
    float divider_voltage = (temp_raw / 4095.0f) * 3.3f;
    float thermistor_resistance = (divider_voltage * 10000) / (3.3f - divider_voltage);
    float temp_kelvin = 1.0f / (1.0f / 298.15f + log(thermistor_resistance / TEMP_ROOM_RES) / TEMP_BETA);
    return temp_kelvin - 273.15f;
}

// display sensor info on oled
void display_info() {
    char display_buffer[32];
    ssd1306_clear(&disp);

    snprintf(display_buffer, sizeof(display_buffer), "RPM: %4u", rpm);
    ssd1306_draw_string(&disp, 0, 0, 1, display_buffer);

    snprintf(display_buffer, sizeof(display_buffer), "MAP: %u", map);
    ssd1306_draw_string(&disp, 0, 16, 1, display_buffer);

    snprintf(display_buffer, sizeof(display_buffer), "Injector PW: %.2fus", injector_us);
    ssd1306_draw_string(&disp, 0, 28, 1, display_buffer);

    snprintf(display_buffer, sizeof(display_buffer), "Temp: %.2fC", temp_c);
    ssd1306_draw_string(&disp, 0, 40, 1, display_buffer);

    snprintf(display_buffer, sizeof(display_buffer), "Peak RPM: %5u", rpm_max);
    ssd1306_draw_string(&disp, 0, 52, 1, display_buffer);

    ssd1306_show(&disp);
}

void prep_injector() {
    adc_select_input(ADC_MAP_CHANNEL);
    map = adc_read();
    bool intake = (map < INTAKE_THRESH);

    if (intake) {
        // determine fuel amount
        float fuel_modifier = 1.0f;
        if (temp_c < 20.0f) {
            fuel_modifier = COLD_RATIO; 
        }

        float base_pulse_us = 1000 + (700 - map) * 2;   // i am using this because i have no idea what the real equation is
        injector_us = base_pulse_us * fuel_modifier;

        // set injection duration limits
        if (injector_us > INJ_DUR_MAX) {
            injector_us = INJ_DUR_MAX;
        }
        else if (injector_us < INJ_DUR_MIN) {
            injector_us = INJ_DUR_MIN;
        }

        injector_pulse(injector_us);
    } 
}

void hall_effect_sensor_callback(uint gpio, uint32_t events) {
    static absolute_time_t last_time;
    absolute_time_t now = get_absolute_time();
    uint64_t dt = absolute_time_diff_us(last_time, now);
    if (dt > MIN_CYCLE) {
        rpm = 60000000 / dt;
        if (rpm > rpm_max) rpm_max = rpm;
        last_time = now;
        prep_injector();
    }
}

int main() {
    stdio_init_all();

    adc_init();
    adc_gpio_init(ADC_MAP_PIN);
    adc_gpio_init(ADC_TEMP_PIN);

    gpio_init(HALL_PIN);
    gpio_set_dir(HALL_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(HALL_PIN, GPIO_IRQ_EDGE_FALL, true, hall_effect_sensor_callback);

    gpio_init(INJECTOR_PIN);
    gpio_set_dir(INJECTOR_PIN, GPIO_OUT);
    gpio_put(INJECTOR_PIN, 0);

    i2c_init(I2C_PORT, 400*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    disp.external_vcc = false;
    ssd1306_init(&disp, 128, 64, 0x3C, I2C_PORT);

    last_cycle_time = get_absolute_time();

    while (true) {
        display_info();
        sleep_ms(CYCLE_DELAY); 
    }

    return 0;
}
