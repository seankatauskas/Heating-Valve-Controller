/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_log.h>
#include <stdlib.h>
#include <string.h>

#include <esp_matter.h>
#include "bsp/esp-bsp.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"
#include "ina219.h"

#include <app_priv.h>

// Continuous servo control parameters
#define SERVO_STOP_PULSEWIDTH_US 1500  // Pulse width to stop the servo
#define SERVO_MAX_PULSEWIDTH_US 2000  // Maximum pulse width for full speed CW
#define SERVO_MIN_PULSEWIDTH_US 1000  // Minimum pulse width for full speed CCW

#define SERVO_PULSE_GPIO             0        // GPIO connects to the PWM signal line
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms (50Hz)

// Current sensor control parameters
#define I2C_PORT I2C_NUM_0
#define I2C_ADDR 0x40

#define EXAMPLE_I2C_MASTER_SDA GPIO_NUM_19
#define EXAMPLE_I2C_MASTER_SCL GPIO_NUM_20
#define EXAMPLE_SHUNT_RESISTOR_MILLI_OHM 100

using namespace chip::app::Clusters;
using namespace esp_matter;

static const char *TAG = "app_driver";
extern uint16_t light_endpoint_id;

mcpwm_timer_handle_t timer = NULL;
mcpwm_oper_handle_t oper = NULL;
mcpwm_cmpr_handle_t comparator = NULL;
mcpwm_gen_handle_t generator = NULL;

ina219_t g_ina219_dev;

void detect_motor_resistance();

static esp_err_t app_driver_motor_control(esp_matter_attr_val_t *val)
{
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    if (val->val.b) {
        ESP_LOGI(TAG, "Opening: Setting mottor to maximum speed CCW");
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, SERVO_MIN_PULSEWIDTH_US));
    } else {
        ESP_LOGI(TAG, "Closing: Setting mottor to maximum speed CW");
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, SERVO_MAX_PULSEWIDTH_US));
    }
    
    detect_motor_resistance();
    ESP_LOGI(TAG, "Stopping and disabling timer for motor");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_STOP_EMPTY));
    ESP_ERROR_CHECK(mcpwm_timer_disable(timer));

    return ESP_OK;
}

esp_err_t app_driver_sensor_init()
{
    ESP_ERROR_CHECK(i2cdev_init());
    memset(&g_ina219_dev, 0, sizeof(ina219_t));
    ESP_ERROR_CHECK(ina219_init_desc(&g_ina219_dev, I2C_ADDR, I2C_PORT, EXAMPLE_I2C_MASTER_SDA, EXAMPLE_I2C_MASTER_SCL));
    ESP_LOGI(TAG, "Initializing INA219");
    ESP_ERROR_CHECK(ina219_init(&g_ina219_dev));

    ESP_LOGI(TAG, "Configuring INA219");
    ESP_ERROR_CHECK(ina219_configure(&g_ina219_dev, INA219_BUS_RANGE_16V, INA219_GAIN_0_125,
            INA219_RES_12BIT_1S, INA219_RES_12BIT_1S, INA219_MODE_CONT_SHUNT_BUS));
    ESP_LOGI(TAG, "Calibrating INA219");
    ESP_ERROR_CHECK(ina219_calibrate(&g_ina219_dev, (float)EXAMPLE_SHUNT_RESISTOR_MILLI_OHM / 1000.0f));

    return ESP_OK;
}

void detect_motor_resistance()
{
    float bus_voltage, shunt_voltage, current, power;
    float previous_current = 0.0;
    float rate_of_change;
    const int time_interval_ms = 25;
    const float POWER_THRESHOLD = 1400.0;  // in mW
    const float CURRENT_RATE_THRESHOLD = 1.5;  // in mA/ms
    const int GRACE_PERIOD_MS = 500;
    const int MAX_RUN_TIME_MS = 15000;  // 15 seconds in milliseconds

    TickType_t start_time = xTaskGetTickCount();

    while (1) {
        ESP_ERROR_CHECK(ina219_get_bus_voltage(&g_ina219_dev, &bus_voltage));
        ESP_ERROR_CHECK(ina219_get_shunt_voltage(&g_ina219_dev, &shunt_voltage));
        ESP_ERROR_CHECK(ina219_get_current(&g_ina219_dev, &current));
        ESP_ERROR_CHECK(ina219_get_power(&g_ina219_dev, &power));

        // Convert current and power to mA and mW for logging
        current *= 1000;  // Convert to mA
        power *= 1000;    // Convert to mW
        rate_of_change = (current - previous_current) / time_interval_ms;  // in mA/ms
        TickType_t elapsed_time = xTaskGetTickCount() - start_time;

        if (elapsed_time > pdMS_TO_TICKS(GRACE_PERIOD_MS)) {
            if ((power >= POWER_THRESHOLD && rate_of_change >= CURRENT_RATE_THRESHOLD) ||
                (elapsed_time >= pdMS_TO_TICKS(MAX_RUN_TIME_MS))) {
                ESP_LOGW(TAG, "Thresholds exceeded! Stopping motor...");
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, SERVO_STOP_PULSEWIDTH_US));  // Stop motor by setting pulse width to stop
                return;
            }
        }

        previous_current = current;  // Store current in mA
        vTaskDelay(pdMS_TO_TICKS(time_interval_ms));
    }
}


esp_err_t led_attribute_set(led_indicator_handle_t handle, int brightness, int saturation, int hue)
{
    esp_err_t err = ESP_OK;
    err |= led_indicator_set_brightness(handle, brightness);
    led_indicator_ihsv_t hsv;
    hsv.value = led_indicator_get_hsv(handle);
    hsv.s = saturation;
    hsv.h = hue;
    err |= led_indicator_set_hsv(handle, hsv.value);

    return err;
}

static void app_driver_button_toggle_cb(void *arg, void *data)
{
    ESP_LOGI(TAG, "Toggle button pressed");
    uint16_t endpoint_id = light_endpoint_id;
    uint32_t cluster_id = OnOff::Id;
    uint32_t attribute_id = OnOff::Attributes::OnOff::Id;

    attribute_t *attribute = attribute::get(endpoint_id, cluster_id, attribute_id);

    esp_matter_attr_val_t val = esp_matter_invalid(NULL);
    attribute::get_val(attribute, &val);
    val.val.b = !val.val.b;
    attribute::update(endpoint_id, cluster_id, attribute_id, &val);
}

esp_err_t app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
                                      uint32_t attribute_id, esp_matter_attr_val_t *val)
{
    esp_err_t err = ESP_OK;
    if (endpoint_id == light_endpoint_id) {
        led_indicator_handle_t handle = (led_indicator_handle_t)driver_handle;
        if (cluster_id == OnOff::Id) {
            if (attribute_id == OnOff::Attributes::OnOff::Id) {
                if (val->val.b) {
                    err |= led_attribute_set(handle, GREEN_BRIGHTNESS, GREEN_SATURATION, GREEN_HUE);
                } else {
                    err |= led_attribute_set(handle, RED_BRIGHTNESS, RED_SATURATION, RED_HUE);
                }
                err |= led_indicator_start(handle, BSP_LED_ON);
                err |= app_driver_motor_control(val);
                err |= led_indicator_start(handle, BSP_LED_OFF);
            }
        } 
    }
    return err;
}


void led_blink_setup(led_indicator_handle_t handle, int blink_count = 10, int blink_interval_ms = 500) {
    for (int i = 0; i < blink_count; i++) { 
        led_indicator_start(handle, BSP_LED_ON);
        vTaskDelay(pdMS_TO_TICKS(blink_interval_ms));
        led_indicator_start(handle, BSP_LED_OFF);
        vTaskDelay(pdMS_TO_TICKS(blink_interval_ms));
    }
}

esp_err_t app_driver_light_set_defaults(uint16_t endpoint_id)
{
    esp_err_t err = ESP_OK;
    void *priv_data = endpoint::get_priv_data(endpoint_id);
    led_indicator_handle_t handle = (led_indicator_handle_t)priv_data;
    esp_matter_attr_val_t val = esp_matter_invalid(NULL);

    /* Setting brightness, saturation, and hue for blinking setup*/
    ESP_LOGI(TAG, "Light: Blinking Light Set");
    err |= led_attribute_set(handle, DEFAULT_BRIGHTNESS, DEFAULT_SATURATION, DEFAULT_HUE);

    /* Blink for 5 seconds */
    ESP_LOGI(TAG, "Light: Blinking");
    led_blink_setup(handle);

    return err;
}

app_driver_handle_t app_driver_light_init()
{
#if CONFIG_BSP_LEDS_NUM > 0
    /* Initialize led */
    led_indicator_handle_t leds[CONFIG_BSP_LEDS_NUM];
    ESP_ERROR_CHECK(bsp_led_indicator_create(leds, NULL, CONFIG_BSP_LEDS_NUM));
    led_indicator_set_hsv(leds[0], SET_HSV(DEFAULT_HUE, DEFAULT_SATURATION, DEFAULT_BRIGHTNESS));
    
    return (app_driver_handle_t)leds[0];
#else
    return NULL;
#endif
}

app_driver_handle_t app_driver_button_init()
{
    /* Initialize button */
    button_handle_t btns[BSP_BUTTON_NUM];
    ESP_ERROR_CHECK(bsp_iot_button_create(btns, NULL, BSP_BUTTON_NUM));
    ESP_ERROR_CHECK(iot_button_register_cb(btns[0], BUTTON_PRESS_DOWN, app_driver_button_toggle_cb, NULL));
    
    return (app_driver_handle_t)btns[0];
}

esp_err_t app_driver_motor_init()
{
    ESP_LOGI(TAG, "Initializing motor...");
    mcpwm_timer_config_t timerConfig{
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
    };

    ESP_ERROR_CHECK(mcpwm_new_timer(&timerConfig, &timer));
    mcpwm_operator_config_t operatorConfig{.group_id = 0};
    ESP_ERROR_CHECK(mcpwm_new_operator(&operatorConfig, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    mcpwm_comparator_config_t comparatorConfig;
    comparatorConfig.flags.update_cmp_on_tez = true;
    comparatorConfig.intr_priority = 0;
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparatorConfig, &comparator));

    mcpwm_generator_config_t generatorConfig{.gen_gpio_num = SERVO_PULSE_GPIO};
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generatorConfig, &generator));

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, SERVO_STOP_PULSEWIDTH_US));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));
    ESP_LOGI(TAG, "Motor initialized successfully");

    return ESP_OK;
}

