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

#include <app_priv.h>

// Continuous servo control parameters
#define SERVO_STOP_PULSEWIDTH_US 1500  // Pulse width to stop the servo
#define SERVO_MAX_PULSEWIDTH_US 2000  // Maximum pulse width for full speed CW
#define SERVO_MIN_PULSEWIDTH_US 1000  // Minimum pulse width for full speed CCW

#define SERVO_PULSE_GPIO             0        // GPIO connects to the PWM signal line
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms (50Hz)

using namespace chip::app::Clusters;
using namespace esp_matter;

static const char *TAG = "app_driver";
extern uint16_t light_endpoint_id;

mcpwm_timer_handle_t timer = NULL;
mcpwm_oper_handle_t oper = NULL;
mcpwm_cmpr_handle_t comparator = NULL;
mcpwm_gen_handle_t generator = NULL;


static esp_err_t app_driver_motor_control(esp_matter_attr_val_t *val)
{
    esp_err_t err = ESP_OK;
    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    if (val->val.b) {
        ESP_LOGI(TAG, "Opening: Setting mottor to maximum speed CCW");
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, SERVO_MIN_PULSEWIDTH_US));
        vTaskDelay(pdMS_TO_TICKS(3000));  // Run for 3 seconds
    } else {
        ESP_LOGI(TAG, "Closing: Setting mottor to maximum speed CW");
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, SERVO_MAX_PULSEWIDTH_US));
        vTaskDelay(pdMS_TO_TICKS(3000));  // Run for 3 seconds
    }

    ESP_LOGI(TAG, "Motor: Stopping");
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, SERVO_STOP_PULSEWIDTH_US));
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, "Stopping and disabling timer for motor");
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_STOP_EMPTY));
    ESP_ERROR_CHECK(mcpwm_timer_disable(timer));
    return err;
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

int app_driver_motor_init()
{
    /* Initialize motor */
    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    mcpwm_comparator_config_t comparator_config;
    comparator_config.flags.update_cmp_on_tez = true;
    comparator_config.intr_priority = 0;

    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = SERVO_PULSE_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    // Set the initial compare value to stop the servo
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, SERVO_STOP_PULSEWIDTH_US));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // Go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // Go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Motor driver initialized");

    return 0;
}

