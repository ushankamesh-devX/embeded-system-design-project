#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_client.h"
#include "driver/mcpwm.h"

/* ── Pin Definitions ─────────────────────────────────── */
#define TRIG_PIN        GPIO_NUM_5
#define ECHO_PIN        GPIO_NUM_18
#define LED_GREEN       GPIO_NUM_25
#define LED_YELLOW      GPIO_NUM_26
#define LED_RED         GPIO_NUM_27
#define BUZZER_PIN      GPIO_NUM_14
#define SERVO_PIN       GPIO_NUM_13
#define BUTTON_PIN      GPIO_NUM_4

/* ── Thresholds (cm) ─────────────────────────────────── */
#define WARN_DIST       15
#define CRIT_DIST       8

/* ── WiFi Config ─────────────────────────────────────── */
#define WIFI_SSID       "YOUR_WIFI"
#define WIFI_PASS       "YOUR_PASS"
#define API_KEY         "YOUR_API_KEY"

/* ── Global Flags ────────────────────────────────────── */
static volatile uint8_t reset_flag = 0;

/* ═══════════════════════════════════════════════════════
   INTERRUPT SERVICE ROUTINE
   ═══════════════════════════════════════════════════════ */
static void IRAM_ATTR button_isr_handler(void *arg)
{
    reset_flag = 1;
}

/* ═══════════════════════════════════════════════════════
   SENSOR ACQUISITION
   ═══════════════════════════════════════════════════════ */
float sensor_read_distance(void)
{
    uint32_t start, end;
    float    distance;

    /* send trigger pulse */
    gpio_set_level(TRIG_PIN, 0);
    esp_rom_delay_us(2);
    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_PIN, 0);

    /* wait for echo to go HIGH */
    start = esp_log_timestamp();
    while (gpio_get_level(ECHO_PIN) == 0) {
        if ((esp_log_timestamp() - start) > 30) return -1;
    }

    /* measure pulse width */
    start = esp_log_timestamp();
    while (gpio_get_level(ECHO_PIN) == 1) {
        if ((esp_log_timestamp() - start) > 30) return -1;
    }
    end = esp_log_timestamp();

    distance = ((end - start) * 0.0343f) / 2.0f;
    return distance;
}

/* ═══════════════════════════════════════════════════════
   OUTPUT CONTROL
   ═══════════════════════════════════════════════════════ */
void output_set_leds(uint8_t g, uint8_t y, uint8_t r)
{
    gpio_set_level(LED_GREEN,  g);
    gpio_set_level(LED_YELLOW, y);
    gpio_set_level(LED_RED,    r);
}

void output_set_buzzer(uint8_t state)
{
    gpio_set_level(BUZZER_PIN, state);
}

void output_set_gate(uint8_t open)
{
    /* servo: 0 deg = closed, 90 deg = open
       MCPWM duty: ~1ms = 0deg, ~2ms = 90deg at 50Hz */
    uint32_t duty = open ? 2000 : 1000;
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
}

void output_upload_cloud(float dist, const char *status)
{
    char url[200];
    snprintf(url, sizeof(url),
             "http://api.thingspeak.com/update?api_key=%s&field1=%.2f&field2=%s",
             API_KEY, dist, status);

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_perform(client);
    esp_http_client_cleanup(client);

    printf("[Cloud] Uploaded: %s | %.2f cm\n", status, dist);
}

/* ═══════════════════════════════════════════════════════
   PROCESSING LOGIC
   ═══════════════════════════════════════════════════════ */
void logic_process_level(float dist)
{
    if (dist <= CRIT_DIST) {
        printf("[STATE] CRITICAL - Gate Open\n");
        output_set_leds(0, 0, 1);
        output_set_buzzer(1);
        output_set_gate(1);
        output_upload_cloud(dist, "CRITICAL");

    } else if (dist <= WARN_DIST) {
        printf("[STATE] WARNING - Gate Closed\n");
        output_set_leds(0, 1, 0);
        output_set_buzzer(0);
        output_set_gate(0);
        output_upload_cloud(dist, "WARNING");

    } else {
        printf("[STATE] NORMAL - Gate Closed\n");
        output_set_leds(1, 0, 0);
        output_set_buzzer(0);
        output_set_gate(0);
        output_upload_cloud(dist, "NORMAL");
    }
}

/* ═══════════════════════════════════════════════════════
   FREERTOS MAIN TASK
   ═══════════════════════════════════════════════════════ */
void dam_monitor_task(void *pvParameters)
{
    float dist;

    while (1) {
        /* handle manual reset */
        if (reset_flag) {
            printf("[ISR] Manual Reset\n");
            output_set_leds(1, 0, 0);
            output_set_buzzer(0);
            output_set_gate(0);
            reset_flag = 0;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        /* read sensor */
        dist = sensor_read_distance();
        printf("[SENSOR] Distance: %.2f cm\n", dist);

        /* process */
        if (dist > 0) {
            logic_process_level(dist);
        }

        /* 2 second delay via FreeRTOS */
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/* ═══════════════════════════════════════════════════════
   APP MAIN  (entry point in ESP-IDF)
   ═══════════════════════════════════════════════════════ */
void app_main(void)
{
    /* NVS init (required for WiFi) */
    nvs_flash_init();

    /* GPIO config - outputs */
    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << LED_GREEN)  |
                        (1ULL << LED_YELLOW) |
                        (1ULL << LED_RED)    |
                        (1ULL << BUZZER_PIN) |
                        (1ULL << TRIG_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&out_cfg);

    /* GPIO config - input (echo) */
    gpio_config_t echo_cfg = {
        .pin_bit_mask = (1ULL << ECHO_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&echo_cfg);

    /* GPIO config - button with interrupt */
    gpio_config_t btn_cfg = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&btn_cfg);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, NULL);

    /* MCPWM for servo */
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_PIN);
    mcpwm_config_t pwm_cfg = {
        .frequency    = 50,
        .cmpr_a       = 0,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode    = MCPWM_DUTY_MODE_0,
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_cfg);
    output_set_gate(0);

    /* WiFi init */
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid     = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    esp_wifi_start();
    esp_wifi_connect();
    vTaskDelay(pdMS_TO_TICKS(5000)); /* wait for connection */

    printf("[SYS] System Ready\n");

    /* Create FreeRTOS task */
    xTaskCreate(dam_monitor_task, "dam_task", 4096, NULL, 5, NULL);
}