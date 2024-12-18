#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "esp_random.h"
#include "esp_mac.h"

#define BLINK_GPIO CONFIG_BLINK_GPIO
#define BLINK_PERIOD CONFIG_BLINK_PERIOD
#define BUTTON_GPIO_LEFT CONFIG_BUTTON_GPIO_LEFT
#define BUTTON_GPIO_RIGHT CONFIG_BUTTON_GPIO_RIGHT

#ifdef CONFIG_BLINK_LED_STRIP

static led_strip_handle_t led_strip;

static void configure_led(void)
{
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 25,
    };
#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
#elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "unsupported LED strip backend"
#endif
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}

#elif CONFIG_BLINK_LED_GPIO

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

#else
#error "unsupported LED type"
#endif

typedef struct {
    uint32_t gpio_num;
    bool winning_condition;
} game_status_t;

bool game_running = false;
bool winning_condition = false;
uint8_t max_wait_time_in_seconds = 5;
volatile bool button_pressed = false;
game_status_t game_status = {0, false};

uint8_t leftarrow[5][5] = {
    {0, 0, 1, 0, 0},
    {0, 1, 0, 0, 1},
    {1, 1, 1, 1, 1},
    {0, 1, 0, 0, 1},
    {0, 0, 1, 0, 0}
};

uint8_t rightarrow[5][5] = {
    {0, 0, 1, 0, 0},
    {1, 0, 0, 1, 0},
    {1, 1, 1, 1, 1},
    {1, 0, 0, 1, 0},
    {0, 0, 1, 0, 0}
};

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    if (!button_pressed) {
        game_status.gpio_num = gpio_num;
        game_status.winning_condition = winning_condition;
        button_pressed = true;
    }
}

static void configure_buttons()
{
    gpio_config_t gpioConfigIn = {
            .pin_bit_mask = (1 << BUTTON_GPIO_LEFT) | (1 << BUTTON_GPIO_RIGHT),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&gpioConfigIn);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO_LEFT, gpio_isr_handler, (void*) BUTTON_GPIO_LEFT);
    gpio_isr_handler_add(BUTTON_GPIO_RIGHT, gpio_isr_handler, (void*) BUTTON_GPIO_RIGHT);

    ESP_LOGD("CONFIGURATION", "Buttons configured"); 
}

uint8_t get_pixel_index(uint8_t row, uint8_t col) {
    return row * 5 + col;
}

void draw_picture(uint8_t picture[5][5], uint8_t r, uint8_t g, uint8_t b){
    led_strip_clear(led_strip);

    uint8_t current_patterns[5][5];
    memcpy(current_patterns, picture, sizeof(uint8_t) * 5 * 5);

    for (uint8_t row = 0; row < 5; row++) {
        for (uint8_t col = 0; col < 5; col++) {
            if (current_patterns[row][col] == 1) {
                uint8_t pixel_index = get_pixel_index(row, col);
                led_strip_set_pixel(led_strip, pixel_index, r, g, b);
            }
        }
    }

    led_strip_refresh(led_strip);
}

void fill_led_strip(uint8_t r, uint8_t g, uint8_t b){
    led_strip_clear(led_strip);

    for (uint8_t i = 0; i < 25; i++) {
        led_strip_set_pixel(led_strip, i, r, g, b);
    }

    led_strip_refresh(led_strip);
}

bool both_players_ready() {
    uint32_t left = gpio_get_level(BUTTON_GPIO_LEFT);
    uint32_t right = gpio_get_level(BUTTON_GPIO_RIGHT);

    if (left == 0 && right == 0) {
        ESP_LOGI("Game", "Both players ready");
        return true;
    }
    return false;
}

void wait_both_players_release_button() {
    uint32_t left = gpio_get_level(BUTTON_GPIO_LEFT);
    uint32_t right = gpio_get_level(BUTTON_GPIO_RIGHT);

    while (left == 0 || right == 0) {
        if (left == 0 && right == 0) {
            ESP_LOGD("Game", "Waiting for both players to release button");
        } else if (left != 0 && right == 0) {
            ESP_LOGD("Game", "Waiting for right player to release button");
        } else if (left == 0 && right != 0) {
            ESP_LOGD("Game", "Waiting for left player to release button");
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
        left = gpio_get_level(BUTTON_GPIO_LEFT);
        right = gpio_get_level(BUTTON_GPIO_RIGHT);
    }
}

void run_game() {
    ESP_LOGI("Game", "Game started");
    winning_condition = false;
    button_pressed = false;

    fill_led_strip(50, 0, 0);

    uint8_t time = (esp_random() % max_wait_time_in_seconds);

    game_running = true;
    vTaskDelay(time * 1000 / portTICK_PERIOD_MS);

    // Only create winning condition if no player, pressed to early
    if (!button_pressed) {
        winning_condition = true;
        fill_led_strip(0, 50, 0);
    }
}

void app_main(void)
{
    configure_led();
    configure_buttons();

    ESP_LOGI("Debug", "Everything configured, program start...");

    while (true) {
        if (game_running) {
            if (button_pressed){
                if (game_status.winning_condition) {
                    // Game is running and timer has finished
                    if (game_status.gpio_num == BUTTON_GPIO_LEFT) {
                        draw_picture(leftarrow, 0, 50, 0);
                    } else if (game_status.gpio_num == BUTTON_GPIO_RIGHT) {
                        draw_picture(rightarrow, 0, 50, 0);
                    }
                } else {
                    // A player pressed the button too early
                    if (game_status.gpio_num == BUTTON_GPIO_LEFT) {
                        draw_picture(leftarrow, 50, 0, 0);
                    } else if (game_status.gpio_num == BUTTON_GPIO_RIGHT) {
                        draw_picture(rightarrow, 50, 0, 0);
                    }
                }
                game_running = false;
            }
        } else {
            if (both_players_ready()) {
                wait_both_players_release_button();

                run_game();
            }
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}