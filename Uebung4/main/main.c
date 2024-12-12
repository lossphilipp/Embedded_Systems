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

int8_t leftButton = 9;
int8_t rightButton = 2;
uint8_t timer_config = 1;

uint16_t max_time = 0;
uint16_t current_time = 0;

uint8_t number_patterns[20][5][5] = {
    // 1
    {
        {0, 0, 1, 0, 0},
        {0, 1, 1, 0, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 1, 0, 0},
        {0, 1, 1, 1, 0}
    },
    // 2
    {
        {0, 1, 1, 1, 0},
        {0, 0, 0, 1, 0},
        {0, 1, 1, 1, 0},
        {0, 1, 0, 0, 0},
        {0, 1, 1, 1, 0}
    },
    // 3
    {
        {0, 1, 1, 1, 0},
        {0, 0, 0, 1, 0},
        {0, 1, 1, 1, 0},
        {0, 0, 0, 1, 0},
        {0, 1, 1, 1, 0}},
    // 4
    {
        {0, 1, 0, 1, 0},
        {0, 1, 0, 1, 0},
        {0, 1, 1, 1, 0},
        {0, 0, 0, 1, 0},
        {0, 0, 0, 1, 0}
    },
    // 5
    {
        {0, 1, 1, 1, 0},
        {0, 1, 0, 0, 0},
        {0, 1, 1, 1, 0},
        {0, 0, 0, 1, 0},
        {0, 1, 1, 1, 0}
    },
    // 6
    {
        {0, 1, 1, 1, 0},
        {0, 1, 0, 0, 0},
        {0, 1, 1, 1, 0},
        {0, 1, 0, 1, 0},
        {0, 1, 1, 1, 0}
    },
    // 7
    {
        {0, 1, 1, 1, 0},
        {0, 0, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 1, 0, 0},
        {0, 1, 0, 0, 0}
    },
    // 8
    {
        {0, 1, 1, 1, 0},
        {0, 1, 0, 1, 0},
        {0, 1, 1, 1, 0},
        {0, 1, 0, 1, 0},
        {0, 1, 1, 1, 0}
    },
    // 9
    {
        {0, 1, 1, 1, 0},
        {0, 1, 0, 1, 0},
        {0, 1, 1, 1, 0},
        {0, 0, 0, 1, 0},
        {0, 1, 1, 1, 0}
    },
    // 10
    {
        {1, 0, 1, 1, 1},
        {1, 0, 1, 0, 1},
        {1, 0, 1, 0, 1},
        {1, 0, 1, 0, 1},
        {1, 0, 1, 1, 1}
    },
    // 11
    {
        {1, 0, 0, 1, 0},
        {1, 0, 1, 1, 0},
        {1, 0, 0, 1, 0},
        {1, 0, 0, 1, 0},
        {1, 0, 1, 1, 1}
    },
    // 12
    {
        {1, 0, 1, 1, 1},
        {1, 0, 0, 0, 1},
        {1, 0, 1, 1, 1},
        {1, 0, 1, 0, 0},
        {1, 0, 1, 1, 1}
    },
    // 13
    {
        {1, 0, 1, 1, 1},
        {1, 0, 0, 0, 1},
        {1, 0, 1, 1, 1},
        {1, 0, 0, 0, 1},
        {1, 0, 1, 1, 1}
    },
    // 14
    {
        {1, 0, 1, 0, 1},
        {1, 0, 1, 0, 1},
        {1, 0, 1, 1, 1},
        {1, 0, 0, 0, 1},
        {1, 0, 0, 0, 1}
    },
    // 15
    {
        {1, 0, 1, 1, 1},
        {1, 0, 1, 0, 0},
        {1, 0, 1, 1, 1},
        {1, 0, 0, 0, 1},
        {1, 0, 1, 1, 1}
    }
};



uint8_t get_pixel_index(uint8_t row, uint8_t col) {
    return row * 5 + col;
}

void draw_number(uint8_t number){
    uint8_t r = 50;
    uint8_t g = 0;
    uint8_t b = 0;

    led_strip_clear(led_strip);

    uint8_t current_patterns[5][5];
    memcpy(current_patterns, number_patterns[number - 1], sizeof(uint8_t) * 5 * 5);

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

void configure_timer(){
    ESP_LOGI("Debug", "Configuring timer...");

    bool confuration_mode = true;
    draw_number(timer_config);
    while (confuration_mode)
    {
        uint32_t left = gpio_get_level(leftButton);
        uint32_t right = gpio_get_level(rightButton);

        if (left == 0 && right == 0)
        {
            confuration_mode = false;
        }
        else
        {
            if (left == 0)
            {
                if (timer_config == 1)
                {
                    continue;
                }
                timer_config -= 1;
                draw_number(timer_config);
            }
            if (right == 0)
            {
                if (timer_config == 15)
                {
                    continue;
                }
                timer_config += 1;
                draw_number(timer_config);
            }
        }
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

void calculate_binary(int dec_number, int *binary_array, int bit_size) {
    for (int i = bit_size - 1; i >= 0; i--) {
        binary_array[i] = dec_number % 2;
        dec_number /= 2;
    }
}

void draw_binary_numbers()
{
    int minutes_pattern[4];
    int seconds_pattern[6];

    calculate_binary(current_time / 60, minutes_pattern, 4);
    calculate_binary(current_time % 60, seconds_pattern, 6);

    for (uint8_t i = 0; i < 4; i++) {
        if (minutes_pattern[i] == 1)
        {
            led_strip_set_pixel(led_strip, i, 0, 0, 30);
        }
    }

    for (uint8_t j = 0; j < 6; j++) {
        if (seconds_pattern[j] == 1)
        {
            led_strip_set_pixel(led_strip, j + 4, 30, 30, 30);
        }
    }
}

void draw_colorpattern()
{
    uint8_t r = (current_time * 50) / max_time;
    uint8_t g = 50 - r;
    uint8_t b = 0;

    for (uint8_t row = 2; row < 5; row++) {
        for (uint8_t col = 0; col < 5; col++) {
            uint8_t pixel_index = get_pixel_index(row, col);
            led_strip_set_pixel(led_strip, pixel_index, r, g, b);
        }
    }
}

void draw_timer() {
    led_strip_clear(led_strip);

    draw_binary_numbers();
    draw_colorpattern();

    led_strip_refresh(led_strip);
}

void execute_timer()
{
    ESP_LOGI("Debug", "Starting timer...");

    max_time = timer_config * 60;
    current_time = max_time;

    bool timer_running = true;
    while (timer_running)
    {
        ESP_LOGI("Verbose", "Current timer time: %d", current_time);

        draw_timer();
        current_time -= 1;
        if (current_time == 0)
        {
            timer_running = false;
        }
        

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void timer_finished()
{
    ESP_LOGI("Debug", "Timer finished!");
    led_strip_clear(led_strip);

    for (size_t i = 0; i < 5; i++)
    {
        for (uint8_t row = 0; row < 5; row++) {
            for (uint8_t col = 0; col < 5; col++) {
                uint8_t pixel_index = get_pixel_index(row, col);
                led_strip_set_pixel(led_strip, pixel_index, 0, 50, 0);
            }
        }
        led_strip_refresh(led_strip);

        vTaskDelay(500 / portTICK_PERIOD_MS);

        led_strip_clear(led_strip);
        led_strip_refresh(led_strip);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    configure_led();

    gpio_config_t gpioConfigIn = {
        .pin_bit_mask = (1 << leftButton) | (1 << rightButton),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&gpioConfigIn);

    ESP_LOGI("Debug", "Everything configured, program start...");

    configure_timer();

    execute_timer();

    timer_finished();
}