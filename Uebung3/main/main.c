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

uint8_t current_max_number_index = 0;
uint8_t max_numbers[] = {6, 10, 12, 20};

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
    },
    // 16
    {
        {1, 0, 1, 1, 1},
        {1, 0, 1, 0, 0},
        {1, 0, 1, 1, 1},
        {1, 0, 1, 0, 1},
        {1, 0, 1, 1, 1}
    },
    // 17
    {
        {1, 0, 1, 1, 1},
        {1, 0, 0, 0, 1},
        {1, 0, 0, 1, 0},
        {1, 0, 0, 1, 0},
        {1, 0, 1, 0, 0}
    },
    // 18
    {
        {1, 0, 1, 1, 1},
        {1, 0, 1, 0, 1},
        {1, 0, 1, 1, 1},
        {1, 0, 1, 0, 1},
        {1, 0, 1, 1, 1}
    },
    // 19
    {
        {1, 0, 1, 1, 1},
        {1, 0, 1, 0, 1},
        {1, 0, 1, 1, 1},
        {1, 0, 0, 0, 1},
        {1, 0, 1, 1, 1}
    },
    // 20
    {
        {1, 1, 1, 1, 1},
        {0, 1, 1, 0, 1},
        {1, 1, 1, 0, 1},
        {1, 0, 1, 0, 1},
        {1, 1, 1, 1, 1}
    }
};

uint8_t rolling_pattern[8][5][5] = {
    {
        {0, 0, 1, 0, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0}
    },
    {
        {0, 0, 0, 0, 1},
        {0, 0, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0}
    },
    {
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 1, 1, 1},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0}
    },
    {
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 1, 0},
        {0, 0, 0, 0, 1}
    },
    {
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 1, 0, 0}
    },
    {
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 1, 0, 0},
        {0, 1, 0, 0, 0},
        {1, 0, 0, 0, 0}
    },
    {
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {1, 1, 1, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0}
    },
    {
        {1, 0, 0, 0, 0},
        {0, 1, 0, 0, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0}
    }
};

uint8_t get_pixel_index(uint8_t row, uint8_t col) {
    return row * 5 + col;
}

void draw_roll_animation(){
    for (size_t i = 0; i < 3; i++)
    {
        for (uint8_t i = 0; i < 8; i++) {
            led_strip_clear(led_strip);

            uint8_t current_patterns[5][5];
            memcpy(current_patterns, rolling_pattern[i], sizeof(uint8_t) * 5 * 5);

            for (uint8_t row = 0; row < 5; row++) {
                for (uint8_t col = 0; col < 5; col++) {
                    if (current_patterns[row][col] == 1) {
                        uint8_t pixel_index = get_pixel_index(row, col);
                        led_strip_set_pixel(led_strip, pixel_index, 20, 20, 20);
                    }
                }
            }

            led_strip_refresh(led_strip);
            vTaskDelay(35 / portTICK_PERIOD_MS);
        }
    }
}

void draw_number(uint8_t number, bool is_max_number){
    uint8_t r = is_max_number ? 50 : 0;  // Red for max numbers
    uint8_t g = 0;
    uint8_t b = is_max_number ? 0 : 50; // Blue for other numbers

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

void rotate_max_number() {
    ESP_LOGI("Button", "Left Pressed");

    current_max_number_index = (current_max_number_index + 1) % 4;

    draw_number(max_numbers[current_max_number_index], true);
}

void create_random_number() {
    ESP_LOGI("Button", "Right Pressed");

    uint8_t number = (esp_random() % max_numbers[current_max_number_index]) + 1;

    draw_roll_animation();
    draw_number(number, false);
}

// *************
// Second week: test random numbers
#define SQUARE(x) ((x) * (x))
#define RNG_BASE 0x60026000
#define RNG_DATA_REG_OFFS 0xB0

volatile uint32_t* pRngDataReg = (volatile uint32_t*) (RNG_BASE | RNG_DATA_REG_OFFS);

inline uint32_t nextRand() {
    return *pRngDataReg;
}

inline uint8_t create_random_number_manual() {
    return (*pRngDataReg % max_numbers[current_max_number_index]) + 1;
}

bool equalDistChi2(const uint32_t n[], uint32_t m, uint32_t n0, uint32_t chi2) {
    uint32_t squaresum = 0;
    for (uint32_t i = 0; i < m; i++) {
        squaresum += SQUARE(n[i] - n0);
    }
    uint32_t x2 = squaresum / n0;
    return (x2 <= chi2);
}

Status_t testRNG(uint32_t observations, uint32_t m) {
    uint32_t* n = calloc(m, sizeof(uint32_t));
    if (n == NULL) {
        return Status_OutofMemory;
    }
    for (uint32_t i = 0; i < observations; i++) {
        n[create_random_number_manual() % m] += 1;
    }
    Status_t status = equalDistChi2(n, m, (observations / m), chiSquared(m - 1, ChiSquaredAlpha_10_percent));
    free (n);
    n = NULL;
    return status;
}

// Second week: test random numbers
// *************

void app_main(void)
{
    configure_led();

    int8_t leftButton = 9;
    int8_t rightButton = 2;

    gpio_config_t gpioConfigIn = {
        .pin_bit_mask = (1 << leftButton) | (1 << rightButton),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = true,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&gpioConfigIn);

    ESP_LOGI("Debug", "Everything configured, program start...");

    while (1)
    {
        uint32_t left = gpio_get_level(leftButton);
        uint32_t right = gpio_get_level(rightButton);
        if (left == 0)
        {
            rotate_max_number();
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
        if (right == 0)
        {
            create_random_number();
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}