/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

// static uint8_t s_led_state = 0;
static uint8_t led_rows = 5;
static uint8_t column_number = 0;
static uint8_t nightrider_status[5] = {0, 0, 0, 0, 0};
static uint8_t offset;

#ifdef CONFIG_BLINK_LED_STRIP

static led_strip_handle_t led_strip;

// static void blink_led(void)
// {
//     /* If the addressable LED is enabled */
//     if (s_led_state) {
//         /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
//         led_strip_set_pixel(led_strip, 0, 16, 16, 16);
//         /* Refresh the strip to send data */
//         led_strip_refresh(led_strip);
//     } else {
//         /* Set all LED off to clear all pixels */
//         led_strip_clear(led_strip);
//     }
// }

// static void straight(void)
// {
//     for (size_t i = 0; i < 25; i++)
//     {
//         if ((i + led_rows - column_number) % led_rows == 0)
//         {
//             led_strip_set_pixel(led_strip, i, 50, 0, 0);
//         }
//         else
//         {
//             led_strip_set_pixel(led_strip, i, 00, 00, 0);
//         }
//     }

//     led_strip_refresh(led_strip);
// }

// static void diagonal(void)
// {
//     for (size_t i = 0; i < 25; i++)
//     {
//         uint8_t offset = i / led_rows;

//         if ((i + led_rows - (column_number + offset)) % led_rows == 0)
//         {
//             led_strip_set_pixel(led_strip, i, 50, 0, 0);
//         }
//         else
//         {
//             led_strip_set_pixel(led_strip, i, 00, 00, 0);
//         }
//     }

//     led_strip_refresh(led_strip);
// }

static void nightrider(void)
{
    for (size_t i = 0; i < 25; i++)
    {
        uint8_t row = i / led_rows;

        if ((i + led_rows - nightrider_status[row]) % led_rows == 0)
        {
            led_strip_set_pixel(led_strip, i, 50, 0, 0);  // Set the pixel to red
        }
        else
        {
            led_strip_set_pixel(led_strip, i, 0, 0, 0);  // Turn off the pixel
        }
    }

    led_strip_refresh(led_strip);  // Refresh the LED strip
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
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

void app_main(void)
{

    /* Configure the peripheral according to the LED type */
    configure_led();

    // while (1) {
    //     ESP_LOGI(TAG, "column_number: %d", column_number);
    //     diagonal();
    //     column_number = (column_number + 1) % led_rows;
    //     vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
    // }

    //// bouncing
    // while (1) {
    //     for (size_t i = 0; i < (led_rows * 2 - 2); i++)
    //     {
    //         ESP_LOGI(TAG, "column_number: %d", column_number);
    //         diagonal();

    //         if (i <= (led_rows - 1))
    //         {
    //             column_number = i;
    //         }
    //         else
    //         {
    //             column_number = (led_rows * 2 - 2) - i;
    //         }

    //         vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
    //     }
    // }

    // nightrider
    while (1) {
        for (size_t i = 0; i < (led_rows * 2 - 2); i++)
        {
            for (size_t row = 0; row < led_rows; row++)
            {
                offset = i + row;
                if (offset <= (led_rows - 1))
                {
                    nightrider_status[row] = offset;
                }
                else if (offset > (led_rows * 2 - 2))
                {
                    nightrider_status[row] = offset % (led_rows * 2 - 2);
                }
                else
                {
                    nightrider_status[row] = (led_rows * 2 - 2) - offset;
                }
            }

            ESP_LOGI(TAG, "i: %d", i);
            nightrider();
            vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);  // Delay for blinking effect
        }
    }
}
