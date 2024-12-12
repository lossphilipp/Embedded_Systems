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



bool game_running = true;
uint8_t matrix[5][5] = {0};
uint8_t falling_x = 2; // Starting in the middle of the top row
uint8_t falling_y = 0;
uint32_t last_button_press_time = 0;
TaskHandle_t gameTaskHandle = NULL;

uint8_t get_pixel_index(uint8_t x, uint8_t y)
{
    return y * 5 + x;
}

void clear_full_lines()
{
    for (uint8_t y = 4; y > 0; y--)
    {
        bool is_full = true;
        for (uint8_t x = 0; x < 5; x++)
        {
            if (matrix[y][x] == 0)
            {
                is_full = false;
                break;
            }
        }

        if (is_full)
        {
            // ToDo: Blink before going down
            ESP_LOGI("Debug", "Line %d is full", y);
            for (uint8_t yy = y; yy > 0; yy--)
            {
                for (uint8_t x = 0; x < 5; x++)
                {
                    matrix[yy][x] = matrix[yy - 1][x];
                }
            }

            for (uint8_t x = 0; x < 5; x++)
            {
                matrix[0][x] = 0;
            }

            // After clearing and shifting, recheck the same row (y)
            // because the new row that "fell down" might also be full
            // This is not needed right now, since only one pixel at
            // a time is falling
            // y++;
        }
    }
}

bool can_move_down()
{
    if (falling_y >= 4) {
        return false;
    }
    return matrix[falling_y + 1][falling_x] == 0;
}

void move_down()
{
    if (can_move_down())
    {
        matrix[falling_y][falling_x] = 0;
        falling_y++;
        matrix[falling_y][falling_x] = 1;
    }
    else
    {
        ESP_LOGI("Debug", "Pixel has hit bottom at x: %d & y: %d", falling_x, falling_y);

        // lost game
        if (matrix[0][2] != 0)
        {
            game_running = false;
        }
        else
        {
            // Spawn a new pixel
            falling_x = 2;
            falling_y = 0;
            matrix[falling_y][falling_x] = 1;
        }
    }

    clear_full_lines();
}

void move_left()
{
    ESP_LOGI("Debug", "Try to move left");
    if (falling_x > 0 && matrix[falling_y][falling_x - 1] == 0)
    {
        matrix[falling_y][falling_x] = 0;
        falling_x--;
        matrix[falling_y][falling_x] = 1;
    }
}

void move_right()
{
    ESP_LOGI("Debug", "Try to move right");
    if (falling_x < 4 && matrix[falling_y][falling_x + 1] == 0)
    {
        matrix[falling_y][falling_x] = 0;
        falling_x++;
        matrix[falling_y][falling_x] = 1;
    }
}

void draw_matrix()
{
    // ToDo: update to only draw the nex pixel instead of the whole matrix
    ESP_LOGI("Debug", "Drawing matrix");

    // Clear all LEDs before drawing
    led_strip_clear(led_strip);

    for (uint8_t y = 0; y < 5; y++)
    {
        for (uint8_t x = 0; x < 5; x++)
        {
            if (matrix[y][x] == 1)
            {
                uint8_t pixel_num = get_pixel_index(x, y);

                if (x == falling_x && y == falling_y)
                {
                    led_strip_set_pixel(led_strip, pixel_num, 0, 50, 0);
                }
                else
                {
                    led_strip_set_pixel(led_strip, pixel_num, 50, 0, 0);
                }
            }
        }
    }

    led_strip_refresh(led_strip);
}

void button_isr_handler(void* arg)
{
    uint32_t current_time = xTaskGetTickCountFromISR(); // get tick in ISR

    if (current_time - last_button_press_time > 50)
    {
        last_button_press_time = current_time;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        uint8_t gpio_num = (uint8_t)arg;

        // Notify the game task which button was pressed
        vTaskNotifyGiveFromISR(gameTaskHandle, &xHigherPriorityTaskWoken);

        if (xHigherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR(); // Yield if a higher priority task was woken
        }
    }
}

void configure_game()
{
    gpio_config_t gpioConfig = {
        .pin_bit_mask = (1 << BUTTON_GPIO_LEFT) | (1 << BUTTON_GPIO_RIGHT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = true ,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&gpioConfig);

    configure_led();
}

void configure_isr()
{
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO_LEFT, button_isr_handler, (void*) BUTTON_GPIO_LEFT);
    gpio_isr_handler_add(BUTTON_GPIO_RIGHT, button_isr_handler, (void*) BUTTON_GPIO_RIGHT);
}

void blink_matrix()
{
    for (int8_t i = 0; i < 3; i++)
    {
        for (int8_t j = 0; j < 25; j++)
        {
            led_strip_set_pixel(led_strip, j, 0, 0, 50);
        }

        led_strip_refresh(led_strip);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        led_strip_clear(led_strip);
        led_strip_refresh(led_strip);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void finish_game()
{
    ESP_LOGI("Debug", "Game is lost!");
    blink_matrix();
    
    gpio_isr_handler_remove(BUTTON_GPIO_LEFT);
    gpio_isr_handler_remove(BUTTON_GPIO_RIGHT);
    gpio_uninstall_isr_service();
}

void game_task(void* arg)
{
    while (true)
    {
        if (ulTaskNotifyTake(pdTRUE, 0) == pdTRUE)
        {
            if (gpio_get_level(BUTTON_GPIO_LEFT) == 0)
            {
                move_left();
            }
            else if (gpio_get_level(BUTTON_GPIO_RIGHT) == 0)
            {
                move_right();
            }

            draw_matrix();
        }

        vTaskDelay(BLINK_PERIOD / portTICK_PERIOD_MS);
    }
}

void button_check_bad()
{
    uint32_t left = gpio_get_level(BUTTON_GPIO_LEFT);
    uint32_t right = gpio_get_level(BUTTON_GPIO_RIGHT);

    if (left == 0)
    {
        move_left();
    }
    if (right == 0)
    {
        move_right();
    }
}

void app_main(void)
{
    configure_game();

    // xTaskCreate(game_task, "game_task", 2048, NULL, 10, &gameTaskHandle);
    // configure_isr();

    ESP_LOGI("Debug", "Starting game...");

    do
    {
        vTaskDelay(BLINK_PERIOD / portTICK_PERIOD_MS);
        move_down();
        button_check_bad();
        draw_matrix();
    } while (game_running);

    finish_game();
}