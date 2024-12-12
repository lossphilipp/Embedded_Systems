#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_strip.h"
#include <driver/i2c.h>

#define BLINK_GPIO CONFIG_BLINK_GPIO
#define BLINK_PERIOD CONFIG_BLINK_PERIOD

#define BUTTON_GPIO_LEFT CONFIG_BUTTON_GPIO_LEFT
#define BUTTON_GPIO_RIGHT CONFIG_BUTTON_GPIO_RIGHT

#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA_IO
#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL_IO
#define I2C_MASTER_BITRATE CONFIG_I2C_MASTER_BITRATE

// https://sensirion.com/de/produkte/katalog/SHTC3
#define SHTC3_I2C_ADDRESS 0x70
#define SHTC3_CMD_WAKEUP 0x3517
#define SHTC3_CMD_SLEEP 0xB098
#define SHTC3_MEAS_RH_FIRST 0x5C24
#define SHTC3_MEAS_T_FIRST 0x7CA2

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

    ESP_LOGD("LED", "LED configured"); 
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

    ESP_LOGD("LED", "LED configured"); 
}

#else
#error "unsupported LED type"
#endif

#ifdef SHTC3_I2C_ADDRESS

i2c_port_t i2c_port = I2C_NUM_0;

typedef struct {
    float humidity;
    float temperature;
} measurement_t; 

void initI2C(i2c_port_t i2c_num)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_BITRATE
    };

    i2c_param_config(i2c_num, &conf);
    ESP_ERROR_CHECK(i2c_driver_install(i2c_num, conf.mode, 0, 0, 0));

    ESP_LOGD("I2C", "I2C connection initialized"); 
}

void configure_humidity_sensor()
{
    initI2C(i2c_port);

    ESP_LOGD("SENSOR", "Humidity sensor configured"); 
}

void SHTC3_writeRegister(uint16_t command)
{
    uint8_t writeCmd[2] = {command >> 8, command & 0xFF};
    //ESP_ERROR_CHECK(i2c_master_write_to_device(i2c_port, SHTC3_I2C_ADDRESS, writeCmd, 2, pdMS_TO_TICKS(50)));
    i2c_master_write_to_device(i2c_port, SHTC3_I2C_ADDRESS, writeCmd, 2, pdMS_TO_TICKS(50));
}

void SHTC3_readRegister(uint8_t readBuffer[6])
{
    //ESP_ERROR_CHECK(i2c_master_read_from_device(i2c_port, SHTC3_I2C_ADDRESS, readBuffer, 6, pdMS_TO_TICKS(50)));
    i2c_master_read_from_device(i2c_port, SHTC3_I2C_ADDRESS, readBuffer, 6, pdMS_TO_TICKS(50));
}

uint8_t SHTC3_CalculateChecksum(uint16_t readValue)
{
    uint8_t crc = 0xFF;
    crc ^= (readValue >> 8);
    for (int i = 0; i < 8; i++) {
        crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    }
    crc ^= (readValue & 0xFF);
    for (int i = 0; i < 8; i++) {
        crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    }
    return crc;
}

measurement_t read_humidity_sensor()
{
    // ToDo: Somehow this only reads once and then always returns the same value
    uint8_t readBuf_humidity_sensor[6];

    SHTC3_writeRegister(SHTC3_CMD_WAKEUP);
    vTaskDelay(pdMS_TO_TICKS(1));
    SHTC3_writeRegister(SHTC3_MEAS_RH_FIRST);
    vTaskDelay(pdMS_TO_TICKS(15)); // Should be enough time for the sensor to measure

    SHTC3_readRegister(readBuf_humidity_sensor);
    uint16_t raw_humidity = (readBuf_humidity_sensor[0] << 8) | readBuf_humidity_sensor[1];
    uint16_t raw_temperature = (readBuf_humidity_sensor[3] << 8) | readBuf_humidity_sensor[4];

    float humidity;
    float temperature;

    if (SHTC3_CalculateChecksum(raw_humidity) == readBuf_humidity_sensor[2]) {
        humidity = 100.0 * raw_humidity / pow(2, 16);
        ESP_LOGD("DATA", "Read humidity as %f", humidity);
    } else {
        humidity = NAN;
        ESP_LOGE("DATA", "Checksum for humidity is wrong!");
    }

    if (SHTC3_CalculateChecksum(raw_humidity) == readBuf_humidity_sensor[2]) {
        temperature = 175.0 * raw_temperature / pow(2, 16) - 45.0;
        ESP_LOGD("DATA", "Read temperature as %f", temperature);
    } else {
        temperature = NAN;
        ESP_LOGE("DATA", "Checksum for temperature is wrong!");
    }

    vTaskDelay(pdMS_TO_TICKS(1));

    // SHTC3_writeRegister(SHTC3_CMD_SLEEP);

    measurement_t measurement = {
        .humidity = humidity,
        .temperature = temperature
    };

    return measurement;
}

#endif

void configure_buttons()
{
    gpio_config_t gpioConfigIn = {
            .pin_bit_mask = (1 << BUTTON_GPIO_LEFT) | (1 << BUTTON_GPIO_RIGHT),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = true,
            .pull_down_en = false,
            .intr_type = GPIO_INTR_DISABLE};
        gpio_config(&gpioConfigIn);

    ESP_LOGD("CONFIGURATION", "Buttons configured"); 
}

// ######################### Program #########################

#define MEASUREMENT_TYPE_TEMPERATURE 0
#define MEASUREMENT_TYPE_HUMIDITY 1

uint8_t measurement_type = 1; // magic number
measurement_t current_measurement = {
        .humidity = 0,
        .temperature = 0
    };

void read_sensordata()
{
    current_measurement = read_humidity_sensor();
}

uint8_t get_pixel_index(uint8_t row, uint8_t col) {
    return row * 5 + col;
}

void draw_colorpattern()
{
    uint8_t r;
    uint8_t g;
    uint8_t b;

    if (measurement_type == MEASUREMENT_TYPE_TEMPERATURE)
    {
        r = (current_measurement.temperature * 50) / 40;
        g = 0;
        b = 50 - r;
    }
    else if (measurement_type == MEASUREMENT_TYPE_HUMIDITY)
    {
        g = 50 - abs(current_measurement.humidity - 50);
        r = 50 - g;
        b = 0;
    }
    else
    {
        r = 50;
        g = 50;
        b = 50;
    }

    for (uint8_t row = 2; row < 5; row++) {
        for (uint8_t col = 0; col < 5; col++) {
            uint8_t pixel_index = get_pixel_index(row, col);
            led_strip_set_pixel(led_strip, pixel_index, r, g, b);
        }
    }
}

void calculate_binary(float number, uint8_t binary_array[10]) {
    int integer_part = (int)number;
    float fractional_part = number - integer_part;

    for (int8_t i = 4; i >= 0; i--) {
        binary_array[i] = integer_part % 2;
        integer_part /= 2;
    }

    for (int8_t i = 5; i < 10; i++) {
        fractional_part *= 2;
        if (fractional_part >= 1.0) {
            binary_array[i] = 1;
            fractional_part -= 1.0;
        } else {
            binary_array[i] = 0;
        }
    }
}

void draw_value()
{
    uint8_t pixel_pattern[10];

    if (measurement_type == MEASUREMENT_TYPE_TEMPERATURE)
    {
        ESP_LOGI("DATA", "Displaying temperature with value %f", current_measurement.temperature);
        calculate_binary(current_measurement.temperature, pixel_pattern);
    }
    else if (measurement_type == MEASUREMENT_TYPE_HUMIDITY)
    {
        ESP_LOGI("DATA", "Displaying humidity with value %f", current_measurement.humidity);
        calculate_binary(current_measurement.humidity, pixel_pattern);
    }

    for (uint8_t i = 0; i < 10; i++) {
        if (pixel_pattern[i] == 1)
        {
            led_strip_set_pixel(led_strip, i, 30, 30, 30);
        }
    }
}

void draw_data()
{
    led_strip_clear(led_strip);

    draw_value();
    draw_colorpattern();

    led_strip_refresh(led_strip);
}

void switch_mode()
{
    measurement_type = (measurement_type + 1) % 2;
    ESP_LOGI("CONFIGURATION", "Switching mode to %d", measurement_type);

    draw_data();
}

void calibrate_humidity_sensor()
{
    // This has to be done since the very first measurement always fails
    // and the first few measurements are not very accurate
    for (uint8_t i = 0; i <= 10; i++)
    {
        draw_colorpattern();
        read_sensordata();
    }
}

void app_main(void)
{
    configure_led();
    configure_buttons();
    configure_humidity_sensor();

    ESP_LOGI("CONFIGURATION", "Everything configured, calibrating humidity sensor...");

    calibrate_humidity_sensor();

    ESP_LOGI("CONFIGURATION", "Everything configured, program start...");

    while (1)
    {
        // For performance reasons, check the input more often than drawing
        for (uint8_t i = 0; i < 5; i++)
        {
            if (gpio_get_level(BUTTON_GPIO_RIGHT) == 0)
            {
                switch_mode();
            }

            vTaskDelay(200 / portTICK_PERIOD_MS);
        }

        read_sensordata();
        draw_data();
    }
}