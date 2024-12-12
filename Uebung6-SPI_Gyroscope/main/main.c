#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "led_strip.h"
#include "esp_random.h"
#include "driver/spi_master.h"

#define BLINK_GPIO CONFIG_BLINK_GPIO
#define BLINK_PERIOD CONFIG_BLINK_PERIOD
#define BUTTON_GPIO_LEFT CONFIG_BUTTON_GPIO_LEFT
#define BUTTON_GPIO_RIGHT CONFIG_BUTTON_GPIO_RIGHT


#define GYRO_SPI_REG 0x80
#define GYRO_SPI_MSB_CLEAR 0x7F

#define GYRO_SPI_ACCEL_XOUT_H 0x1F | GYRO_SPI_REG
#define GYRO_SPI_ACCEL_XOUT_L 0x20 | GYRO_SPI_REG
#define GYRO_SPI_ACCEL_YOUT_H 0x21 | GYRO_SPI_REG
#define GYRO_SPI_ACCEL_YOUT_L 0x22 | GYRO_SPI_REG
#define GYRO_SPI_ACCEL_ZOUT_H 0x23 | GYRO_SPI_REG
#define GYRO_SPI_ACCEL_ZOUT_L 0x24 | GYRO_SPI_REG

#define GYRO_SHAKE_THRESHOLD 1000

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

#ifdef GYRO_SPI_REG
static spi_device_interface_config_t spiDeviceConfig = { 0 };
static spi_device_handle_t spiDeviceHandle;

bool changes[10] = {false, false, false, false, false, false, false, false, false, false};
uint8_t curr_change_index = 0;

uint16_t accel_threshold = 1000;
uint16_t rolling_threshold = 4;

static void initSPI(spi_host_device_t spiHost) {
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = GPIO_NUM_4,
        .miso_io_num = GPIO_NUM_7,
        .sclk_io_num = GPIO_NUM_10,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = 0 // DMA default
    };
    esp_err_t res = spi_bus_initialize(spiHost, &bus_cfg, SPI_DMA_CH_AUTO);
    if (res != ESP_OK) {
        ESP_LOGE("CONFIGURATION", "spi_bus_initialize() FAILED: %d!", res);
        return;
    } else {
        ESP_LOGD("CONFIGURATION", "spi_bus_initialize() OK");
    }
}

static void configure_gyroscope()
{
    initSPI(SPI2_HOST);

    spiDeviceConfig.command_bits = 0;
    spiDeviceConfig.address_bits = 0;
    spiDeviceConfig.dummy_bits = 0;
    spiDeviceConfig.duty_cycle_pos = 128;
    spiDeviceConfig.mode = 0; // CPOL = 0, CPHA = 0
    spiDeviceConfig.clock_source = SPI_CLK_SRC_DEFAULT;
    spiDeviceConfig.clock_speed_hz = 500000; // 500 kbit/s
    spiDeviceConfig.spics_io_num = GPIO_NUM_1;
    spiDeviceConfig.flags = 0; //SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_DUMMY;
    spiDeviceConfig.queue_size = 3;
    spiDeviceConfig.cs_ena_posttrans = 3;
    spi_bus_add_device(SPI2_HOST, &spiDeviceConfig, &spiDeviceHandle);
}

uint8_t* gyroscop_read(uint8_t command)
{
    spi_transaction_t spiTransaction = { 0 };

    uint8_t cmd[16];
    cmd[0] = command;
    memset(&cmd[1], 0xFF, 15);

    uint8_t rcv[16];
    memset(&rcv[0], 0xCC, 16);

    spiTransaction.length = 8 * 2; // 8 bit send, 8 bit receive, only the register
    spiTransaction.rxlength = 0;
    spiTransaction.flags = 0; // SPI_TRANS_CS_KEEP_ACTIVE;
    spiTransaction.tx_buffer = &cmd;
    spiTransaction.rx_buffer = &rcv;

    esp_err_t ret = spi_device_transmit(spiDeviceHandle, &spiTransaction);
    if (ret == ESP_OK) {
        ESP_LOGD("GYRO", "got data from gyro: %X %X", rcv[0], rcv[1]);
        return rcv[1];
    } else {
        ESP_LOGE("GYRO", "SPI transmission failed!");
        return ESP_FAIL;
    }
}

esp_err_t write_register(uint8_t reg, uint8_t value) {
    spi_transaction_t spiTransaction = { 0 };

    uint8_t cmd[2];
    cmd[0] = reg & GYRO_SPI_MSB_CLEAR;
    cmd[1] = value;

    spiTransaction.length = 8 * 2;
    spiTransaction.tx_buffer = cmd;
    spiTransaction.rx_buffer = NULL;

    // Transmit the SPI transaction
    esp_err_t ret = spi_device_transmit(spiDeviceHandle, &spiTransaction);
    if (ret != ESP_OK) {
        ESP_LOGE("GYRO", "SPI write failed: %d\n", ret);
        return ret;
    }

    ESP_LOGD("GYRO", "Wrote 0x%02X to register 0x%02X\n", value, reg);
    return ESP_OK;
}

bool gyroscope_whoami()
{
    spi_transaction_t spiTransaction = { 0 };
    uint8_t cmd[16];
    cmd[0] = 0x75 | 0x80; // WHO AM I, Read
    cmd[1] = 0xFF; // second byte, for response transmit
    uint8_t rcv[16];
    rcv[0] = 0xCC;
    rcv[1] = 0xCC;
    spiTransaction.length = 8 * 2; // 8 bit send, 8 bit receive, only the register
    spiTransaction.rxlength = 0;
    spiTransaction.flags = 0; // SPI_TRANS_CS_KEEP_ACTIVE;
    spiTransaction.tx_buffer = &cmd;
    spiTransaction.rx_buffer = &rcv;
    esp_err_t ret = spi_device_transmit(spiDeviceHandle, &spiTransaction);
    if (ret == ESP_OK) {
        ESP_LOGI("WHOAMI", "got data from ICM42688: %X %X", rcv[0], rcv[1]);
        if (rcv[1] == 0x47) { // see datasheet
            ESP_LOGI("WHOAMI", "gyro responds, ID ok");
            return true;
        } else {
            ESP_LOGE("WHOAMI", "gyro responds, but wrong ID!");
            //return ESP_ERR_INVALID_VERSION;
        }
    } else {
        ESP_LOGE("WHOAMI", "communication with ICM42688 failed");
        //return ESP_ERR_INVALID_RESPONSE;
    }
    return false;
}

static bool gyro_is_shaken()
{
    uint16_t meanX = 0;
    uint16_t meanY = 0;
    uint16_t meanZ = 0;

    uint8_t upper_x = gyroscop_read(GYRO_SPI_ACCEL_XOUT_H);
    uint8_t lower_x = gyroscop_read(GYRO_SPI_ACCEL_XOUT_L);
    uint16_t x = ((upper_x << 8) | lower_x);

    uint8_t upper_y = gyroscop_read(GYRO_SPI_ACCEL_YOUT_H);
    uint8_t lower_y = gyroscop_read(GYRO_SPI_ACCEL_YOUT_L);
    uint16_t y = ((upper_y << 8) | lower_y);

    uint8_t upper_z = gyroscop_read(GYRO_SPI_ACCEL_ZOUT_H);
    uint8_t lower_z = gyroscop_read(GYRO_SPI_ACCEL_ZOUT_L);
    uint16_t z = ((upper_z << 8) | lower_z);

    if(meanX == 0 && meanY == 0 && meanZ == 0){
        meanX = x;
        meanY = y;
        meanZ = z;
    } else {
        uint16_t tempMeanX = (meanX + x) / 2;
        uint16_t tempmeanY = (meanY + y) / 2;
        uint16_t tempmeanZ = (meanZ + z) / 2;

        if(abs(meanX - x) > accel_threshold){
            changes[curr_change_index] = true;
        }
        else if(abs(meanY - y) > accel_threshold){
            changes[curr_change_index] = true;
        }
        else if(abs(meanZ - z) > accel_threshold){
            changes[curr_change_index] = true;
        } else {
            changes[curr_change_index] = false;
        }
        curr_change_index = (curr_change_index + 1) % 10;

        meanX = tempMeanX;
        meanY = tempmeanY;
        meanZ = tempmeanZ;

        uint8_t count = 0;
        for (int i = 0; i < 10; i++) {
            if(changes[i]){
                count++;
            }
        }

        if(count > rolling_threshold){
            for (int i = 0; i < 10; i++) {
                changes[i] = false;
            }
            meanX = 0;
            meanY = 0;
            meanZ = 0;
            return true;
        }
    }

    return false;
}
#endif

static void configure_buttons()
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
    ESP_LOGD("BUTTON", "Left Pressed");

    current_max_number_index = (current_max_number_index + 1) % 4;
    uint8_t current_max_number = max_numbers[current_max_number_index];

    ESP_LOGI("DATA", "Setting max number: %d", current_max_number);
    draw_number(current_max_number, true);
}

void create_random_number() {
    ESP_LOGD("DATA", "Random number generated");

    uint8_t current_max_number = max_numbers[current_max_number_index];
    uint8_t number = (esp_random() % current_max_number) + 1;
    ESP_LOGI("DATA", "\nMaximal number:    %d\nCalculated number: %d", current_max_number, number);

    draw_number(number, false);
}

void app_main(void)
{
    configure_led();
    configure_buttons();
    configure_gyroscope();

    bool gyroscopeWorking = gyroscope_whoami();
    if (!gyroscopeWorking) {
        ESP_LOGE("CONFIGURATION", "Gyroscope configuration failed, exiting...");
        return;
    }

    write_register(0x4E, 0x0F);
    write_register(0x50, 0x00);

    ESP_LOGI("CONFIGURATION", "Everything configured, program start...");

    while (1)
    {
        if (gyro_is_shaken())
        {
            ESP_LOGI("SHAKE", "Sensor is shaken!");
            do
            {
                draw_roll_animation();
            }
            while (gyro_is_shaken());

            ESP_LOGI("SHAKE", "Sensor stopped shaking!");
            create_random_number();
        }

        if (gpio_get_level(BUTTON_GPIO_LEFT) == 0)
        {
            rotate_max_number();
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}