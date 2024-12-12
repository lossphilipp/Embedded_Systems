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
#include <time.h>
#include <math.h>
#include "driver/i2c.h"

#define SHTC3_ADDRESS 0x70
#define CMD_WAKEUP 0x3517
#define MEAS_RH_FIRST 0x5C24

i2c_port_t i2c_port = I2C_NUM_0; 
uint8_t writeCmdWakeUp[] = {CMD_WAKEUP >> 8, CMD_WAKEUP & 0xFF};
uint8_t writeCmdMeasRH[] = {MEAS_RH_FIRST >> 8, MEAS_RH_FIRST & 0xFF};
uint8_t readBuf_humidity[3]; 

typedef struct {
    float relHumidity_percentage; 
    esp_err_t esp_err; 
} humidityMeasurement_t; 

void initI2C(i2c_port_t i2c_num) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER, 
        .sda_io_num = GPIO_NUM_5, 
        .scl_io_num = GPIO_NUM_6, 
        .sda_pullup_en = GPIO_PULLUP_ENABLE, 
        .scl_pullup_en = GPIO_PULLUP_ENABLE, 
        .master.clk_speed = 400000
    }; 
    ESP_ERROR_CHECK(i2c_param_config(i2c_num, &conf)); 
    ESP_ERROR_CHECK(i2c_driver_install(i2c_num, conf.mode, 0, 0, 0)); 

    ESP_LOGI("INFO", "I2C connection initialized"); 
}

humidityMeasurement_t readOutRelativeHumidity() {
    humidityMeasurement_t humidityMeasurement = { 0.0, ESP_OK }; 

    humidityMeasurement.esp_err = i2c_master_write_to_device(i2c_port, SHTC3_ADDRESS, writeCmdWakeUp, sizeof(writeCmdWakeUp), pdMS_TO_TICKS(50)); 
    if (humidityMeasurement.esp_err != ESP_OK) {
        ESP_LOGI("ESP_ERR", "Write cmd Wake up: %d", humidityMeasurement.esp_err);
    }
    vTaskDelay(pdMS_TO_TICKS(1));

    humidityMeasurement.esp_err = i2c_master_write_to_device(i2c_port, SHTC3_ADDRESS, writeCmdMeasRH, sizeof(writeCmdMeasRH), pdMS_TO_TICKS(50));  
    if (humidityMeasurement.esp_err != ESP_OK) {
        ESP_LOGI("ESP_ERR", "Write cmd Measure: %d", humidityMeasurement.esp_err);
    }
    vTaskDelay(pdMS_TO_TICKS(15));

    humidityMeasurement.esp_err = i2c_master_read_from_device(i2c_port, SHTC3_ADDRESS, readBuf_humidity, sizeof(readBuf_humidity), pdMS_TO_TICKS(50));     
    if (humidityMeasurement.esp_err == ESP_OK) { 
        humidityMeasurement.relHumidity_percentage =  100.0* ((readBuf_humidity[1] << 8) | readBuf_humidity[0]) / pow(2, 16); 
        ESP_LOGI("HUMIDITY", "%f%%", humidityMeasurement.relHumidity_percentage);
    } 
    else {
        ESP_LOGI("ESP_ERR", "Read cmd Measure: %d", humidityMeasurement.esp_err);  
    }

    return humidityMeasurement; 
}


void jonas_app_main(void)
{
    initI2C(i2c_port); 
 
    while (1)
    {          
        readOutRelativeHumidity(); 
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}