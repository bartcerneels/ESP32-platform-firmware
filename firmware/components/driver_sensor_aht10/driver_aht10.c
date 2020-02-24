#include <sdkconfig.h>

#include "include/driver_aht10.h"
#include "include/driver_i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include <stdbool.h>
#include <math.h>

#ifdef CONFIG_DRIVER_AHT10_ENABLE
#if CONFIG_I2C_MASTER_FREQ_HZ > 400000
#error \
    "The AM2320 sensor supports an I2C clock of at most 400kHz."
#endif //CONFIG_I2C_MASTER_FREQ_HZ > 400000

#define AHT10_ADDRESS 0x38

#define AHT10_INIT_CMD 0b11100001
#define AHT10_TRIGGER_CMD 0b10101100
#define AHT10_RESET_CMD 0b10111010

#define AHT10_STATUS_BUSY_MASK (1<<7)
#define AHT10_STATUS_MODE_MASK (0b11<<5)
#define AHT10_STATUS_CALIBRATED_MASK (1<<3)

static const char *TAG              = "driver_aht10";
static bool driver_aht10_init_done = false;

static unsigned int status()
{
    uint8_t status = 0;
    if (driver_i2c_read_bytes(AHT10_ADDRESS, &status, 1) != ESP_OK)
        ESP_LOGE(TAG, "Failed to read status!");

    return status;
}

esp_err_t driver_aht10_init()
{
    if (driver_aht10_init_done)
      return ESP_OK;

    ESP_LOGD(TAG, "init called");

    uint8_t buf[3];
    buf[0] = AHT10_INIT_CMD;
    buf[1] = 0x08;
    buf[2] = 0x00;

    esp_err_t err = driver_i2c_write_buffer(AHT10_ADDRESS, buf, sizeof(buf));
    if (err != ESP_OK)
        return err;

    driver_aht10_init_done = true;

    ESP_LOGD(TAG, "init done");
    return ESP_OK;
}

float driver_aht10_get_temperature()
{
    float temperature = NAN;
    float humidity = NAN;

    if (driver_aht10_read_sensor(&temperature, &humidity) != ESP_OK)
        return NAN;

    return temperature;
}

float driver_aht10_get_humidity()
{
    float temperature = NAN;
    float humidity = NAN;

    if (driver_aht10_read_sensor(&temperature, &humidity) != ESP_OK)
        return NAN;

    return humidity;
}

esp_err_t driver_aht10_read_sensor(float *temperature, float *humidity)
{
    esp_err_t err = driver_i2c_write_byte(AHT10_ADDRESS, AHT10_TRIGGER_CMD);
    if (err != ESP_OK)
        return err;

    ESP_LOGD(TAG, "Wait for sensor to be ready");
    while(status() & AHT10_STATUS_BUSY_MASK)
        vTaskDelay(300/portTICK_RATE_MS);

    ESP_LOGD(TAG, "Reading values");
    uint8_t data[6];
    err = driver_i2c_read_bytes(AHT10_ADDRESS, data, sizeof(data));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read values!");
        return err;
    }

    unsigned int rh_data = data[1]<<12 | data[2]<<4 | data[3]>>4;
    //round to whole integer
    *humidity = roundf((rh_data * 100.0F) / (1<<20));

    unsigned int t_data = (data[3] & 0x0F)<<16 | data[4]<<8 | data[5];
    // Round to 0.1
    *temperature = ((t_data * 200) / (1<<20)) - 50;

    return ESP_OK;
}

#else //CONFIG_DRIVER_AHT10_ENABLE
esp_err_t driver_aht10_init()
{
    return ESP_OK;
}
#endif //CONFIG_DRIVER_AHT10_ENABLE
