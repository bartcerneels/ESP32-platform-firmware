#include <sdkconfig.h>

#include "include/driver_aht10.h"
#include "include/driver_i2c.h"

#include "esp_log.h"

#include <stdbool.h>

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
    return 0.0f;
}

float driver_aht10_get_humidity()
{
    return 100.0f;
}

esp_err_t driver_aht10_read_sensor(float *temperature, float *humidity)
{
    return ESP_OK;
}

#else //CONFIG_DRIVER_AHT10_ENABLE
esp_err_t driver_aht10_init()
{
    return ESP_OK;
}
#endif //CONFIG_DRIVER_AHT10_ENABLE
