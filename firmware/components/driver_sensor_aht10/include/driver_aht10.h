/*
 * Driver for the AHT10 I2C temperature and humidity sensor
 * @author Bart Cerneels <bart.cerneels@gmail.com>
 *
 */

#ifndef DRIVER_AHT10_H
#define DRIVER_AHT10_H

#include <stdint.h>
#include <esp_err.h>

#define CACHE_TIMEOUT_MS 10 * (1000 * 1000)  // 10 seconds
#define SENSOR_NAN_VALUE 0xFFFF

__BEGIN_DECLS

extern esp_err_t driver_aht10_init(void);
extern float driver_aht10_get_temperature();
extern float driver_aht10_get_humidity();
extern esp_err_t driver_aht10_read_sensor(float *temperature, float *humidity);

__END_DECLS

#endif // DRIVER_AHT10_H
