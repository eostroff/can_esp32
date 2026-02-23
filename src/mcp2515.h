#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        uint32_t id;
        bool extended;
        bool rtr;
        uint8_t dlc;
        uint8_t data[8];
    } mcp2515_can_frame_t;

    typedef struct
    {
        spi_host_device_t host;
        gpio_num_t pin_miso;
        gpio_num_t pin_mosi;
        gpio_num_t pin_sclk;
        gpio_num_t pin_cs;
        gpio_num_t pin_int;
        int clock_speed_hz;
        int queue_size;
        bool initialize_bus;
    } mcp2515_config_t;

    typedef struct
    {
        spi_device_handle_t spi;
        mcp2515_config_t cfg;
    } mcp2515_t;

    void mcp2515_get_default_config(mcp2515_config_t *cfg);

    esp_err_t mcp2515_init(mcp2515_t *dev, const mcp2515_config_t *cfg);
    esp_err_t mcp2515_set_mode_normal(mcp2515_t *dev);
    esp_err_t mcp2515_send_standard(mcp2515_t *dev, uint16_t id, const uint8_t *data, uint8_t dlc);
    esp_err_t mcp2515_read_rx0(mcp2515_t *dev, mcp2515_can_frame_t *frame, bool *frame_available);

#ifdef __cplusplus
}
#endif
