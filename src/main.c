#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mcp2515.h"

static const char *TAG = "app";

void app_main()
{
    const bool receiver_online = false;

    mcp2515_t can;
    mcp2515_config_t cfg;
    mcp2515_get_default_config(&cfg);

    esp_err_t err = mcp2515_init(&can, &cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "MCP2515 init failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "MCP2515 initialized (SPI host=%d)", cfg.host);

    uint8_t counter = 0;
    while (1)
    {
        uint8_t tx_data[8] = {counter, 1, 2, 3, 4, 5, 6, 7};
        err = mcp2515_send_standard(&can, 0x123, tx_data, sizeof(tx_data));
        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "TX id=0x123 data0=%u", tx_data[0]);
        }
        else if (!receiver_online && (err == ESP_FAIL || err == ESP_ERR_TIMEOUT))
        {
            ESP_LOGI(TAG, "TX no ACK expected yet (receiver offline): %s", esp_err_to_name(err));
        }
        else
        {
            ESP_LOGW(TAG, "TX failed: %s", esp_err_to_name(err));
        }

        counter++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}