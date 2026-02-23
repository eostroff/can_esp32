#include "mcp2515.h"

#include <string.h>
#include "esp_check.h"

#define MCP2515_CMD_RESET 0xC0
#define MCP2515_CMD_READ 0x03
#define MCP2515_CMD_READ_RX_BUF_0 0x90
#define MCP2515_CMD_WRITE 0x02
#define MCP2515_CMD_LOAD_TX_BUF_0 0x40
#define MCP2515_CMD_RTS_TX0 0x81
#define MCP2515_CMD_READ_STATUS 0xA0
#define MCP2515_CMD_BIT_MODIFY 0x05

#define MCP2515_REG_CANSTAT 0x0E
#define MCP2515_REG_CANCTRL 0x0F
#define MCP2515_REG_CNF3 0x28
#define MCP2515_REG_CNF2 0x29
#define MCP2515_REG_CNF1 0x2A
#define MCP2515_REG_CANINTE 0x2B
#define MCP2515_REG_CANINTF 0x2C
#define MCP2515_REG_RXB0CTRL 0x60
#define MCP2515_REG_TXB0CTRL 0x30

#define MCP2515_MODE_NORMAL 0x00
#define MCP2515_MODE_CONFIG 0x80
#define MCP2515_MODE_MASK 0xE0
#define MCP2515_CANCTRL_OSM 0x08

#define MCP2515_RX0IF 0x01
#define MCP2515_TXREQ 0x08
#define MCP2515_TXERR 0x10
#define MCP2515_MLOA 0x20
#define MCP2515_ABTF 0x40

static const char *TAG = "mcp2515";

static esp_err_t mcp2515_transfer(mcp2515_t *dev, const uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    return spi_device_transmit(dev->spi, &t);
}

static esp_err_t mcp2515_write_reg(mcp2515_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t tx[3] = {MCP2515_CMD_WRITE, reg, val};
    return mcp2515_transfer(dev, tx, NULL, sizeof(tx));
}

static esp_err_t mcp2515_read_reg(mcp2515_t *dev, uint8_t reg, uint8_t *val)
{
    uint8_t tx[3] = {MCP2515_CMD_READ, reg, 0x00};
    uint8_t rx[3] = {0};
    esp_err_t err = mcp2515_transfer(dev, tx, rx, sizeof(tx));
    if (err != ESP_OK)
    {
        return err;
    }
    *val = rx[2];
    return ESP_OK;
}

static esp_err_t mcp2515_bit_modify(mcp2515_t *dev, uint8_t reg, uint8_t mask, uint8_t data)
{
    uint8_t tx[4] = {MCP2515_CMD_BIT_MODIFY, reg, mask, data};
    return mcp2515_transfer(dev, tx, NULL, sizeof(tx));
}

static esp_err_t mcp2515_read_status(mcp2515_t *dev, uint8_t *status)
{
    uint8_t tx[2] = {MCP2515_CMD_READ_STATUS, 0x00};
    uint8_t rx[2] = {0};
    esp_err_t err = mcp2515_transfer(dev, tx, rx, sizeof(tx));
    if (err != ESP_OK)
    {
        return err;
    }
    *status = rx[1];
    return ESP_OK;
}

void mcp2515_get_default_config(mcp2515_config_t *cfg)
{
    if (!cfg)
    {
        return;
    }

    *cfg = (mcp2515_config_t){
        .host = SPI2_HOST,
        .pin_miso = GPIO_NUM_19,
        .pin_mosi = GPIO_NUM_23,
        .pin_sclk = GPIO_NUM_18,
        .pin_cs = GPIO_NUM_5,
        .pin_int = GPIO_NUM_4,
        .clock_speed_hz = 10 * 1000 * 1000,
        .queue_size = 3,
        .initialize_bus = true,
    };
}

static esp_err_t mcp2515_hw_reset(mcp2515_t *dev)
{
    uint8_t cmd = MCP2515_CMD_RESET;
    ESP_RETURN_ON_ERROR(mcp2515_transfer(dev, &cmd, NULL, 1), TAG, "reset command failed");
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

static esp_err_t mcp2515_set_mode(mcp2515_t *dev, uint8_t mode)
{
    ESP_RETURN_ON_ERROR(mcp2515_bit_modify(dev, MCP2515_REG_CANCTRL, MCP2515_MODE_MASK, mode), TAG, "set mode failed");
    for (int i = 0; i < 10; i++)
    {
        uint8_t canstat = 0;
        ESP_RETURN_ON_ERROR(mcp2515_read_reg(dev, MCP2515_REG_CANSTAT, &canstat), TAG, "read CANSTAT failed");
        if ((canstat & MCP2515_MODE_MASK) == mode)
        {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t mcp2515_init(mcp2515_t *dev, const mcp2515_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(dev && cfg, ESP_ERR_INVALID_ARG, TAG, "invalid args");
    memset(dev, 0, sizeof(*dev));
    dev->cfg = *cfg;

    if (cfg->initialize_bus)
    {
        spi_bus_config_t bus_cfg = {
            .miso_io_num = cfg->pin_miso,
            .mosi_io_num = cfg->pin_mosi,
            .sclk_io_num = cfg->pin_sclk,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 32,
        };
        ESP_RETURN_ON_ERROR(spi_bus_initialize(cfg->host, &bus_cfg, SPI_DMA_CH_AUTO), TAG, "spi_bus_initialize failed");
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = cfg->clock_speed_hz,
        .mode = 0,
        .spics_io_num = cfg->pin_cs,
        .queue_size = cfg->queue_size,
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_device(cfg->host, &dev_cfg, &dev->spi), TAG, "spi_bus_add_device failed");

    if (cfg->pin_int != GPIO_NUM_NC)
    {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << cfg->pin_int,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "int pin config failed");
    }

    ESP_RETURN_ON_ERROR(mcp2515_hw_reset(dev), TAG, "reset failed");
    ESP_RETURN_ON_ERROR(mcp2515_set_mode(dev, MCP2515_MODE_CONFIG), TAG, "failed to enter config mode");
    ESP_RETURN_ON_ERROR(mcp2515_bit_modify(dev, MCP2515_REG_CANCTRL, MCP2515_CANCTRL_OSM, MCP2515_CANCTRL_OSM), TAG, "failed to set one-shot mode");

    ESP_RETURN_ON_ERROR(mcp2515_write_reg(dev, MCP2515_REG_CNF1, 0x00), TAG, "CNF1 write failed");
    ESP_RETURN_ON_ERROR(mcp2515_write_reg(dev, MCP2515_REG_CNF2, 0x88), TAG, "CNF2 write failed");
    ESP_RETURN_ON_ERROR(mcp2515_write_reg(dev, MCP2515_REG_CNF3, 0x03), TAG, "CNF3 write failed");

    ESP_RETURN_ON_ERROR(mcp2515_write_reg(dev, MCP2515_REG_RXB0CTRL, 0x60), TAG, "RXB0CTRL write failed");
    ESP_RETURN_ON_ERROR(mcp2515_write_reg(dev, MCP2515_REG_CANINTE, MCP2515_RX0IF), TAG, "CANINTE write failed");

    return mcp2515_set_mode_normal(dev);
}

esp_err_t mcp2515_set_mode_normal(mcp2515_t *dev)
{
    return mcp2515_set_mode(dev, MCP2515_MODE_NORMAL);
}

esp_err_t mcp2515_send_standard(mcp2515_t *dev, uint16_t id, const uint8_t *data, uint8_t dlc)
{
    ESP_RETURN_ON_FALSE(dev && dlc <= 8, ESP_ERR_INVALID_ARG, TAG, "invalid args");

    uint8_t txb0ctrl = 0;
    ESP_RETURN_ON_ERROR(mcp2515_read_reg(dev, MCP2515_REG_TXB0CTRL, &txb0ctrl), TAG, "read TXB0CTRL failed");
    if (txb0ctrl & MCP2515_TXREQ)
    {
        ESP_RETURN_ON_ERROR(mcp2515_bit_modify(dev, MCP2515_REG_TXB0CTRL, MCP2515_TXREQ, 0), TAG, "abort stuck TX failed");
        ESP_RETURN_ON_ERROR(mcp2515_bit_modify(dev, MCP2515_REG_TXB0CTRL, MCP2515_ABTF | MCP2515_MLOA | MCP2515_TXERR, 0), TAG, "clear TX status failed");
        return ESP_ERR_TIMEOUT;
    }

    uint8_t tx[14] = {0};
    tx[0] = MCP2515_CMD_LOAD_TX_BUF_0;
    tx[1] = (uint8_t)(id >> 3);
    tx[2] = (uint8_t)((id & 0x07) << 5);
    tx[3] = 0;
    tx[4] = 0;
    tx[5] = dlc & 0x0F;
    if (data && dlc > 0)
    {
        memcpy(&tx[6], data, dlc);
    }

    ESP_RETURN_ON_ERROR(mcp2515_transfer(dev, tx, NULL, 6 + dlc), TAG, "load tx buffer failed");

    uint8_t rts = MCP2515_CMD_RTS_TX0;
    ESP_RETURN_ON_ERROR(mcp2515_transfer(dev, &rts, NULL, 1), TAG, "RTS failed");

    for (int i = 0; i < 20; i++)
    {
        ESP_RETURN_ON_ERROR(mcp2515_read_reg(dev, MCP2515_REG_TXB0CTRL, &txb0ctrl), TAG, "read TX status failed");
        if (!(txb0ctrl & MCP2515_TXREQ))
        {
            if (txb0ctrl & (MCP2515_ABTF | MCP2515_MLOA | MCP2515_TXERR))
            {
                ESP_RETURN_ON_ERROR(mcp2515_bit_modify(dev, MCP2515_REG_TXB0CTRL, MCP2515_ABTF | MCP2515_MLOA | MCP2515_TXERR, 0), TAG, "clear TX status failed");
                return ESP_FAIL;
            }
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    ESP_RETURN_ON_ERROR(mcp2515_bit_modify(dev, MCP2515_REG_TXB0CTRL, MCP2515_TXREQ, 0), TAG, "abort timed-out TX failed");
    return ESP_ERR_TIMEOUT;
}

esp_err_t mcp2515_read_rx0(mcp2515_t *dev, mcp2515_can_frame_t *frame, bool *frame_available)
{
    ESP_RETURN_ON_FALSE(dev && frame && frame_available, ESP_ERR_INVALID_ARG, TAG, "invalid args");

    uint8_t status = 0;
    ESP_RETURN_ON_ERROR(mcp2515_read_status(dev, &status), TAG, "read status failed");
    if (!(status & MCP2515_RX0IF))
    {
        *frame_available = false;
        return ESP_OK;
    }

    uint8_t tx[14] = {0};
    uint8_t rx[14] = {0};
    tx[0] = MCP2515_CMD_READ_RX_BUF_0;
    ESP_RETURN_ON_ERROR(mcp2515_transfer(dev, tx, rx, sizeof(tx)), TAG, "read rx buffer failed");

    uint8_t sidh = rx[1];
    uint8_t sidl = rx[2];
    frame->id = ((uint32_t)sidh << 3) | (sidl >> 5);
    frame->extended = false;
    frame->rtr = false;
    frame->dlc = rx[5] & 0x0F;
    if (frame->dlc > 8)
    {
        frame->dlc = 8;
    }
    memcpy(frame->data, &rx[6], frame->dlc);

    ESP_RETURN_ON_ERROR(mcp2515_bit_modify(dev, MCP2515_REG_CANINTF, MCP2515_RX0IF, 0), TAG, "clear RX0IF failed");

    *frame_available = true;
    return ESP_OK;
}
