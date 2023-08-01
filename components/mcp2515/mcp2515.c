#include "mcp2515.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/spi_master.h"


//FROM h FILE

        // static const uint32_t DEFAULT_SPI_CLOCK = 10000000; // 10MHz

        // static const int N_TXBUFFERS = 3;
        // static const int N_RXBUFFERS = 2;
#define N_TXBUFFERS 3
#define N_RXBUFFERS 2

        // static const T_TXBn_REGS TXB[N_TXBUFFERS];
        // static const RXBn_REGS RXB[N_RXBUFFERS];
//FROM h FILE END

static const char *TAG = "mcp2515";

static const struct TXBn_REGS TXB[N_TXBUFFERS] = {
    {MCP_TXB0CTRL, MCP_TXB0SIDH, MCP_TXB0DATA},
    {MCP_TXB1CTRL, MCP_TXB1SIDH, MCP_TXB1DATA},
    {MCP_TXB2CTRL, MCP_TXB2SIDH, MCP_TXB2DATA}
};

static const struct RXBn_REGS RXB[N_RXBUFFERS] = {
    {MCP_RXB0CTRL, MCP_RXB0SIDH, MCP_RXB0DATA, CANINTF_RX0IF},
    {MCP_RXB1CTRL, MCP_RXB1SIDH, MCP_RXB1DATA, CANINTF_RX1IF}
};
/*
MCP2515::MCP2515(const uint8_t _CS, const uint32_t _SPI_CLOCK, SPIClass * _SPI)
{
    if (_SPI != nullptr) {
        SPIn = _SPI;
    }
    else {
        SPIn = &SPI;
        SPIn->begin();
    }

    SPICS = _CS;
    SPI_CLOCK = _SPI_CLOCK;
    pinMode(SPICS, OUTPUT);
    MCP____endSPI();
}
*/
//VOCOm hardware
// #define PIN_NUM_MISO 12
// #define PIN_NUM_MOSI 13
// #define PIN_NUM_CLK  14
// #define PIN_NUM_CS   2

//Can2ethard
#define PIN_NUM_MISO 12
#define PIN_NUM_MOSI 13
#define PIN_NUM_CLK  14
#define PIN_NUM_CS   2



#define EEPROM_HOST SPI2_HOST

static spi_device_handle_t spi;

static SemaphoreHandle_t spiBusMutex = NULL;
    
void MCP____INIT() {
    ESP_LOGI(TAG, "Initializing bus SPI%d...", EEPROM_HOST+1);

    if ( spiBusMutex == NULL ) {
        spiBusMutex = xSemaphoreCreateMutex();
    }

    spi_bus_config_t buscfg={
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .max_transfer_sz = 32,
        .flags = SPICOMMON_BUSFLAG_MASTER,
        .intr_flags = 0
    };
    //Initialize the SPI bus
    esp_err_t ret = spi_bus_initialize(EEPROM_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    spi_device_interface_config_t devcfg ={
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,                                //SPI mode 0
        .duty_cycle_pos = 0,
        .clock_speed_hz = SPI_MASTER_FREQ_10M,           //Clock out at 1 MHz
        .spics_io_num = PIN_NUM_CS,               //CS pin
        .flags = SPI_DEVICE_NO_DUMMY,
        .queue_size = 7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb = NULL,
        .post_cb = NULL
    };
    
    ret = spi_bus_add_device(EEPROM_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "SPI BUS INITED");
}

void MCP____DEINIT() {
    ESP_LOGI(TAG, "DeInitializing bus SPI%d...", EEPROM_HOST+1);
    
    ESP_ERROR_CHECK(spi_bus_remove_device(spi));

    ESP_ERROR_CHECK(spi_bus_free(EEPROM_HOST));

    vSemaphoreDelete(spiBusMutex);
    spiBusMutex = NULL;

    ESP_LOGI(TAG, "SPI BUS DEINITED");
}

uint8_t spi_transfer(uint8_t a) {
    esp_err_t ret;
    spi_transaction_t t = {
        .flags = SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_DUMMY
            | SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA
            ,
        .cmd = 0,
        .addr = 0,
        .length = 8,
        .rxlength = 8,
        .user = NULL,
        // .tx_buffer = &cmd,
        .tx_data = {a}
    };
    spi_transaction_ext_t spiext_t = {
        .base = t,
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0
    };
    ret = spi_device_polling_transmit(spi, (spi_transaction_t *) &spiext_t);  //Transmit!
    assert(ret==ESP_OK);

    return spiext_t.base.rx_data[0];
}

int MCP____startSPIFromISR() {
    // SPIn->beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
    // digitalWrite(SPICS, LOW);
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
    int failCtr = 0;
    while(failCtr++ < 1000) {
        BaseType_t ret = xSemaphoreTakeFromISR(spiBusMutex, &pxHigherPriorityTaskWoken);
        if (ret == pdPASS) {
            // ESP_LOGI(TAG, "SPI BUS MUTEX ACQUIRED");
            while (spi_device_acquire_bus(spi, portMAX_DELAY) != ESP_OK) {
                ESP_LOGW(TAG, "SPI BUS LOCK Acquire fail");
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            return 0;
        } else {
            // printf("Semphr acquire fail %d", pxHigherPriorityTaskWoken);
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    return -1;
}

int MCP____startSPI() {
    // SPIn->beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
    // digitalWrite(SPICS, LOW);
    // ESP_LOGI(TAG, "SPI mutex req from : %s", pcTaskGetName(NULL));
    BaseType_t ret = xSemaphoreTake(spiBusMutex, pdMS_TO_TICKS(10000));
    if (ret == pdPASS) {
        // ESP_LOGI(TAG, "SPI BUS MUTEX ACQUIRED");
        // ESP_LOGI(TAG, "SPI mutex got from : %s %p", pcTaskGetName(NULL), spiBusMutex);
        while (spi_device_acquire_bus(spi, portMAX_DELAY) != ESP_OK) {
            ESP_LOGW(TAG, "SPI BUS LOCK Acquire fail");
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        // ESP_LOGI(TAG, "SPI mutex got from : %s %p", pcTaskGetName(NULL), spiBusMutex);
        return 0;
    } else {
        TaskHandle_t tskHndle = xSemaphoreGetMutexHolder(spiBusMutex);
        ESP_LOGW(TAG, "Start SPI mutex FAIL, acq by: %s, req by: %s %p", 
            pcTaskGetName(tskHndle), pcTaskGetName(NULL), spiBusMutex);
    }
    return -1;
}

int MCP____endSPIFromISR() {
    // digitalWrite(SPICS, HIGH);
    // SPIn->endTransaction();
    
    spi_device_release_bus(spi);

    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
    int failCtr = 0;
    while(failCtr++ < 1000) {    
        BaseType_t ret = xSemaphoreGiveFromISR(spiBusMutex, &pxHigherPriorityTaskWoken);
        if (ret == pdPASS) {
            // ESP_LOGI(TAG, "SPI BUS MUTEX RELEASED");
            return 0;
        }
    }
    return -1;
}

int MCP____endSPI() {
    // digitalWrite(SPICS, HIGH);
    // SPIn->endTransaction();
    spi_device_release_bus(spi);
    BaseType_t ret = xSemaphoreGive(spiBusMutex);
    if (ret == pdPASS) {
        // ESP_LOGI(TAG, "SPI BUS MUTEX RELEASED");
        // ESP_LOGI(TAG, "SPI mutex released by : %s", pcTaskGetName(NULL));
        return 0;
    } else {
        ESP_LOGW(TAG, "Failed to release MUTEX");
    }
    return -1;
}

MCP__ERROR MCP____reset(void)
{
    MCP____startSPI();
    spi_transfer(INSTRUCTION_RESET);
    MCP____endSPI();

    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t zeros[14];
    memset(zeros, 0, sizeof(zeros));
    MCP____startSPI();
    MCP____setRegisters(MCP_TXB0CTRL, zeros, 14);
    MCP____setRegisters(MCP_TXB1CTRL, zeros, 14);
    MCP____setRegisters(MCP_TXB2CTRL, zeros, 14);

    MCP____setRegister(MCP_RXB0CTRL, 0);
    MCP____setRegister(MCP_RXB1CTRL, 0);

    MCP____setRegister(MCP_CANINTE, CANINTF_RX0IF | CANINTF_RX1IF | CANINTF_ERRIF | CANINTF_MERRF);

    // receives all valid messages using either Standard or Extended Identifiers that
    // meet filter criteria. RXF0 is applied for RXB0, RXF1 is applied for RXB1

    MCP____modifyRegister(MCP_RXB0CTRL,
                   RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT | RXB0CTRL_FILHIT_MASK,
                   RXBnCTRL_RXM_STDEXT | 0x00 | RXB0CTRL_FILHIT);
                //    RXBnCTRL_RXM_STDEXT | RXB0CTRL_BUKT | RXB0CTRL_FILHIT);
    MCP____modifyRegister(MCP_RXB1CTRL,
                   RXBnCTRL_RXM_MASK | RXB1CTRL_FILHIT_MASK,
                   RXBnCTRL_RXM_STDEXT | RXB1CTRL_FILHIT);
    MCP____endSPI();

    // clear filters and masks
    // do not filter any standard frames for RXF0 used by RXB0
    // do not filter any extended frames for RXF1 used by RXB1
    RXF filters[] = {RXF0, RXF1, RXF2, RXF3, RXF4, RXF5};
    for (int i=0; i<6; i++) {
        bool ext = (i == 1);
        MCP__ERROR result = MCP____setFilter(filters[i], ext, 0);
        if (result != MCP__ERROR_OK) {
            return result;
        }
    }

    MCP__MASK masks[] = {MCP__MASK0, MCP__MASK1};
    for (int i=0; i<2; i++) {
        MCP__ERROR result = MCP____setFilterMask(masks[i], true, 0);
        if (result != MCP__ERROR_OK) {
            return result;
        }
    }

    return MCP__ERROR_OK;
}

int MCP____readRXDataIfDataPresent(QueueHandle_t rxQueue) {

    struct can_frame canMsg2;
    MCP__ERROR readRet = MCP__ERROR_FAIL;

    uint8_t val = 0x00;
    int strtSpiRet = MCP____startSPI();
    if (strtSpiRet != 0) {
        return -7;
    }

    MCP____readRegisters(MCP_CANINTF, &val, 1);
    
    if ((val & MCP__STAT_RX0IF)) {
        // ESP_LOGI(TAG, "Interrupt RXB0 val: %02x", val);
        readRet = MCP____readMessageFromBuffer(RXB0, &canMsg2);
    } else if ((val & MCP__STAT_RX1IF)) {
        ESP_LOGI(TAG, "Interrupt RXB1 val: %02x", val);
        readRet = MCP____readMessageFromBuffer(RXB1, &canMsg2);
    } else if ((val & MCP__STAT_RX1IF)) {
        ESP_LOGI(TAG, "Interrupt RXB1 val: %02x", val);
        readRet = MCP____readMessageFromBuffer(RXB1, &canMsg2);
    } else if ((val & MCP__STAT_RX1IF)) {
        ESP_LOGI(TAG, "Interrupt RXB1 val: %02x", val);
        readRet = MCP____readMessageFromBuffer(RXB1, &canMsg2);
    } else {
        MCP____endSPI();
        return -1;
    }
    MCP____endSPI();
    if (readRet == MCP__ERROR_OK) {
        // ESP_LOGW(TAG, "Some data read without interrupt");
        BaseType_t retSend = xQueueSend(rxQueue, (void *) &canMsg2, pdMS_TO_TICKS(1000));
        if ( retSend == pdFALSE ) {
            return -2;
        } else {
            return 0;
        }
    } else {
        return -5;
    }
}

int MCP____readRXDataIfInterruptFromISR(QueueHandle_t rxQueue) {

    struct can_frame canMsg2;
    MCP__ERROR readRet = MCP__ERROR_FAIL;

    uint8_t val = 0x00;
    int spiRet = MCP____startSPIFromISR();
    if ( spiRet != 0 ) {
        return spiRet;
    }
    MCP____readRegisters(MCP_CANINTF, &val, 1);
    
    if ((val & MCP__STAT_RX0IF)) {
        //   ESP_LOGI(TAG, "Interrupt val: %x %x", val, MCP____readRegister(MCP_CANINTF));
        readRet = MCP____readMessageFromBuffer(RXB0, &canMsg2);
        // bufferNum = 0;
    } else if ((val & MCP__STAT_RX1IF)) {
        readRet = MCP____readMessageFromBuffer(RXB1, &canMsg2);
        // bufferNum = 1;
    } else {
        spiRet = MCP____endSPIFromISR();
        if ( spiRet != 0 ) {
            return spiRet;
        }
        return -101;
    }
    spiRet = MCP____endSPIFromISR();
    if ( spiRet != 0 ) {
        return spiRet;
    }
    if (readRet == MCP__ERROR_OK) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        BaseType_t retSend = xQueueSendFromISR(rxQueue, (void *) &canMsg2, &xHigherPriorityTaskWoken);
        if ( retSend == pdFALSE ) {
            return -2;
        } else {
            return 0;
        }
    } else {
        return -5;
    }
}

uint8_t MCP____readRegister(const MCP__REGISTER reg)
{
    uint8_t ret = 0xFF;
    MCP____readRegisters(reg, &ret, 1);

    return ret;
}

void MCP____readRegisters(const MCP__REGISTER reg, uint8_t values[], const uint8_t n)
{
    esp_err_t ret;
    spi_transaction_t t = {
        .flags = SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_CMD,
        .cmd = INSTRUCTION_READ,
        .addr = reg,
        .length = 8 * n,
        .rxlength = 8 * n,
        .user = NULL,
        .tx_buffer = values,
        .rx_buffer = values,
    };
    spi_transaction_ext_t spiext_t = {
        .base = t,
        .command_bits = 8,
        .address_bits = 8,
        .dummy_bits = 0
    };
    // ESP_LOGI(TAG, "SPIEXT_T in readRegisters: %" PRId32 " %" PRId16" %" PRId8, spiext_t.base.flags, spiext_t.base.cmd, spiext_t.command_bits);
    ret = spi_device_polling_transmit(spi, (spi_transaction_t *) &spiext_t);  //Transmit!

    // vPortFree(refVal);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to tranmit to SPI : %d", ret);
    }
    assert(ret==ESP_OK);
}

void MCP____setRegister(const MCP__REGISTER reg, const uint8_t value)
{
    MCP____setRegisters(reg, &value, 1);
}

void MCP____setRegisters(const MCP__REGISTER reg, const uint8_t values[], const uint8_t n)
{
    esp_err_t ret;
    spi_transaction_t t = {
        .flags = SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_CMD,
        .cmd = INSTRUCTION_WRITE,
        .addr = reg,
        .length = n * 8,
        .rxlength = 0,
        .user = NULL,
        .tx_buffer = values,
        .rx_buffer = NULL
    };
    spi_transaction_ext_t spiext_t = {
        .base = t,
        .command_bits = 8,
        .address_bits = 8,
        .dummy_bits = 0
    };
    ret=spi_device_polling_transmit(spi, (spi_transaction_t *) &spiext_t);  //Transmit!
    assert(ret==ESP_OK);
}

void MCP____modifyRegister(const MCP__REGISTER reg, const uint8_t mask, const uint8_t data)
{
    esp_err_t ret;
    spi_transaction_t t = {
        .flags = SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_CMD | SPI_TRANS_USE_TXDATA,
        .cmd = INSTRUCTION_BITMOD,
        .addr = reg,
        .length = 2 * 8,
        .rxlength = 0,
        .user = NULL,
        .tx_data = {mask, data},
        .rx_buffer = NULL
    };
    spi_transaction_ext_t spiext_t = {
        .base = t,
        .command_bits = 8,
        .address_bits = 8,
        .dummy_bits = 0
    };
    ret=spi_device_polling_transmit(spi, (spi_transaction_t *) &spiext_t);  //Transmit!
    assert(ret==ESP_OK);
}

uint8_t MCP____getStatus(void)
{
    MCP____startSPI();
    // spi_transfer(INSTRUCTION_READ_STATUS);
    // uint8_t i = spi_transfer(0x00);

    esp_err_t ret;
    spi_transaction_t t = {
        .flags = SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_DUMMY
            | SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
        .cmd = INSTRUCTION_READ_STATUS,
        .addr = 0,
        .length = 8,
        .rxlength = 8,
        .user = NULL,
        .tx_data = {0x00}
    };
    spi_transaction_ext_t spiext_t = {
        .base = t,
        .command_bits = 8,
        .address_bits = 0,
        .dummy_bits = 0
    };
    ret = spi_device_polling_transmit(spi, (spi_transaction_t *) &spiext_t);  //Transmit!
    assert(ret==ESP_OK);

    MCP____endSPI();

    return spiext_t.base.rx_data[0];
}

MCP__ERROR MCP____setConfigMode()
{
    return MCP____setMode(CANCTRL_REQOP_CONFIG);
}

MCP__ERROR MCP____setListenOnlyMode()
{
    return MCP____setMode(CANCTRL_REQOP_LISTENONLY);
}

MCP__ERROR MCP____setSleepMode()
{
    return MCP____setMode(CANCTRL_REQOP_SLEEP);
}

MCP__ERROR MCP____setLoopbackMode()
{
    return MCP____setMode(CANCTRL_REQOP_LOOPBACK);
}

MCP__ERROR MCP____setNormalMode()
{
    return MCP____setMode(CANCTRL_REQOP_NORMAL);
}

MCP__ERROR MCP____setMode(const CANCTRL_REQOP_MODE mode)
{
    MCP____startSPI();
    MCP____modifyRegister(MCP_CANCTRL, CANCTRL_REQOP, mode);

    int64_t endTime = esp_timer_get_time() + (10 * 1000);

    bool modeMatch = false;
    uint8_t newmode = 0xAA;
    while (esp_timer_get_time() < endTime) {
        // ESP_LOGI(TAG, "Starting read reg");
        newmode = MCP____readRegister(MCP_CANSTAT);
        newmode &= CANSTAT_OPMOD;

        modeMatch = newmode == mode;

        if (modeMatch) {
            break;
        }
        // ESP_LOGI(TAG, "Current Time: %" PRIi64, esp_timer_get_time());
        // vTaskDelay(pdMS_TO_TICKS(1));
    }
    MCP____endSPI();
    // ESP_LOGI(TAG, "%s %x %x", modeMatch ? "SET MODE OK" : "SET MOde FAIL", mode, newmode);
    return modeMatch ? MCP__ERROR_OK : MCP__ERROR_FAIL;

}

MCP__ERROR MCP____setBitrate(const MCP__CAN_SPEED canSpeed, CAN_CLOCK canClock)
{
    MCP__ERROR error = MCP____setConfigMode();
    if (error != MCP__ERROR_OK) {
        return error;
    }

    uint8_t set, cfg1, cfg2, cfg3;
    set = 1;
    switch (canClock)
    {
        case (MCP_8MHZ):
        switch (canSpeed)
        {
            case (MCP__CAN_5KBPS):                                               //   5KBPS
            cfg1 = MCP_8MHz_5kBPS_CFG1;
            cfg2 = MCP_8MHz_5kBPS_CFG2;
            cfg3 = MCP_8MHz_5kBPS_CFG3;
            break;

            case (MCP__CAN_10KBPS):                                              //  10KBPS
            cfg1 = MCP_8MHz_10kBPS_CFG1;
            cfg2 = MCP_8MHz_10kBPS_CFG2;
            cfg3 = MCP_8MHz_10kBPS_CFG3;
            break;

            case (MCP__CAN_20KBPS):                                              //  20KBPS
            cfg1 = MCP_8MHz_20kBPS_CFG1;
            cfg2 = MCP_8MHz_20kBPS_CFG2;
            cfg3 = MCP_8MHz_20kBPS_CFG3;
            break;

            case (MCP__CAN_31K25BPS):                                            //  31.25KBPS
            cfg1 = MCP_8MHz_31k25BPS_CFG1;
            cfg2 = MCP_8MHz_31k25BPS_CFG2;
            cfg3 = MCP_8MHz_31k25BPS_CFG3;
            break;

            case (MCP__CAN_33KBPS):                                              //  33.333KBPS
            cfg1 = MCP_8MHz_33k3BPS_CFG1;
            cfg2 = MCP_8MHz_33k3BPS_CFG2;
            cfg3 = MCP_8MHz_33k3BPS_CFG3;
            break;

            case (MCP__CAN_40KBPS):                                              //  40Kbps
            cfg1 = MCP_8MHz_40kBPS_CFG1;
            cfg2 = MCP_8MHz_40kBPS_CFG2;
            cfg3 = MCP_8MHz_40kBPS_CFG3;
            break;

            case (MCP__CAN_50KBPS):                                              //  50Kbps
            cfg1 = MCP_8MHz_50kBPS_CFG1;
            cfg2 = MCP_8MHz_50kBPS_CFG2;
            cfg3 = MCP_8MHz_50kBPS_CFG3;
            break;

            case (MCP__CAN_80KBPS):                                              //  80Kbps
            cfg1 = MCP_8MHz_80kBPS_CFG1;
            cfg2 = MCP_8MHz_80kBPS_CFG2;
            cfg3 = MCP_8MHz_80kBPS_CFG3;
            break;

            case (MCP__CAN_100KBPS):                                             // 100Kbps
            cfg1 = MCP_8MHz_100kBPS_CFG1;
            cfg2 = MCP_8MHz_100kBPS_CFG2;
            cfg3 = MCP_8MHz_100kBPS_CFG3;
            break;

            case (MCP__CAN_125KBPS):                                             // 125Kbps
            cfg1 = MCP_8MHz_125kBPS_CFG1;
            cfg2 = MCP_8MHz_125kBPS_CFG2;
            cfg3 = MCP_8MHz_125kBPS_CFG3;
            break;

            case (MCP__CAN_200KBPS):                                             // 200Kbps
            cfg1 = MCP_8MHz_200kBPS_CFG1;
            cfg2 = MCP_8MHz_200kBPS_CFG2;
            cfg3 = MCP_8MHz_200kBPS_CFG3;
            break;

            case (MCP__CAN_250KBPS):                                             // 250Kbps
            cfg1 = MCP_8MHz_250kBPS_CFG1;
            cfg2 = MCP_8MHz_250kBPS_CFG2;
            cfg3 = MCP_8MHz_250kBPS_CFG3;
            break;

            case (MCP__CAN_500KBPS):                                             // 500Kbps
            cfg1 = MCP_8MHz_500kBPS_CFG1;
            cfg2 = MCP_8MHz_500kBPS_CFG2;
            cfg3 = MCP_8MHz_500kBPS_CFG3;
            break;

            case (MCP__CAN_1000KBPS):                                            //   1Mbps
            cfg1 = MCP_8MHz_1000kBPS_CFG1;
            cfg2 = MCP_8MHz_1000kBPS_CFG2;
            cfg3 = MCP_8MHz_1000kBPS_CFG3;
            break;

            default:
            set = 0;
            break;
        }
        break;

        case (MCP_10MHZ):
        switch (canSpeed)
        {
            case (MCP__CAN_5KBPS):                                               //   5KBPS
            cfg1 = MCP_10MHz_5kBPS_CFG1;
            cfg2 = MCP_10MHz_5kBPS_CFG2;
            cfg3 = MCP_10MHz_5kBPS_CFG3;
            break;

            case (MCP__CAN_10KBPS):                                              //  10KBPS
            cfg1 = MCP_10MHz_10kBPS_CFG1;
            cfg2 = MCP_10MHz_10kBPS_CFG2;
            cfg3 = MCP_10MHz_10kBPS_CFG3;
            break;

            case (MCP__CAN_20KBPS):                                              //  20KBPS
            cfg1 = MCP_10MHz_20kBPS_CFG1;
            cfg2 = MCP_10MHz_20kBPS_CFG2;
            cfg3 = MCP_10MHz_20kBPS_CFG3;
            break;

            case (MCP__CAN_31K25BPS):                                            //  31.25KBPS
            cfg1 = MCP_10MHz_31k25BPS_CFG1;
            cfg2 = MCP_10MHz_31k25BPS_CFG2;
            cfg3 = MCP_10MHz_31k25BPS_CFG3;
            break;

            case (MCP__CAN_33KBPS):                                              //  33.333KBPS
            cfg1 = MCP_10MHz_33k3BPS_CFG1;
            cfg2 = MCP_10MHz_33k3BPS_CFG2;
            cfg3 = MCP_10MHz_33k3BPS_CFG3;
            break;

            case (MCP__CAN_40KBPS):                                              //  40Kbps
            cfg1 = MCP_10MHz_40kBPS_CFG1;
            cfg2 = MCP_10MHz_40kBPS_CFG2;
            cfg3 = MCP_10MHz_40kBPS_CFG3;
            break;

            case (MCP__CAN_50KBPS):                                              //  50Kbps
            cfg1 = MCP_10MHz_50kBPS_CFG1;
            cfg2 = MCP_10MHz_50kBPS_CFG2;
            cfg3 = MCP_10MHz_50kBPS_CFG3;
            break;

            case (MCP__CAN_80KBPS):                                              //  80Kbps
            cfg1 = MCP_10MHz_80kBPS_CFG1;
            cfg2 = MCP_10MHz_80kBPS_CFG2;
            cfg3 = MCP_10MHz_80kBPS_CFG3;
            break;

            case (MCP__CAN_100KBPS):                                             // 100Kbps
            cfg1 = MCP_10MHz_100kBPS_CFG1;
            cfg2 = MCP_10MHz_100kBPS_CFG2;
            cfg3 = MCP_10MHz_100kBPS_CFG3;
            break;

            case (MCP__CAN_125KBPS):                                             // 125Kbps
            cfg1 = MCP_10MHz_125kBPS_CFG1;
            cfg2 = MCP_10MHz_125kBPS_CFG2;
            cfg3 = MCP_10MHz_125kBPS_CFG3;
            break;

            case (MCP__CAN_200KBPS):                                             // 200Kbps
            cfg1 = MCP_10MHz_200kBPS_CFG1;
            cfg2 = MCP_10MHz_200kBPS_CFG2;
            cfg3 = MCP_10MHz_200kBPS_CFG3;
            break;

            case (MCP__CAN_250KBPS):                                             // 250Kbps
            cfg1 = MCP_10MHz_250kBPS_CFG1;
            cfg2 = MCP_10MHz_250kBPS_CFG2;
            cfg3 = MCP_10MHz_250kBPS_CFG3;
            break;

            case (MCP__CAN_500KBPS):                                             // 500Kbps
            cfg1 = MCP_10MHz_500kBPS_CFG1;
            cfg2 = MCP_10MHz_500kBPS_CFG2;
            cfg3 = MCP_10MHz_500kBPS_CFG3;
            break;

            case (MCP__CAN_1000KBPS):                                            //   1Mbps
            cfg1 = MCP_10MHz_1000kBPS_CFG1;
            cfg2 = MCP_10MHz_1000kBPS_CFG2;
            cfg3 = MCP_10MHz_1000kBPS_CFG3;
            break;

            default:
            set = 0;
            break;
        }
        break;

        case (MCP_16MHZ):
        switch (canSpeed)
        {
            case (MCP__CAN_5KBPS):                                               //   5Kbps
            cfg1 = MCP_16MHz_5kBPS_CFG1;
            cfg2 = MCP_16MHz_5kBPS_CFG2;
            cfg3 = MCP_16MHz_5kBPS_CFG3;
            break;

            case (MCP__CAN_10KBPS):                                              //  10Kbps
            cfg1 = MCP_16MHz_10kBPS_CFG1;
            cfg2 = MCP_16MHz_10kBPS_CFG2;
            cfg3 = MCP_16MHz_10kBPS_CFG3;
            break;

            case (MCP__CAN_20KBPS):                                              //  20Kbps
            cfg1 = MCP_16MHz_20kBPS_CFG1;
            cfg2 = MCP_16MHz_20kBPS_CFG2;
            cfg3 = MCP_16MHz_20kBPS_CFG3;
            break;

            case (MCP__CAN_33KBPS):                                              //  33.333Kbps
            cfg1 = MCP_16MHz_33k3BPS_CFG1;
            cfg2 = MCP_16MHz_33k3BPS_CFG2;
            cfg3 = MCP_16MHz_33k3BPS_CFG3;
            break;

            case (MCP__CAN_40KBPS):                                              //  40Kbps
            cfg1 = MCP_16MHz_40kBPS_CFG1;
            cfg2 = MCP_16MHz_40kBPS_CFG2;
            cfg3 = MCP_16MHz_40kBPS_CFG3;
            break;

            case (MCP__CAN_50KBPS):                                              //  50Kbps
            cfg1 = MCP_16MHz_50kBPS_CFG1;
            cfg2 = MCP_16MHz_50kBPS_CFG2;
            cfg3 = MCP_16MHz_50kBPS_CFG3;
            break;

            case (MCP__CAN_80KBPS):                                              //  80Kbps
            cfg1 = MCP_16MHz_80kBPS_CFG1;
            cfg2 = MCP_16MHz_80kBPS_CFG2;
            cfg3 = MCP_16MHz_80kBPS_CFG3;
            break;

            case (MCP__CAN_83K3BPS):                                             //  83.333Kbps
            cfg1 = MCP_16MHz_83k3BPS_CFG1;
            cfg2 = MCP_16MHz_83k3BPS_CFG2;
            cfg3 = MCP_16MHz_83k3BPS_CFG3;
            break; 

            case (MCP__CAN_100KBPS):                                             // 100Kbps
            cfg1 = MCP_16MHz_100kBPS_CFG1;
            cfg2 = MCP_16MHz_100kBPS_CFG2;
            cfg3 = MCP_16MHz_100kBPS_CFG3;
            break;

            case (MCP__CAN_125KBPS):                                             // 125Kbps
            cfg1 = MCP_16MHz_125kBPS_CFG1;
            cfg2 = MCP_16MHz_125kBPS_CFG2;
            cfg3 = MCP_16MHz_125kBPS_CFG3;
            break;

            case (MCP__CAN_200KBPS):                                             // 200Kbps
            cfg1 = MCP_16MHz_200kBPS_CFG1;
            cfg2 = MCP_16MHz_200kBPS_CFG2;
            cfg3 = MCP_16MHz_200kBPS_CFG3;
            break;

            case (MCP__CAN_250KBPS):                                             // 250Kbps
            cfg1 = MCP_16MHz_250kBPS_CFG1;
            cfg2 = MCP_16MHz_250kBPS_CFG2;
            cfg3 = MCP_16MHz_250kBPS_CFG3;
            break;

            case (MCP__CAN_500KBPS):                                             // 500Kbps
            cfg1 = MCP_16MHz_500kBPS_CFG1;
            cfg2 = MCP_16MHz_500kBPS_CFG2;
            cfg3 = MCP_16MHz_500kBPS_CFG3;
            break;

            case (MCP__CAN_1000KBPS):                                            //   1Mbps
            cfg1 = MCP_16MHz_1000kBPS_CFG1;
            cfg2 = MCP_16MHz_1000kBPS_CFG2;
            cfg3 = MCP_16MHz_1000kBPS_CFG3;
            break;

            default:
            set = 0;
            break;
        }
        break;

        case (MCP_20MHZ):
        switch (canSpeed)
        {
            case (MCP__CAN_33KBPS):                                              //  33.333Kbps
            cfg1 = MCP_20MHz_33k3BPS_CFG1;
            cfg2 = MCP_20MHz_33k3BPS_CFG2;
            cfg3 = MCP_20MHz_33k3BPS_CFG3;
	    break;

            case (MCP__CAN_40KBPS):                                              //  40Kbps
            cfg1 = MCP_20MHz_40kBPS_CFG1;
            cfg2 = MCP_20MHz_40kBPS_CFG2;
            cfg3 = MCP_20MHz_40kBPS_CFG3;
            break;

            case (MCP__CAN_50KBPS):                                              //  50Kbps
            cfg1 = MCP_20MHz_50kBPS_CFG1;
            cfg2 = MCP_20MHz_50kBPS_CFG2;
            cfg3 = MCP_20MHz_50kBPS_CFG3;
            break;

            case (MCP__CAN_80KBPS):                                              //  80Kbps
            cfg1 = MCP_20MHz_80kBPS_CFG1;
            cfg2 = MCP_20MHz_80kBPS_CFG2;
            cfg3 = MCP_20MHz_80kBPS_CFG3;
            break;

            case (MCP__CAN_83K3BPS):                                             //  83.333Kbps
            cfg1 = MCP_20MHz_83k3BPS_CFG1;
            cfg2 = MCP_20MHz_83k3BPS_CFG2;
            cfg3 = MCP_20MHz_83k3BPS_CFG3;
	    break;

            case (MCP__CAN_100KBPS):                                             // 100Kbps
            cfg1 = MCP_20MHz_100kBPS_CFG1;
            cfg2 = MCP_20MHz_100kBPS_CFG2;
            cfg3 = MCP_20MHz_100kBPS_CFG3;
            break;

            case (MCP__CAN_125KBPS):                                             // 125Kbps
            cfg1 = MCP_20MHz_125kBPS_CFG1;
            cfg2 = MCP_20MHz_125kBPS_CFG2;
            cfg3 = MCP_20MHz_125kBPS_CFG3;
            break;

            case (MCP__CAN_200KBPS):                                             // 200Kbps
            cfg1 = MCP_20MHz_200kBPS_CFG1;
            cfg2 = MCP_20MHz_200kBPS_CFG2;
            cfg3 = MCP_20MHz_200kBPS_CFG3;
            break;

            case (MCP__CAN_250KBPS):                                             // 250Kbps
            cfg1 = MCP_20MHz_250kBPS_CFG1;
            cfg2 = MCP_20MHz_250kBPS_CFG2;
            cfg3 = MCP_20MHz_250kBPS_CFG3;
            break;

            case (MCP__CAN_500KBPS):                                             // 500Kbps
            cfg1 = MCP_20MHz_500kBPS_CFG1;
            cfg2 = MCP_20MHz_500kBPS_CFG2;
            cfg3 = MCP_20MHz_500kBPS_CFG3;
            break;

            case (MCP__CAN_1000KBPS):                                            //   1Mbps
            cfg1 = MCP_20MHz_1000kBPS_CFG1;
            cfg2 = MCP_20MHz_1000kBPS_CFG2;
            cfg3 = MCP_20MHz_1000kBPS_CFG3;
            break;

            default:
            set = 0;
            break;
        }
        break;

        default:
        set = 0;
        break;
    }

    if (set) {
        MCP____startSPI();
        MCP____setRegister(MCP_CNF1, cfg1);
        MCP____setRegister(MCP_CNF2, cfg2);
        MCP____setRegister(MCP_CNF3, cfg3);
        MCP____endSPI();
        return MCP__ERROR_OK;
    }
    else {
        return MCP__ERROR_FAIL;
    }
}

MCP__ERROR MCP____setClkOut(const MCP__CAN_CLKOUT divisor)
{
    if (divisor == MCP__CLKOUT_DISABLE) {
	    /* Turn off CLKEN */
        MCP____startSPI();
        MCP____modifyRegister(MCP_CANCTRL, CANCTRL_CLKEN, 0x00);

        /* Turn on CLKOUT for SOF */
        MCP____modifyRegister(MCP_CNF3, CNF3_SOF, CNF3_SOF);
        MCP____endSPI();
        return MCP__ERROR_OK;
    }

    /* Set the prescaler (CLKPRE) */
    
    MCP____startSPI();
    MCP____modifyRegister(MCP_CANCTRL, CANCTRL_CLKPRE, divisor);

    /* Turn on CLKEN */
    MCP____modifyRegister(MCP_CANCTRL, CANCTRL_CLKEN, CANCTRL_CLKEN);

    /* Turn off CLKOUT for SOF */
    MCP____modifyRegister(MCP_CNF3, CNF3_SOF, 0x00);
    MCP____endSPI();
    return MCP__ERROR_OK;
}

void MCP____prepareId(uint8_t *buffer, const bool ext, const uint32_t id)
{
    uint16_t canid = (uint16_t)(id & 0x0FFFF);

    if (ext) {
        buffer[MCP_EID0] = (uint8_t) (canid & 0xFF); // LSB Lower byte
        buffer[MCP_EID8] = (uint8_t) (canid >> 8); // LSB upper byte
        canid = (uint16_t)(id >> 16);
        buffer[MCP_SIDL] = (uint8_t) (canid & 0x03); // MSB Lower byte's Lower [1:0] 2 bits
        buffer[MCP_SIDL] += (uint8_t) ((canid & 0x1C) << 3);  // MSB Lower byte's Lower [5:2] 3 bits
        buffer[MCP_SIDL] |= TXB_EXIDE_MASK;
        buffer[MCP_SIDH] = (uint8_t) (canid >> 5); // MSB upper 8 bits for extid
    } else {
        buffer[MCP_SIDH] = (uint8_t) (canid >> 3);
        buffer[MCP_SIDL] = (uint8_t) ((canid & 0x07 ) << 5);
        buffer[MCP_EID8] = 0;
        buffer[MCP_EID0] = 0;
    }
}

MCP__ERROR MCP____setFilterMask(const MCP__MASK mask, const bool ext, const uint32_t ulData)
{
    MCP__ERROR res = MCP____setConfigMode();
    if (res != MCP__ERROR_OK) {
        return res;
    }
    
    uint8_t tbufdata[4];
    MCP____prepareId(tbufdata, ext, ulData);

    MCP__REGISTER reg;
    switch (mask) {
        case MCP__MASK0: reg = MCP_RXM0SIDH; break;
        case MCP__MASK1: reg = MCP_RXM1SIDH; break;
        default:
            return MCP__ERROR_FAIL;
    }

    MCP____startSPI();
    MCP____setRegisters(reg, tbufdata, 4);
    MCP____endSPI();
    
    return MCP__ERROR_OK;
}

MCP__ERROR MCP____setFilter(const RXF num, const bool ext, const uint32_t ulData)
{
    MCP__ERROR res = MCP____setConfigMode();
    if (res != MCP__ERROR_OK) {
        return res;
    }

    MCP__REGISTER reg;

    switch (num) {
        case RXF0: reg = MCP_RXF0SIDH; break;
        case RXF1: reg = MCP_RXF1SIDH; break;
        case RXF2: reg = MCP_RXF2SIDH; break;
        case RXF3: reg = MCP_RXF3SIDH; break;
        case RXF4: reg = MCP_RXF4SIDH; break;
        case RXF5: reg = MCP_RXF5SIDH; break;
        default:
            return MCP__ERROR_FAIL;
    }

    uint8_t tbufdata[4];
    MCP____prepareId(tbufdata, ext, ulData);
    MCP____startSPI();
    MCP____setRegisters(reg, tbufdata, 4);
    MCP____endSPI();

    return MCP__ERROR_OK;
}

MCP__ERROR MCP____sendMessageUsingBuffer(const TXBn txbn, const struct can_frame *frame)
{
    if (frame->can_dlc > CAN_MAX_DLEN) {
        return MCP__ERROR_FAILTX;
    }

    const struct TXBn_REGS *txbuf = &TXB[txbn];

    uint8_t data[13];

    bool ext = (frame->can_id & CAN_EFF_FLAG);
    bool rtr = (frame->can_id & CAN_RTR_FLAG);
    uint32_t id = (frame->can_id & (ext ? CAN_EFF_MASK : CAN_SFF_MASK));

    MCP____prepareId(data, ext, id);

    data[MCP_DLC] = rtr ? (frame->can_dlc | RTR_MASK) : frame->can_dlc;

    memcpy(&data[MCP_DATA], frame->data, frame->can_dlc);

    MCP____setRegisters(txbuf->SIDH, data, 5 + frame->can_dlc);
    MCP____modifyRegister(txbuf->CTRL, TXB_TXREQ, TXB_TXREQ);
    uint8_t ctrl = MCP____readRegister(txbuf->CTRL);
    if ((ctrl & (TXB_ABTF | TXB_MLOA | TXB_TXERR)) != 0) {
        return MCP__ERROR_FAILTX;
    }
    return MCP__ERROR_OK;
}

MCP__ERROR MCP____sendMessage(const struct can_frame *frame)
{
    if (frame->can_dlc > CAN_MAX_DLEN) {
        return MCP__ERROR_FAILTX;
    }

    TXBn txBuffers[N_TXBUFFERS] = {TXB0, TXB1, TXB2};

    for (int i=0; i<N_TXBUFFERS; i++) {
        const struct TXBn_REGS *txbuf = &TXB[txBuffers[i]];
        MCP____startSPI();
        uint8_t ctrlval = MCP____readRegister(txbuf->CTRL);
        MCP____endSPI();
        if ( (ctrlval & TXB_TXREQ) == 0 ) {
            MCP____startSPI();
            MCP__ERROR sendMsgResp = MCP____sendMessageUsingBuffer(txBuffers[i], frame);
            MCP____endSPI();
            return sendMsgResp;
        }
    }

    return MCP__ERROR_ALLTXBUSY;
}

MCP__ERROR MCP____readMessageFromBuffer(const RXBn rxbn, struct can_frame *frame)
{
    const struct RXBn_REGS *rxb = &RXB[rxbn];

    uint8_t tbufdata[5];

    MCP____readRegisters(rxb->SIDH, tbufdata, 5);

    uint32_t id = (tbufdata[MCP_SIDH]<<3) + (tbufdata[MCP_SIDL]>>5);

    if ( (tbufdata[MCP_SIDL] & TXB_EXIDE_MASK) ==  TXB_EXIDE_MASK ) {
        id = (id<<2) + (tbufdata[MCP_SIDL] & 0x03);
        id = (id<<8) + tbufdata[MCP_EID8];
        id = (id<<8) + tbufdata[MCP_EID0];
        id |= CAN_EFF_FLAG;
    }

    uint8_t dlc = (tbufdata[MCP_DLC] & DLC_MASK);
    if (dlc > CAN_MAX_DLEN) {
        return MCP__ERROR_FAIL;
    }

    uint8_t ctrl = MCP____readRegister(rxb->CTRL);
    if (ctrl & RXBnCTRL_RTR) {
        id |= CAN_RTR_FLAG;
    }

    frame->can_id = id;
    frame->can_dlc = dlc;

    MCP____readRegisters(rxb->DATA, frame->data, dlc);
    MCP____modifyRegister(MCP_CANINTF, rxb->CANINTF_RXnIF, 0);

    return MCP__ERROR_OK;
}

MCP__ERROR MCP____readMessage(struct can_frame *frame)
{
    MCP__ERROR rc;
    uint8_t stat = MCP____getStatus();

    MCP____startSPI();
    if ( stat & MCP__STAT_RX0IF ) {
        rc = MCP____readMessageFromBuffer(RXB0, frame);
    } else if ( stat & MCP__STAT_RX1IF ) {
        rc = MCP____readMessageFromBuffer(RXB1, frame);
    } else {
        rc = MCP__ERROR_NOMSG;
    }
    MCP____endSPI();

    return rc;
}

// MCP__ERROR MCP____readMessageFrom(struct can_frame *frame, uint8_t *bufferNum)
// {
//     MCP__ERROR rc;
//     uint8_t stat = MCP____getStatus();

//     if ( stat & MCP__STAT_RX0IF ) {
//         rc = MCP____readMessageFromBuffer(RXB0, frame);
//     } else if ( stat & MCP__STAT_RX1IF ) {
//         rc = MCP____readMessageFromBuffer(RXB1, frame);
//     } else {
//         rc = MCP__ERROR_NOMSG;
//     }

//     return rc;
// }

bool MCP____checkReceive(void)
{
    uint8_t res = MCP____getStatus();
    if ( res & STAT_RXIF_MASK ) {
        return true;
    } else {
        return false;
    }
}

bool MCP____checkError(void)
{
    uint8_t eflg = MCP____getErrorFlags();

    if ( eflg & EFLG_ERRORMASK ) {
        return true;
    } else {
        return false;
    }
}

uint8_t MCP____getErrorFlags(void)
{
    MCP____startSPI();
    uint8_t ret = MCP____readRegister(MCP_EFLG);
    MCP____endSPI();
    return ret;
}

void MCP____clearRXnOVRFlags(void)
{
    MCP____startSPI();
	MCP____modifyRegister(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
    MCP____endSPI();
}

uint8_t MCP____getInterrupts(void)
{
    MCP____startSPI();
    uint8_t ret = MCP____readRegister(MCP_CANINTF);
    MCP____endSPI();
    return ret;
}

void MCP____clearInterrupts(void)
{
    MCP____startSPI();
    MCP____setRegister(MCP_CANINTF, 0);
    MCP____endSPI();
}

uint8_t MCP____getInterruptMask(void)
{
    MCP____startSPI();
    uint8_t ret = MCP____readRegister(MCP_CANINTE);
    MCP____endSPI();
    return ret;
}

void MCP____clearTXInterrupts(void)
{
    MCP____startSPI();
    MCP____modifyRegister(MCP_CANINTF, (CANINTF_TX0IF | CANINTF_TX1IF | CANINTF_TX2IF), 0);
    MCP____endSPI();
}

void MCP____clearRXnOVR(void)
{
	uint8_t eflg = MCP____getErrorFlags();
	if (eflg != 0) {
		MCP____clearRXnOVRFlags();
		MCP____clearInterrupts();
		//modifyRegister(MCP_CANINTF, CANINTF_ERRIF, 0);
	}
	
}

void MCP____clearMERR()
{
    MCP____startSPI();
	MCP____modifyRegister(MCP_CANINTF, CANINTF_MERRF, 0);
    MCP____endSPI();
}

void MCP____clearERRIF()
{
    //modifyRegister(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
    //clearInterrupts();
    MCP____startSPI();
    MCP____modifyRegister(MCP_CANINTF, CANINTF_ERRIF, 0);
    MCP____endSPI();
}

uint8_t MCP____errorCountRX(void)                             
{
    MCP____startSPI();
    uint8_t ret = MCP____readRegister(MCP_REC);
    MCP____endSPI();
    return ret;
}

uint8_t MCP____errorCountTX(void)                             
{
    MCP____startSPI();
    uint8_t ret = MCP____readRegister(MCP_TEC);
    MCP____endSPI();
    return ret;
}
