#include "sock_service.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "tcp_packet_handler.h"
#include "iso_tp.h"
#include "instructions.h"
#include "dev_settings.h"
#include "channel.h"
#include "canbus.h"
#include "can2bus.h"
#include "utility.h"

//#include "usb_service.h"
#include "klinebus.h"

#define PORT 6888
#define KEEPALIVE_IDLE 5
#define KEEPALIVE_INTERVAL 5
#define KEEPALIVE_COUNT 3

static const char *TAG = "sock_service";

#define LOCAL_MIN(a, b) (a < b ? a : b)

#define SOCK_EVENT_SOCKET_CLOSED (1 << 0)
#define SOCK_EVENT_SOCKET_RX_TASK_CLOSED (1 << 1)
#define SOCK_EVENT_SOCKET_TX_TASK_CLOSED (1 << 2)

typedef struct
{
    int sock;
    EventGroupHandle_t eventGroupHandle;
} SocketRunHandle;

// #define SOCKET_RX_CACHE_SIZE (4096 + 10)
// #define SOCKET_RX_BUFFER_SIZE (4096 + 10 + 512 /* Extra buffer */)

// typedef struct
// {
//     uint8_t tcpRxBuffer[SOCKET_RX_BUFFER_SIZE];
//     uint16_t head;
//     uint16_t tail;
//     bool isOverLapped;
// } SockRxData;

bool appcomm;
uint8_t mac[6] = {0, 0, 0, 0, 0, 0};

uint16_t SockRxData_usedBuffer(const SockRxData *s)
{
    uint16_t used = s->tail <= s->head ? s->head - s->tail : sizeof(s->tcpRxBuffer) - (s->tail - s->head);
    // ESP_LOGI(TAG, "UsedBuffer: %d, tail: %d, head: %d", used, s->tail, s->head);
    if (used == 0 && s->isOverLapped)
    {
        return sizeof(s->tcpRxBuffer);
    }
    return used;
}
uint16_t SockRxData_remainingBuffer(const SockRxData *s)
{
    uint16_t used = SockRxData_usedBuffer(s);
    uint16_t free = sizeof(s->tcpRxBuffer) - used;
    // ESP_LOGI(TAG, "REMAINING BUFF: %d, used: %d", free, used);
    return free;
}
uint16_t SockRxData_remainingContinuousLen(const SockRxData *s)
{
    uint16_t used = SockRxData_usedBuffer(s);
    uint16_t free = sizeof(s->tcpRxBuffer) - used;
    // ESP_LOGI(TAG, "REMAINING BUFF: %d, used: %d", free, used);
    uint16_t continuousSpace = LOCAL_MIN(free, sizeof(s->tcpRxBuffer) - s->head);
    return continuousSpace;
}
void SockRxData_addLen(SockRxData *s, uint16_t len)
{
    s->head = (s->head + len) % sizeof(s->tcpRxBuffer);
    if (s->head == s->tail)
    {
        s->isOverLapped = true;
    }
}
void SockRxData_releaseLen(SockRxData *s, uint16_t len)
{
    s->tail = (s->tail + len) % sizeof(s->tcpRxBuffer);
    // ESP_LOGI(TAG, "TAIL VAL: %d LEN %d", s->tail, len);
    if (s->tail == s->head)
    {
        s->isOverLapped = false;
    }
}
uint8_t *SockRxData_getHeadPtr(SockRxData *s)
{
    return &s->tcpRxBuffer[s->head];
}
void SockRxData_fillBuffer(SockRxData *s, uint8_t *buffer, uint16_t len)
{
    uint16_t lenForward = LOCAL_MIN(len, sizeof(s->tcpRxBuffer) - s->tail);
    memcpy(buffer, &s->tcpRxBuffer[s->tail], lenForward);
    if (len > lenForward)
    {
        uint16_t remLen = len - lenForward;
        memcpy(&buffer[lenForward], &s->tcpRxBuffer[0], remLen);
    }
}

static SockRxData sockRxData;

//void processAcquiredData(SockRxData *s, uint8_t *rxPktBuffer, uint16_t rxPktSize)
void processAcquiredData(uint8_t *rxPktBuffer, uint16_t rxPktSize)
{

    uint16_t canDataLen = 0;
    // uint32_t sizeToAllocate = rxPktSize + 2 /*code + len*/ + 1 /*channel num*/
    uint8_t *canData = NULL;
    // canData = (uint8_t *)pvPortMalloc(4096);
    // uint8_t *canData = (uint8_t *)pvPortMalloc(4096);
   
    //  if (canData == NULL) {
    //     ESP_LOGW(TAG, "Failed to allocate buffer of : %d", 4096);
    // } else 
    // if (canData != NULL)
    {
        int retCode = ERR_CUSTOM_PACKET_INTERNAL_ERROR;
        uint16_t lenToRelease = 0;
        uint16_t indexToTest = 0;
        uint16_t processedBytes = 0;
        uint8_t startNibble = 0;
        uint8_t channelNum = 0;
        while (indexToTest < rxPktSize)
        {
            retCode = processCustomPacketIntoCANData(&rxPktBuffer[indexToTest],
                        rxPktSize - indexToTest, &canData, &canDataLen, &processedBytes,
                        &startNibble, &channelNum);
            if (retCode == CUSTOM_PACKET_OK)
            {
                ESP_LOGD(TAG, "CUSTOM PACKET OK");
                lenToRelease = processedBytes + indexToTest;
                break;
            }
            else if (retCode == ERR_CUSTOM_PACKET_INPUT_LEN_IS_LESS_THAN_MIN_POSSIBLE_PACKET_SIZE)
            {
                ESP_LOGW(TAG, "IMPOSSIBLE CASE, SENT 0 BYTES to parse");
                lenToRelease = indexToTest;
                break;
            }
            else if (retCode == ERR_CUSTOM_PACKET_START_NIBBLE_IS_INCORRECT)
            {
                ESP_LOGI(TAG, "CUSTOM PACKET INVALID NIBBLE, Incresing index");
                indexToTest++;
                lenToRelease = indexToTest;
            }
            else if (retCode == ERR_CUSTOM_PACKET_TOTAL_LEN_IS_NOT_SUFFICIENT)
            {
                ESP_LOGD(TAG, "CUSTOM PACKET MORE DATA REQUIRED");
                lenToRelease = indexToTest;
                break;
            }
            else if (retCode == ERR_CUSTOM_PACKET_CRC_MISMATCH)
            {
                ESP_LOGW(TAG, "CUSTOM PACKET CRC MISMATCHED, Discarding whole data");
                lenToRelease = processedBytes + indexToTest;
                sendNACK(channelNum, RESP_NACK_CODE_CRC_FAILURE);
                break;
            }
            else
            {
                ESP_LOGW(TAG, "CUSTOM PACKET INVALID RETURN CODE, Discarding whole data");
                lenToRelease = rxPktSize;
                break;
            }
        }
        if (lenToRelease > 0)
        {
           // SockRxData_releaseLen(s, lenToRelease);
            SockRxData_releaseLen(&sockRxData, lenToRelease);
        }
        if (retCode == CUSTOM_PACKET_OK)
        {
            appcomm=true;
            if (startNibble == CUSTOM_PACKET_START_NIBBLE_DATA)
            {
                if (channelNum == 0x00 || channelNum == 0x01)
                {
                    inst_sendDataToCANProcess(&channelAll[channelNum], canData, canDataLen);
                } 
                else if (channelNum == 0x02 || channelNum == 0x03)
                {
                   senddatatokl(channelNum, canData, canDataLen);
                } else {
                    if (canData != NULL) {
                        vPortFree(canData);
                        canData = NULL;
                    }
                }
            }
            else if (startNibble == CUSTOM_PACKET_START_NIBBLE_SECURITY)
            {
                inst_checkSecurityCode(channelNum, canData, canDataLen);
                if (canData != NULL) {
                    vPortFree(canData);
                    canData = NULL;
                }
            }
            else if (startNibble == CUSTOM_PACKET_START_NIBBLE_SETTING)
            {
                if (channelNum == 0x00 || channelNum == 0x01)
                {
                    inst_settings(&channelAll[channelNum], canData, canDataLen);
                }
                else if (channelNum == 0x02 || channelNum == 0x03)
                {
                    inst_kl_settings(channelNum,canData,canDataLen);
                }
                if (canData != NULL) {
                    vPortFree(canData);
                    canData = NULL;
                }
            }
            else if (startNibble == CUSTOM_PACKET_START_NIBBLE_IVN_CMD)
            {
                inst_handleIVNCMD(&channelAll[channelNum], canData, canDataLen);
                if (canData != NULL) {
                    vPortFree(canData);
                    canData = NULL;
                }
            }
            else
            {
                ESP_LOGE(TAG, "Impossible case, invalid Start nibble %x", startNibble);
                if (canData != NULL) {
                    vPortFree(canData);
                    canData = NULL;
                }
            }
        }
        else
        {
            if (canData != NULL) {
                vPortFree(canData);
                canData = NULL;
            }
        }
    }
}

void sock_recv_task(void *param)
{
    SocketRunHandle *sockRunHadle = (SocketRunHandle *)param;
    const int sock = sockRunHadle->sock;
    EventGroupHandle_t eventHandle = sockRunHadle->eventGroupHandle;

    sockRxData.head = 0;
    sockRxData.tail = 0;
    sockRxData.isOverLapped = false;

    int len;
    ESP_LOGI(TAG, "RX TASK Starting");
    while (1)
    {
        size_t lenToAsk;
        do
        {
            lenToAsk = LOCAL_MIN(SOCKET_RX_CACHE_SIZE, SockRxData_remainingContinuousLen(&sockRxData));
            // ESP_LOGW(TAG, "Head: %d, Tail: %d, isOver: %d, lentoask: %d",
            //         sockRxData.head, sockRxData.tail, sockRxData.isOverLapped, lenToAsk);
            if (lenToAsk <= 0)
            {
                ESP_LOGW(TAG, "No more space in rx buffer, waiting for other tasks to freeup space");
                vTaskDelay(1000);
            }
        } while (lenToAsk <= 0);
        // ESP_LOGI(TAG, "BEFORE: Head: %d, Tail: %d, isOver: %d, lentoask: %d",
        //         sockRxData.head, sockRxData.tail, sockRxData.isOverLapped, lenToAsk);
        len = recv(sock, SockRxData_getHeadPtr(&sockRxData), lenToAsk, 0);
        if (len <= 0)
        {
            ESP_LOGW(TAG, "SOCKET RETURN NEGATIVE DATA");
            break;
        } 
        // else {
        //     ESP_LOGI(TAG, "SOCK RX LEN: %d", len);
        // }
        // ESP_LOGI(TAG, "AFTER: Head: %d, Tail: %d, isOver: %d, len: %d",
        //         sockRxData.head, sockRxData.tail, sockRxData.isOverLapped, len);
        SockRxData_addLen(&sockRxData, len);
        // ESP_LOGW(TAG, "Head: %d, Tail: %d, isOver: %d, lentoask: %d",
        //         sockRxData.head, sockRxData.tail, sockRxData.isOverLapped, lenToAsk);

        uint16_t rxPktSize = SockRxData_usedBuffer(&sockRxData);
        if (rxPktSize > SOCKET_RX_BUFFER_SIZE)
        {
            // ESP_LOGE(TAG, "UNEXPECTED rxPktSize : %d", rxPktSize);
            // ESP_LOGW(TAG, "Head: %d, Tail: %d, isOver: %d",
            //          sockRxData.head, sockRxData.tail, sockRxData.isOverLapped);
            rxPktSize = 1;
        }
        uint8_t *rxPktBuffer = (uint8_t *)pvPortMalloc(rxPktSize);
        if (rxPktBuffer == NULL) {
            ESP_LOGW(TAG, "Failed to allocate memory size : %d", rxPktSize);
        } else {
            SockRxData_fillBuffer(&sockRxData, rxPktBuffer, rxPktSize);

            // ESP_LOGI(TAG, "Received Socket bytes: %d :", rxPktSize);
            // ESP_LOG_BUFFER_HEX(TAG, rxPktBuffer, rxPktSize);
            Activechannel=TCPSCKT; 
           // processAcquiredData(&sockRxData, rxPktBuffer, rxPktSize);
           processAcquiredData(rxPktBuffer, rxPktSize);
            vPortFree(rxPktBuffer);
            rxPktBuffer = NULL;
        }
    }
    if (len < 0)
    {
        ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
    }
    else if (len == 0)
    {
        ESP_LOGW(TAG, "Connection closed");
    }
    xEventGroupSetBits(eventHandle, SOCK_EVENT_SOCKET_CLOSED | SOCK_EVENT_SOCKET_RX_TASK_CLOSED);
    vTaskDelete(NULL);
}

static QueueHandle_t txQueueHandle = NULL;
typedef struct
{
    uint8_t *data;
    uint16_t len;
} TxDataHolder;

void sock_transmit_task(void *param)
{
    SocketRunHandle *sockRunHadle = (SocketRunHandle *)param;
    const int sock = sockRunHadle->sock;
    EventGroupHandle_t eventHandle = sockRunHadle->eventGroupHandle;
    txQueueHandle = xQueueCreate(10, sizeof(TxDataHolder));
    while (1)
    {
        EventBits_t retBits = xEventGroupWaitBits(eventHandle, SOCK_EVENT_SOCKET_CLOSED, pdTRUE, pdFALSE, pdMS_TO_TICKS(1));
        if ((retBits & SOCK_EVENT_SOCKET_CLOSED) != 0x00)
        {
            // SOCKET GOT CLOSED
            ESP_LOGI(TAG, "Socket close event detected in transmit task");
            break;
        }
        // ESP_LOGI(TAG, "Socket TX task is running");
        TxDataHolder dataHolder;
        BaseType_t txRet = xQueueReceive(txQueueHandle, &dataHolder, pdMS_TO_TICKS(100));
        if (txRet == pdTRUE)
        {
            //DEBUG_PRINT_HEX_DATA(dataHolder.data, dataHolder.len);
            size_t sentSize = send(sock, dataHolder.data, dataHolder.len, 0);
            (void)sentSize;
            vPortFree(dataHolder.data);
            dataHolder.data = NULL;
        }
    }

    BaseType_t clrRet;
    do
    {
        TxDataHolder dataHolder;
        clrRet = xQueueReceive(txQueueHandle, &dataHolder, 0);
        if (clrRet == pdTRUE)
        {
            vPortFree(dataHolder.data);
            dataHolder.data = NULL;
        }
    } while (clrRet == pdTRUE);
    vQueueDelete(txQueueHandle);
    txQueueHandle = NULL;

    xEventGroupSetBits(eventHandle, SOCK_EVENT_SOCKET_TX_TASK_CLOSED);
    vTaskDelete(NULL);
}

void sock_queue_dataPointerWithLen(uint8_t *data, uint16_t len)
{
    if (txQueueHandle == NULL)
    {
        //ESP_LOGI(TAG, "TX queue is Null, freeing data");
        vPortFree(data);
        data = NULL;
        return;
    }
    TxDataHolder dataHolder = {.data = data, .len = len};
    BaseType_t ret = xQueueSend(txQueueHandle, &dataHolder, pdMS_TO_TICKS(1000));
    if (ret == pdTRUE)
    {
        ESP_LOGI(TAG, "Queued data to be sent");
    }
    else
    {
        ESP_LOGI(TAG, "Could not queue data to tx");
        vPortFree(data);
        data = NULL;
        return;
    }
}

static void socket_service(void *arg)
{
    char addr_str[128];
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;
    struct sockaddr_storage dest_addr;

    struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4->sin_family = AF_INET;
    dest_addr_ip4->sin_port = htons(PORT);
    ip_protocol = IPPROTO_TCP; // IPPROTO_IP;

    int listen_sock = socket(AF_INET, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0)
    {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0)
    {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", AF_INET);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    err = listen(listen_sock, 1);
    if (err != 0)
    {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while (1)
    {

        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Set tcp keepalive option
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        // Convert ip address to string
        if (source_addr.ss_family == PF_INET)
        {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }

        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

       // initialise_canbus(&channelAll[0]);
       // initialise_can2bus(&channelAll[1]);
        //initialise_kl1();

        EventGroupHandle_t socketEventGroupHandle = xEventGroupCreate();
        SocketRunHandle socketRunHandle = {
            .sock = sock, .eventGroupHandle = socketEventGroupHandle};
        if (xTaskCreatePinnedToCore(sock_recv_task, "sock_rx_task", 1024 * 3, (void *)&socketRunHandle,
                        tskIDLE_PRIORITY + 4, NULL, 1) == pdPASS)
        {
            if (xTaskCreate(sock_transmit_task, "sock_tx_task", 1024 * 2, (void *)&socketRunHandle,
                            tskIDLE_PRIORITY + 4, NULL) == pdPASS)
            {
                EventBits_t retBits = xEventGroupWaitBits(socketEventGroupHandle,
                                                          SOCK_EVENT_SOCKET_RX_TASK_CLOSED | SOCK_EVENT_SOCKET_TX_TASK_CLOSED, pdTRUE, pdTRUE, portMAX_DELAY);
                if (retBits & (SOCK_EVENT_SOCKET_RX_TASK_CLOSED | SOCK_EVENT_SOCKET_TX_TASK_CLOSED))
                {
                    ESP_LOGI(TAG, "Socket RX & TX task got closed");
                }
                vEventGroupDelete(socketEventGroupHandle);
            }
        }

        shutdown(sock, 0);
        close(sock);

        //deinitialise_canbus(&channelAll[0]);
        //deinitialise_can2bus(&channelAll[1]);

        devSettings.status.isSecurityCheckPassed = false;
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}

void initialise_sock(void)
{
    //xTaskCreate(socket_service, "socket_service", 3 * 1024, NULL, tskIDLE_PRIORITY + 4, NULL);
    xTaskCreate(socket_service, "socket_service", 3 * 1024, NULL, tskIDLE_PRIORITY + 6, NULL);
    
}
