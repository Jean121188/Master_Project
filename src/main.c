#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"
#include "string.h"
#include "driver/gpio.h"

#define RXD_PIN (GPIO_NUM_16)
#define TXD_PIN (GPIO_NUM_17)

static const int RX_BUF_SIZE = 128;

SemaphoreHandle_t mutex;

void init();
int sendData(const char* logName, const char* data);
static void tx_task();
static void rx_task();

void app_main(){
    init();
    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, 2, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, 1, NULL);
}

void init() {
    
    // Configura a UART1 
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);

    // Cria o Semafaro do tipo mutex
    mutex = xSemaphoreCreateMutex();

}

int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Command: %s", data);
    return txBytes;
}

static void tx_task()
{
    static const char *TX_TASK_TAG = "AT_COMMAND: ";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        sendData(TX_TASK_TAG, "AT\n");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

static void rx_task()
{
    static const char *RX_TASK_TAG = "RECEBIDO: ";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    char* data = (char*)malloc(RX_BUF_SIZE+1);
    strcpy(data, "");
    const char* R1 = "OK";

    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        //const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, portMAX_DELAY);
        xSemaphoreTake(mutex, portMAX_DELAY);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            //ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            char *ret = strstr(data, R1); // procura em data OK
            if(ret >= 0){
                ESP_LOGI(RX_TASK_TAG, "Encontrei...");
                xSemaphoreGive(mutex);
            }
        }
    }
    free(data);
}