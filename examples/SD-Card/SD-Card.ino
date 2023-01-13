/*

SD Card required file structure:

/
| boot.bin : Bootloader binary
| part.bin : Partition binary
| prog.bin : Application binary


Target ESP32 required pin connections:
- Rx0
- Tx0
- GPIO_0
- EN/RST


If package error occurs, decrease baud rate below

*/

#include "SPI.h"
#include "SD.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "esp32_port.h"

#define BOOTLOADER_ADDRESS  0x1000
#define PARTITION_ADDRESS   0x8000
#define APPLICATION_ADDRESS 0x10000

#define UPLOAD_BAUD 230400

#define RX_PIN GPIO_NUM_26
#define TX_PIN GPIO_NUM_27
#define GPIO_0_PIN GPIO_NUM_25
#define RST_PIN GPIO_NUM_33

esp_loader_error_t flash_binary(Stream& bin, size_t size, size_t address)
{
    esp_loader_error_t err;
    static uint8_t payload[1024];

    printf("Erasing flash (this may take a while)...\n");
    err = esp_loader_flash_start(address, size, sizeof(payload));
    if (err != ESP_LOADER_SUCCESS) {
        printf("Erasing flash failed with error %d.\n", err);
        return err;
    }
    printf("Start programming\n");

    size_t written = 0;
    size_t remaining = size;

    while (remaining > 0) {
        size_t to_read = min(remaining, sizeof(payload));
        bin.readBytes(payload, to_read);

        err = esp_loader_flash_write(payload, to_read);
        if (err != ESP_LOADER_SUCCESS) {
            printf("\nPacket could not be written! Error %d.\n", err);
            return err;
        }

        remaining -= to_read;
        written += to_read;

        int progress = (int)(((float)written / size) * 100);
        printf("\rProgress: %d %%", progress);
        fflush(stdout);
    };

    printf("\nFinished programming\n");

    err = esp_loader_flash_verify();
    if (err == ESP_LOADER_ERROR_UNSUPPORTED_FUNC) {
        printf("ESP8266 does not support flash verify command.");
        return err;
    } else if (err != ESP_LOADER_SUCCESS) {
        printf("MD5 does not match. err: %d\n", err);
        return err;
    }
    printf("Flash verified\n");

    return ESP_LOADER_SUCCESS;
}

bool flashFile(const char* filePath, size_t addr) {
    fs::File file = SD.open(filePath, "r", false);
    if(!file)
        return false;
    flash_binary(file, file.size(), addr);
    file.close();

    return true;
}

void setup() {
    Serial.begin(115200);

    // Reset other board
    /* gpio_reset_pin((gpio_num_t)33);
    gpio_set_pull_mode((gpio_num_t)33, GPIO_PULLUP_ONLY);
    gpio_set_direction((gpio_num_t)33, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)33, 0);
    delayMicroseconds(50);
    gpio_set_level((gpio_num_t)33, 1); */
    
    SPI.begin();
    if(!SD.begin()) {
        Serial.println("SD card mount failed! Restarting in 5 seconds...");
        delay(5000);
        esp_restart();
        return;
    }

    if(!SD.exists("/boot.bin")) {
        Serial.println("Bootloader binary not found! (/boot.bin)");
        return;
    }
    if(!SD.exists("/part.bin")) {
        Serial.println("Partition binary not found! (/part.bin)");
        return;
    }
    if(!SD.exists("/prog.bin")) {
        Serial.println("Application binary not found! (/prog.bin)");
        return;
    }

    Serial.println("Flasher ready, send anything to begin the process!");
    // Wait for signal to flash
    while(Serial.available() == 0)
        ;
    
    // Initialize serial port
    const loader_esp32_config_t config = {
        .baud_rate = 115200,
        .uart_port = UART_NUM_1,
        .uart_rx_pin = TX_PIN,
        .uart_tx_pin = RX_PIN,
        .reset_trigger_pin = RST_PIN,
        .gpio0_trigger_pin = GPIO_0_PIN,
    };

    if (loader_port_esp32_init(&config) != ESP_LOADER_SUCCESS) {
        Serial.println("UART initialization failed.");
        return;
    }

    delay(10);

    // Make connection via reset and gpio0
    esp_loader_connect_args_t connect_arg = ESP_LOADER_CONNECT_DEFAULT();
    esp_loader_error_t connect_err = esp_loader_connect(&connect_arg);

    if(connect_err != esp_loader_error_t::ESP_LOADER_SUCCESS) {
        Serial.println("Connection error!");
        return;
    }

    esp_loader_error_t espBaudErr = esp_loader_change_baudrate(UPLOAD_BAUD);
    if(espBaudErr == ESP_LOADER_SUCCESS) {
        loader_port_change_baudrate(UPLOAD_BAUD);
        Serial.printf("Changed baud rate to %d\n", UPLOAD_BAUD);
    }

    if(!flashFile("/boot.bin", BOOTLOADER_ADDRESS))
        Serial.println("Failed to open/write boot.bin");
    if(!flashFile("/part.bin", PARTITION_ADDRESS))
        Serial.println("Failed to open/write part.bin");
    if(!flashFile("/prog.bin", APPLICATION_ADDRESS))
        Serial.println("Failed to open/write prog.bin");

    Serial.println("Flash finished!");
}

void loop() {

}

