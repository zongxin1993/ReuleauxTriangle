#pragma once

#include <Arduino.h>
#include <EEPROM.h>

class Esp32 {
public:
    Esp32() {
        printInfo();
        if (!EEPROM.begin(1000)) {
            Serial.println("Failed to initialise EEPROM");
            Serial.println("Restarting...");
            delay(1000);
            ESP.restart();
        }
    }

private:

    void printInfo() {
        uint32_t chipId = 0;
        for (int i = 0; i < 17; i = i + 8) {
            chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
        }
        Serial.printf("Chip ID: %d\r\n", chipId);

        Serial.printf("ESP32 Chip ID = %04X", (uint16_t)(ESP.getEfuseMac() >> 32));  //print High 2 bytes
        Serial.printf("%08X\r\n", (uint32_t)ESP.getEfuseMac());                      //print Low 4bytes.

        Serial.printf("Chip model = %s Rev %d\r\n", ESP.getChipModel(), ESP.getChipRevision());
        Serial.printf("This chip has %d cores CpuFreqMHz = %u\r\n", ESP.getChipCores(), ESP.getCpuFreqMHz());
        Serial.printf("get Cycle Count = %u\r\n", ESP.getCycleCount());
        Serial.printf("SDK version:%s\r\n", ESP.getSdkVersion());  //获取IDF版本

        //获取片内内存 Internal RAM
        Serial.printf("Total heap size = %u\t", ESP.getHeapSize());
        Serial.printf("Available heap = %u\r\n", ESP.getFreeHeap());
        Serial.printf("Lowest level of free heap since boot = %u\r\n", ESP.getMinFreeHeap());
        Serial.printf("Largest block of heap that can be allocated at once = %u\r\n", ESP.getMaxAllocHeap());

        //SPI RAM
        Serial.printf("Total Psram size = %u\t", ESP.getPsramSize());
        Serial.printf("Available Psram = %u\r\n", ESP.getFreePsram());
        Serial.printf("Lowest level of free Psram since boot = %u\r\n", ESP.getMinFreePsram());
        Serial.printf("Largest block of Psram that can be allocated at once = %u\r\n", ESP.getMinFreePsram());
        sprintf(mMac, "%02X\r\n", (uint32_t)(ESP.getEfuseMac() >> (24)));
        sprintf(mMac, "ESP32-%c%c%c%c%c%c", mMac[4], mMac[5], mMac[2], mMac[3], mMac[0], mMac[1]);
    }

private:
    char mMac[6];
};
