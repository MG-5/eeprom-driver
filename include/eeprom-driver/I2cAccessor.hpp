#pragma once

#include "FreeRTOS.h"
#include "i2c.h"
#include "semphr.h"

class I2cAccessor
{
public:
    using DeviceAddress = uint8_t;

    explicit I2cAccessor(I2C_HandleTypeDef *hi2c) : i2cHandle{hi2c}
    {
        mutex = xSemaphoreCreateMutex();
        binary = xSemaphoreCreateBinary();
    }

    bool operator==(const I2cAccessor &other) const
    {
        return i2cHandle == other.i2cHandle;
    }

    void beginTransaction(DeviceAddress address)
    {
        xSemaphoreTake(mutex, portMAX_DELAY);
        currentAddress = address;
    }

    void endTransaction()
    {
        xSemaphoreGive(mutex);
    }

    bool read(uint8_t *buffer, size_t length)
    {
        errorCondition = false;
        HAL_I2C_Master_Receive_IT(i2cHandle, currentAddress << 1, buffer, length);

        const auto semphrSuccess = xSemaphoreTake(binary, Timeout);
        return !(semphrSuccess == pdFALSE || errorCondition);
    }

    template <typename RegisterAddress>
    bool readFromRegister(RegisterAddress registerAddress, uint8_t *buffer, size_t length)
    {
        errorCondition = false;
        swapBytes(registerAddress);
        HAL_I2C_Master_Seq_Transmit_IT(i2cHandle, currentAddress << 1,
                                       reinterpret_cast<uint8_t *>(&registerAddress),
                                       sizeof(RegisterAddress), I2C_FIRST_FRAME);

        auto semphrSuccess = xSemaphoreTake(binary, Timeout);
        if (semphrSuccess == pdFALSE || errorCondition)
            return false;

        HAL_I2C_Master_Seq_Receive_IT(i2cHandle, currentAddress << 1,
                                      reinterpret_cast<uint8_t *>(buffer), length, I2C_LAST_FRAME);

        semphrSuccess = xSemaphoreTake(binary, Timeout);
        return !(semphrSuccess == pdFALSE || errorCondition);
    }

    template <typename RegisterAddress>
    bool readByteFromRegister(RegisterAddress registerAddress, uint8_t &byte)
    {
        return readFromRegister(registerAddress, &byte, 1);
    }

    template <typename RegisterAddress>
    bool readWordFromRegister(RegisterAddress registerAddress, uint16_t &word)
    {
        bool returnValue = readFromRegister(registerAddress, reinterpret_cast<uint8_t *>(&word), 2);
        swapBytes(word);
        return returnValue;
    }

    bool write(const uint8_t *data, size_t length)
    {
        errorCondition = false;
        HAL_I2C_Master_Transmit_IT(i2cHandle, currentAddress << 1, const_cast<uint8_t *>(data),
                                   length);

        auto semphrSuccess = xSemaphoreTake(binary, Timeout);
        return !(semphrSuccess == pdFALSE || errorCondition);
    }

    template <typename RegisterAddress>
    bool writeToRegister(RegisterAddress registerAddress, const uint8_t *data, size_t length)
    {
        errorCondition = false;
        swapBytes(registerAddress);
        HAL_I2C_Master_Seq_Transmit_IT(i2cHandle, currentAddress << 1,
                                       reinterpret_cast<uint8_t *>(&registerAddress),
                                       sizeof(RegisterAddress), I2C_FIRST_AND_NEXT_FRAME);

        auto semphrSuccess = xSemaphoreTake(binary, Timeout);
        if (semphrSuccess == pdFALSE || errorCondition)
            return false;

        HAL_I2C_Master_Seq_Transmit_IT(i2cHandle, currentAddress << 1, const_cast<uint8_t *>(data),
                                       length, I2C_LAST_FRAME);

        semphrSuccess = xSemaphoreTake(binary, Timeout);
        return !(semphrSuccess == pdFALSE || errorCondition);
    }

    template <typename RegisterAddress>
    bool writeByteToRegister(RegisterAddress registerAddress, const uint8_t byte)
    {
        return writeToRegister(registerAddress, &byte, 1);
    }

    template <typename RegisterAddress>
    bool writeWordToRegister(RegisterAddress registerAddress, uint16_t word)
    {
        swapBytes(word);
        return writeToRegister(registerAddress, reinterpret_cast<const uint8_t *>(&word), 2);
    }

    void signalTransferCompleteFromIsr()
    {
        BaseType_t higherPrioTaskWoken = pdFALSE;
        errorCondition = false;
        xSemaphoreGiveFromISR(binary, &higherPrioTaskWoken);
        portYIELD_FROM_ISR(higherPrioTaskWoken);
    }

    void signalErrorFromIsr()
    {
        BaseType_t higherPrioTaskWoken = pdFALSE;
        errorCondition = true;
        xSemaphoreGiveFromISR(binary, &higherPrioTaskWoken);
        portYIELD_FROM_ISR(higherPrioTaskWoken);
    }

    template <class T>
    void swapBytes(T &value)
    {
        static_assert(sizeof(T) == 1 || sizeof(T) == 2,
                      "Unimplemented for more than 2-bytes types!");

        if constexpr (sizeof(T) == 1)
            return;

        else if constexpr (sizeof(T) == 2)
            value = ((value & 0xFF) << 8) | (value >> 8);
    }

    static constexpr TickType_t Timeout{pdMS_TO_TICKS(100)};

private:
    I2C_HandleTypeDef *i2cHandle;
    DeviceAddress currentAddress = 0;
    SemaphoreHandle_t mutex = nullptr;
    SemaphoreHandle_t binary = nullptr;
    bool errorCondition = false;
};
