#pragma once

#include "FreeRTOS.h"
#include "semphr.h"

#include "eeprom-driver/BusAccessorBase.hpp"
#include "hal_header.h"

template <class RegisterAddress>
class I2cAccessor : public BusAccessorBase<RegisterAddress>
{
public:
    using DeviceAddress = typename BusAccessorBase<RegisterAddress>::DeviceAddress;

    explicit I2cAccessor(I2C_HandleTypeDef *hi2c) : i2cHandle{hi2c}
    {
        mutex = xSemaphoreCreateMutex();
        binary = xSemaphoreCreateBinary();
    }

    bool operator==(const I2cAccessor &other) const
    {
        return i2cHandle == other.i2cHandle;
    }

    void beginTransaction(DeviceAddress address) override
    {
        xSemaphoreTake(mutex, portMAX_DELAY);
        currentAddress = address;
    }

    void endTransaction() override
    {
        xSemaphoreGive(mutex);
    }

    bool read(uint8_t *buffer, size_t length) override
    {
        errorCondition = false;
        HAL_I2C_Master_Receive_IT(i2cHandle, currentAddress << 1, buffer, length);

        const auto semphrSuccess = xSemaphoreTake(binary, Timeout);
        return !(semphrSuccess == pdFALSE || errorCondition);
    }

    bool readFromRegister(RegisterAddress registerAddress, uint8_t *buffer, size_t length) override
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

    bool readByteFromRegister(RegisterAddress registerAddress, uint8_t &byte) override
    {
        return readFromRegister(registerAddress, &byte, 1);
    }

    bool readWordFromRegister(RegisterAddress registerAddress, uint16_t &word) override
    {
        bool returnValue = readFromRegister(registerAddress, reinterpret_cast<uint8_t *>(&word), 2);
        swapBytes(word);
        return returnValue;
    }

    bool write(const uint8_t *data, size_t length) override
    {
        errorCondition = false;
        HAL_I2C_Master_Transmit_IT(i2cHandle, currentAddress << 1, const_cast<uint8_t *>(data),
                                   length);

        auto semphrSuccess = xSemaphoreTake(binary, Timeout);
        return (semphrSuccess == pdFALSE || errorCondition);
    }

    bool writeToRegister(RegisterAddress registerAddress, const uint8_t *data,
                         size_t length) override
    {
        errorCondition = false;
        swapBytes(registerAddress);
        HAL_I2C_Master_Seq_Transmit_IT(i2cHandle, currentAddress << 1,
                                       reinterpret_cast<uint8_t *>(&registerAddress),
                                       sizeof(RegisterAddress), I2C_FIRST_FRAME);

        auto semphrSuccess = xSemaphoreTake(binary, Timeout);
        if (semphrSuccess == pdFALSE || errorCondition)
            return false;

        HAL_I2C_Master_Seq_Transmit_IT(i2cHandle, currentAddress << 1, const_cast<uint8_t *>(data),
                                       length, I2C_LAST_FRAME);

        semphrSuccess = xSemaphoreTake(binary, Timeout);
        return !(semphrSuccess == pdFALSE || errorCondition);
    }

    bool writeByteToRegister(RegisterAddress registerAddress, const uint8_t byte) override
    {
        return writeToRegister(registerAddress, &byte, 1);
    }

    bool writeWordToRegister(RegisterAddress registerAddress, uint16_t word) override
    {
        swapBytes(word);
        return writeToRegister(registerAddress, reinterpret_cast<const uint8_t *>(&word), 2);
    }

    void signalTransferCompleteFromIsr(BaseType_t *higherPrioTaskWoken)
    {
        errorCondition = false;
        xSemaphoreGiveFromISR(binary, higherPrioTaskWoken);
    }

    void signalErrorFromIsr(BaseType_t *higherPrioTaskWoken)
    {
        errorCondition = true;
        xSemaphoreGiveFromISR(binary, higherPrioTaskWoken);
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
    DeviceAddress currentAddress;
    SemaphoreHandle_t mutex = nullptr;
    SemaphoreHandle_t binary = nullptr;
    bool errorCondition;
};
