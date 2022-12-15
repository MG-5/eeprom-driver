#pragma once

#include <cstddef>
#include <cstdint>

template <class RegisterAddress>
class BusAccessorBase
{
public:
    using DeviceAddress = uint16_t;

    virtual void beginTransaction(DeviceAddress address) = 0;
    virtual void endTransaction() = 0;

    virtual bool read(uint8_t *buffer, size_t length) = 0;
    virtual bool readFromRegister(RegisterAddress registerAddress, uint8_t *buffer,
                                  size_t length) = 0;
    virtual bool readByteFromRegister(RegisterAddress registerAddress, uint8_t &byte) = 0;
    virtual bool readWordFromRegister(RegisterAddress registerAddress, uint16_t &word) = 0;

    virtual bool write(const uint8_t *data, size_t length) = 0;
    virtual bool writeToRegister(RegisterAddress registerAddress, const uint8_t *data,
                                 size_t length) = 0;
    virtual bool writeByteToRegister(RegisterAddress registerAddress, uint8_t byte) = 0;
    virtual bool writeWordToRegister(RegisterAddress registerAddress, uint16_t word) = 0;
};