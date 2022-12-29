#pragma once

#include "eeprom-driver/BusAccessorBase.hpp"
#include <cstddef>
#include <limits>

template <size_t KiB, class AddressSize>
class EepromBase
{
public:
    EepromBase()
    {
        static_assert(std::numeric_limits<AddressSize>::min() == 0,
                      "Choose Address to be unsigned!");

        static_assert(static_cast<uint64_t>(getSizeInBytes() - 1) <=
                          static_cast<uint64_t>(std::numeric_limits<AddressSize>::max()),
                      "Address is not able to cover complete size of eeprom. Choose a bigger type");

        static_assert(static_cast<uint64_t>(getSizeInBytes() - 1) <=
                          static_cast<uint64_t>(std::numeric_limits<uint16_t>::max()),
                      "read() and write() are still statically written for 2-byte addresses. "
                      "Revise implementations");
    }

    static constexpr size_t getSizeInBytes()
    {
        return KiB * 1024;
    }

    virtual void read(AddressSize address, uint8_t *buffer, size_t length) = 0;

    virtual void write(AddressSize address, const uint8_t *data, size_t length) = 0;

protected:
    bool doesAddressExceedLimit(AddressSize address)
    {
        return address >= getSizeInBytes();
    }
};