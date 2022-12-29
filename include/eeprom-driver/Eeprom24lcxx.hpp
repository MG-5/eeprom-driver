#pragma once
#include "FreeRTOS.h"
#include "task.h"

#include "eeprom-driver/EepromBase.hpp"
#include "eeprom-driver/I2cAccessor.hpp"

#include <array>
#include <core/SafeAssert.h>
#include <limits>

using AddressSize = uint16_t;

template <size_t KiB, size_t BytesPerPage>
class Eeprom24lcxx : public EepromBase<KiB, AddressSize>
{
public:
    Eeprom24lcxx(I2cAccessor &accessor, I2cAccessor::DeviceAddress chipSelectBits)
        : accessor{accessor}, deviceAddress{static_cast<I2cAccessor::DeviceAddress>(
                                  BaseAddress | (chipSelectBits & ChipSelectMask))} {

                              };

    void read(AddressSize address, uint8_t *buffer, size_t length) override
    {
        SafeAssert(length != 0);
        SafeAssert(length - 1 <= std::numeric_limits<uint16_t>::max());
        SafeAssert(!this->doesAddressExceedLimit(address + length - 1));

        accessor.beginTransaction(deviceAddress);
        accessor.readFromRegister(address, buffer, length);
        accessor.endTransaction();
    }

    void write(AddressSize address, const uint8_t *data, size_t length) override
    {
        SafeAssert(length != 0);
        SafeAssert(length - 1 <= std::numeric_limits<uint16_t>::max());
        SafeAssert(!this->doesAddressExceedLimit(address + length - 1));

        const auto firstPage = address / BytesPerPage;
        const auto lastPage = (address + length) / BytesPerPage;
        const auto numberOfFullPages = firstPage == lastPage ? 0 : lastPage - firstPage - 1;

        const auto numberOfBytesOnFirstPage =
            std::min(BytesPerPage - (address % BytesPerPage), (size_t)length);
        const auto numberOfBytesOnLastPage = (address + length) % BytesPerPage;

        accessor.beginTransaction(deviceAddress);

        // Write first page
        accessor.writeToRegister(address, data, numberOfBytesOnFirstPage);
        waitPageWriteFinished();

        address += numberOfBytesOnFirstPage;
        data += numberOfBytesOnFirstPage;

        for (size_t i = 0; i < numberOfFullPages; ++i)
        {
            accessor.writeToRegister(address, data, BytesPerPage);
            waitPageWriteFinished();

            address += BytesPerPage;
            data += BytesPerPage;
        }

        // Write last page
        if (firstPage != lastPage && numberOfBytesOnLastPage > 0)
        {
            accessor.writeToRegister(address, data, numberOfBytesOnLastPage);
            waitPageWriteFinished();
        }

        accessor.endTransaction();
    }

protected:
    void waitPageWriteFinished()
    {
        // Do ACK Polling
        while (!accessor.write(nullptr, 0))
        {
            // chip takes about 5ms to store the data
            // no use in polling constantly and eating up cpu
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    static constexpr uint8_t BaseAddress = 0b1010 << 3;
    static constexpr uint8_t ChipSelectMask = 0b111;

    I2cAccessor &accessor;
    I2cAccessor::DeviceAddress deviceAddress;
};

using Eeprom24LC64 = Eeprom24lcxx<64, 32>;