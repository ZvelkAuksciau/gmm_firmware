#pragma once

#include <array>
#include <cstdint>

#include <os.hpp>

namespace Hardware {

    /*
     * SPI configuration (9MHz, CPHA=1, CPOL=0, MSb first).
     */
    static const SPIConfig spicfg = {
        NULL,
        GPIOA,
        GPIOA_SPI1NSS,
        SPI_CR1_BR_1 | SPI_CR1_CPHA,
        0
    };

    typedef std::array<std::uint8_t, 12> UUID;
    UUID readUniqueID();

    void setStatusLed(bool on);
    void setCANLed(bool on);

    void setPwmCommand(float cmd, float set_power);
    void enablePWMOutput();
    void disablePWMOutput();

    os::watchdog::Timer init();
}
