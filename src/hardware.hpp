#pragma once

#include <array>
#include <cstdint>

#include <os.hpp>

namespace Hardware {

#define HARDWARE_ENC_PRESENT       0x01

    /*
     * SPI configuration (9MHz, CPHA=1, CPOL=0, MSb first).
     */
    static const SPIConfig spicfg = {
        NULL,
        PORT_SPI1NSS,
        GPIO_SPI1NSS,
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

    extern uint8_t g_board_status;

    os::watchdog::Timer init();
}
