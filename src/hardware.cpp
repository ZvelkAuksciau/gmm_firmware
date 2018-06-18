#include "ch.hpp"
#include "hal.h"

#include <hardware.hpp>
#include <cstring>
#include <math.h>

#include <config/config_storage_flash.hpp>
#include <config/config.hpp>

namespace Hardware {

    #define HALF_POWER 750

    os::config::Param<float> max_motor_power("mot.max_power", 0.5f, 0.0f, 1.0f);

    static const PWMConfig pwm_cfg = {
       72000000,                         // 72 MHz PWM clock frequency
       1500,                             // 24 kHz PWM frequency
       NULL,                             // No Callback
       {
          {PWM_OUTPUT_ACTIVE_HIGH, NULL},
          {PWM_OUTPUT_ACTIVE_HIGH, NULL},
          {PWM_OUTPUT_ACTIVE_HIGH, NULL},
          {PWM_OUTPUT_DISABLED, NULL}
       },
       0,
       0
    };

    UUID readUniqueID()
    {
        UUID out_bytes;
        memcpy(out_bytes.data(), reinterpret_cast<const void*>(0x1FFFF7E8), std::tuple_size<UUID>::value);
        return out_bytes;
    }

    void setStatusLed(bool on) {
        palWritePad(PORT_LED_RED, GPIO_LED_RED, on);
    }

    void setCANLed(bool on) {
        palWritePad(PORT_LED_GREEN, GPIO_LED_GREEN, on);
    }

    void setPwmCommand(float cmd, float set_power) {
        //Limit max power to motor
        if(set_power >= max_motor_power.get())
            set_power = max_motor_power.get();
        else if(set_power < -max_motor_power.get())
            set_power = -max_motor_power.get();

        float power = HALF_POWER * set_power;
        if(set_power >= 0.0f) {
            pwmEnableChannel(&PWMD3, PWM_CHANNEL_IN3, (HALF_POWER + power * sinf(cmd)));
            pwmEnableChannel(&PWMD3, PWM_CHANNEL_IN2, (HALF_POWER + power * sinf(cmd - 2.094)));
            pwmEnableChannel(&PWMD3, PWM_CHANNEL_IN1, (HALF_POWER + power * sinf(cmd + 2.094)));
        } else {
            pwmEnableChannel(&PWMD3, PWM_CHANNEL_IN2, (HALF_POWER - power * sinf(cmd)));
            pwmEnableChannel(&PWMD3, PWM_CHANNEL_IN3, (HALF_POWER - power * sinf(cmd - 2.094)));
            pwmEnableChannel(&PWMD3, PWM_CHANNEL_IN1, (HALF_POWER - power * sinf(cmd + 2.094)));
        }
    }

    void enablePWMOutput() {
      palSetPad(PORT_EN1, GPIO_EN1);
      palSetPad(PORT_EN2, GPIO_EN2);
      palSetPad(PORT_EN3, GPIO_EN3);
    }

    void disablePWMOutput() {
        palClearPad(PORT_EN1, GPIO_EN1);
        palClearPad(PORT_EN2, GPIO_EN2);
        palClearPad(PORT_EN3, GPIO_EN3);
    }

    static void* const ConfigStorageAddress = reinterpret_cast<void*>(0x08000000 + (128 * 1024) - 1024);
    constexpr unsigned ConfigStorageSize = 1024;

    os::watchdog::Timer init() {
        halInit();
        chSysInit();

        Hardware::disablePWMOutput();

        os::watchdog::init();
        os::watchdog::Timer wdt;
        wdt.startMSec(5000);
        wdt.reset();

        sdStart(&SD1, NULL);

        pwmStart(&PWMD3, &pwm_cfg);
        PWMD3.tim->CR1 |= STM32_TIM_CR1_CMS(1); //Set Center aligned mode

        static os::stm32::ConfigStorageBackend config_storage_backend(ConfigStorageAddress, ConfigStorageSize);
        const int config_init_res = os::config::init(&config_storage_backend);


        return wdt;
    }
}
