/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Board identifier.
 */
#define BOARD_NAME              "Gimbal_CTRL_Rev2"

/*
 * Board frequencies.
 */
#define STM32_LSECLK            32768
#define STM32_HSECLK            16000000

/*
 * MCU type, supported types are defined in ./os/hal/platforms/hal_lld.h.
 */
#define STM32F103xB

/*
 * IO pins assignments.
 */
#define PORT_LED_RED GPIOB
#define GPIO_LED_RED 6
#define PORT_LED_GREEN GPIOB
#define GPIO_LED_GREEN 7
#define PORT_SPI1NSS GPIOA
#define GPIO_SPI1NSS 8

#define PORT_IN1 GPIOA
#define GPIO_IN1 6
#define PORT_IN2 GPIOA
#define GPIO_IN2 7
#define PORT_IN3 GPIOB
#define GPIO_IN3 0

#define PWM_CHANNEL_IN1 0
#define PWM_CHANNEL_IN2 1
#define PWM_CHANNEL_IN3 2


#define PORT_EN1 GPIOA
#define GPIO_EN1 0
#define PORT_EN2 GPIOA
#define GPIO_EN2 1
#define PORT_EN3 GPIOA
#define GPIO_EN3 2

#define PORT_RESET GPIOA
#define GPIO_RESET 3
#define PORT_SLEEP GPIOA
#define GPIO_SLEEP 4
#define PORT_FAULT GPIOA
#define GPIO_FAULT 5


/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 *
 * The digits have the following meaning:
 *   0 - Analog input.
 *   1 - Push Pull output 10MHz.
 *   2 - Push Pull output 2MHz.
 *   3 - Push Pull output 50MHz.
 *   4 - Digital input.
 *   5 - Open Drain output 10MHz.
 *   6 - Open Drain output 2MHz.
 *   7 - Open Drain output 50MHz.
 *   8 - Digital input with PullUp or PullDown resistor depending on ODR.
 *   9 - Alternate Push Pull output 10MHz.
 *   A - Alternate Push Pull output 2MHz.
 *   B - Alternate Push Pull output 50MHz.
 *   C - Reserved.
 *   D - Alternate Open Drain output 10MHz.
 *   E - Alternate Open Drain output 2MHz.
 *   F - Alternate Open Drain output 50MHz.
 * Please refer to the STM32 Reference Manual for details.
 */

/*
 * Port A setup.
 * Everything input with pull-up except:
 * PA0  - Push Pull output 2MHz EN1
 * PA1  - Push Pull output 2MHz EN2
 * PA2  - Push Pull output 2MHz EN3
 * PA3  - Push Pull output 2MHz DRV_RESET
 * PA4  - Push Pull output 2MHz DRV_SLEEP
 * PA5  - Digital input with PullUp or PullDown DRV_FAULT
 * PA6  - Alternate Push Pull output 50MHz PWM_IN1
 * PA7  - Alternate Push Pull output 50MHz PWM_IN2
 * PA8  - Push Pull output 50MHz ENC_SEL
 * PA9  - Alternate Push Pull 50MHz (USART1_TX).
 * PA10 - Digital input with Pull up (USART1_RX).
 * PA11 - Digital input CAN_RX
 * PA12 - Alternate Push Pull output 50MHz CAN_TX
 */
#define VAL_GPIOACRL            0xBB822222      /*  PA7...PA0 */
#define VAL_GPIOACRH            0x888B48B3      /* PA15...PA8 */
#define VAL_GPIOAODR            0xFFFFFFF8

/*
 * Port B setup.
 * Everything input with pull-up except:
 * PB0 - Alternate Push Pull output 50MHz PWM_IN3
 * PB3 - Alternate Push Pull output 50MHz SP1_SCK
 * PB4 - Digital input with PullUp SPI1_MISO
 * PB5 - Alternate Push Pull output 50MHz SPI1_MOSI
 * PB6 - Push Pull output 2MHz STATUS_LED_RED
 * PB7 - Push Pull output 2MHz STATUS_LED_GREEN
 */
#define VAL_GPIOBCRL            0x228BB88B      /*  PB7...PB0 */
#define VAL_GPIOBCRH            0x88888888      /* PB15...PB8 */
#define VAL_GPIOBODR            0xFFFFFF3F

/*
 * Port C setup.
 * Everything input with pull-up except:
 * PC14 reset
 */
#define VAL_GPIOCCRL            0x88888888      /*  PC7...PC0 */
#define VAL_GPIOCCRH            0x82888888      /* PC15...PC8 */
#define VAL_GPIOCODR            0xFFFFFFFF

/*
 * Port D setup.
 * Everything input with pull-up except:
 * PD0  - Normal input (XTAL).
 * PD1  - Normal input (XTAL).
 */
#define VAL_GPIODCRL            0x88888844      /*  PD7...PD0 */
#define VAL_GPIODCRH            0x88888888      /* PD15...PD8 */
#define VAL_GPIODODR            0xFFFFFFFF

/*
 * Port E setup.
 * Everything input with pull-up except:
 */
#define VAL_GPIOECRL            0x88888888      /*  PE7...PE0 */
#define VAL_GPIOECRH            0x88888888      /* PE15...PE8 */
#define VAL_GPIOEODR            0xFFFFFFFF

/*
 * USB bus activation macro, required by the USB driver.
 */
#define usb_lld_connect_bus(usbp) (void)//palClearPad(GPIOC, GPIOC_USB_DISC)

/*
 * USB bus de-activation macro, required by the USB driver.
 */
#define usb_lld_disconnect_bus(usbp) (void)//palSetPad(GPIOC, GPIOC_USB_DISC)

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
    void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
