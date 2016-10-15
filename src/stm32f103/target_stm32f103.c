/*
 * Copyright (c) 2016, Devan Lai
 *
 * Permission to use, copy, modify, and/or distribute this software
 * for any purpose with or without fee is hereby granted, provided
 * that the above copyright notice and this permission notice
 * appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
 * LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/* Common STM32F103 target functions */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/st_usbfs.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/cm3/scb.h>

#include "target.h"
#include "config.h"
#include "backup.h"

#ifndef USES_GPIOA
#if (HAVE_USB_PULLUP_CONTROL == 0)
#define USES_GPIOA 1
#else
#define USES_GPIOA 0
#endif
#endif

#ifndef USES_GPIOB
#define USES_GPIOB 0
#endif

#ifndef USES_GPIOC
#define USES_GPIOC 0
#endif

static const uint32_t CMD_BOOT = 0x544F4F42UL;

void target_clock_setup(void) {
    /* Set system clock to 72 MHz */
    rcc_clock_setup_in_hse_8mhz_out_72mhz();
}

void target_gpio_setup(void) {
    /* Enable GPIO clocks */
    if (USES_GPIOA) {
        rcc_periph_clock_enable(RCC_GPIOA);
    }
    if (USES_GPIOB) {
        rcc_periph_clock_enable(RCC_GPIOB);
    }
    if (USES_GPIOC) {
        rcc_periph_clock_enable(RCC_GPIOC);
    }

    /* Setup LEDs */
#if HAVE_LED
    {
        const uint8_t mode = GPIO_MODE_OUTPUT_10_MHZ;
        const uint8_t conf = (LED_OPEN_DRAIN ? GPIO_CNF_OUTPUT_OPENDRAIN
                                             : GPIO_CNF_OUTPUT_PUSHPULL);
        //if (LED_OPEN_DRAIN) {
            gpio_set(LED_GPIO_PORT, LED_GPIO_PIN);
        /*} else {
            gpio_clear(LED_GPIO_PORT, LED_GPIO_PIN);
        }*/
        gpio_set_mode(LED_GPIO_PORT, mode, conf, LED_GPIO_PIN);
    }
#endif

#if HAVE_USB_PULLUP_CONTROL
    {
        const uint8_t mode = GPIO_MODE_OUTPUT_10_MHZ;
        const uint8_t conf = (USB_PULLUP_OPEN_DRAIN ? GPIO_CNF_OUTPUT_OPENDRAIN
                                                    : GPIO_CNF_OUTPUT_PUSHPULL);
        /* Configure USB pullup transistor, initially disabled */
        if (USB_PULLUP_ACTIVE_HIGH) {
            gpio_clear(USB_PULLUP_GPIO_PORT, USB_PULLUP_GPIO_PIN);
        } else {
            gpio_set(USB_PULLUP_GPIO_PORT, USB_PULLUP_GPIO_PIN);
        }
        gpio_set_mode(USB_PULLUP_GPIO_PORT, mode, conf, USB_PULLUP_GPIO_PIN);
    }
#else
    {
        /* Drive the USB DP pin to override the pull-up */
        gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ,
                      GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
    }
#endif
}

const usbd_driver* target_usb_init(void) {
    rcc_periph_reset_pulse(RST_USB);

#if HAVE_USB_PULLUP_CONTROL
    /* Enable USB pullup to connect */
    if (USB_PULLUP_ACTIVE_HIGH) {
        gpio_set(USB_PULLUP_GPIO_PORT, USB_PULLUP_GPIO_PIN);
    } else {
        gpio_clear(USB_PULLUP_GPIO_PORT, USB_PULLUP_GPIO_PIN);
    }
#else
    /* Override hard-wired USB pullup to disconnect and reconnect */
    gpio_clear(GPIOA, GPIO12);
    int i;
    for (i = 0; i < 800000; i++) {
        __asm__("nop");
    }
#endif

    return &st_usbfs_v1_usb_driver;
}

bool target_get_force_bootloader(void) {
    bool force = false;
    /* Check the RTC backup register */
    uint32_t cmd = backup_read(BKP0);
    if (cmd == CMD_BOOT) {
        force = true;
    }

    /* Clear the RTC backup register */
    backup_write(BKP0, 0);

#if HAVE_BUTTON
    /* Check if the user button is held down */
    if (BUTTON_ACTIVE_HIGH) {
        if (gpio_get(BUTTON_GPIO_PORT, BUTTON_GPIO_PIN)) {
            force = true;
        }
    } else {
        if (!gpio_get(BUTTON_GPIO_PORT, BUTTON_GPIO_PIN)) {
            force = true;
        }
    }
#endif

    return force;
}

void target_get_serial_number(char* dest, size_t max_chars) {
    desig_get_unique_id_as_string(dest, max_chars+1);
}

size_t target_get_max_firmware_size(void) {
    uint8_t* flash_end = (void*)(FLASH_BASE + DESIG_FLASH_SIZE*1024);
    uint8_t* flash_start = (void*)(APP_BASE_ADDRESS);

    return (size_t)(flash_end - flash_start);
}

void target_relocate_vector_table(void) {
    SCB_VTOR = APP_BASE_ADDRESS & 0xFFFF;
}

void target_flash_unlock(void) {
    flash_unlock();
}

void target_flash_lock(void) {
    flash_lock();
}

static inline uint16_t* get_flash_page_address(uint16_t* dest) {
    return (uint16_t*)(((uint32_t)dest / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE);
}
static uint16_t* erase_start;
static uint16_t* erase_end;
bool target_flash_program_array(uint16_t* dest, const uint16_t* data, size_t half_word_count) {
    bool verified = true;

    /* Remember the bounds of erased data in the current page */

    while (half_word_count > 0) {
        if (dest >= erase_end || dest < erase_start) {
            erase_start = get_flash_page_address(dest);
            erase_end = erase_start + (FLASH_PAGE_SIZE)/sizeof(uint16_t);
            flash_erase_page((uint32_t)erase_start);
        }
        flash_program_half_word((uint32_t)dest, *data);
        erase_start = dest + 1;
        if (*dest != *data) {
            verified = false;
            break;
        }
        dest++;
        data++;
        half_word_count--;
    }

    return verified;
}
