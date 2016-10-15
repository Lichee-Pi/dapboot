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

#include <string.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/stm32/gpio.h>

#include "dapboot.h"
#include "target.h"
#include "usb_conf.h"
#include "dfu.h"
#include "webusb.h"
#include "winusb.h"
#include "config.h"

static inline void __set_MSP(uint32_t topOfMainStack) {
    asm("msr msp, %0" : : "r" (topOfMainStack));
}

bool validate_application(void) {
    if ((*(volatile uint32_t *)APP_BASE_ADDRESS & 0x2FFE0000) == 0x20000000) {
        return true;
    }
    return false;
}

static void jump_to_application(void) __attribute__ ((noreturn));

static void jump_to_application(void) {
    vector_table_t* app_vector_table = (vector_table_t*)APP_BASE_ADDRESS;
    
    /* Use the application's vector table */
    target_relocate_vector_table();

    /* Initialize the application's stack pointer */
    __set_MSP((uint32_t)(app_vector_table->initial_sp_value));

    /* Jump to the application entry point */
    app_vector_table->reset();
 #if HAVE_LED
    {
        //if (LED_OPEN_DRAIN) {
            gpio_clear(LED_GPIO_PORT, LED_GPIO_PIN);
        /*} else {
            gpio_clear(LED_GPIO_PORT, LED_GPIO_PIN);
        }*/
        
    }
#endif   
    while (1);
}

static const char* https_urls[] = {
   /* "devanlai.github.io/webdfu/dfu-util/",*/
    "localhost:8000"
};

int main(void) {
    /* Setup clocks */
    target_clock_setup();

    /* Initialize GPIO/LEDs if needed */
    target_gpio_setup();

    if (target_get_force_bootloader() || !validate_application()) {
#if HAVE_LED
    {
        const uint8_t mode = GPIO_MODE_OUTPUT_10_MHZ;
        const uint8_t conf = (LED_OPEN_DRAIN ? GPIO_CNF_OUTPUT_OPENDRAIN
                                             : GPIO_CNF_OUTPUT_PUSHPULL);
	gpio_set_mode(LED_GPIO_PORT, mode, conf, LED_GPIO_PIN);

        //if (LED_OPEN_DRAIN) {
            gpio_set(LED_GPIO_PORT, LED_GPIO_PIN);
        /*} else {
            gpio_clear(LED_GPIO_PORT, LED_GPIO_PIN);
        }*/
        
    }
#endif
        /* Setup USB */
        {
            char serial[USB_SERIAL_NUM_LENGTH+1];
            serial[0] = '\0';
            target_get_serial_number(serial, USB_SERIAL_NUM_LENGTH);
            usb_set_serial_number(serial);
        }

        usbd_device* usbd_dev = usb_setup();
        dfu_setup(usbd_dev, &target_manifest_app, NULL, NULL);
        webusb_setup(usbd_dev,
                     https_urls, sizeof(https_urls)/sizeof(https_urls[0]));
        winusb_setup(usbd_dev);
        
        while (1) {
            usbd_poll(usbd_dev);
        }
    } else {
        jump_to_application();
    }
    
    return 0;
}
