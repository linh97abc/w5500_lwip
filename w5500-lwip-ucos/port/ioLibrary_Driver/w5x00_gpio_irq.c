/**
 * Copyright (c) 2022 WIZnet Co.,Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * ----------------------------------------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------------------------------------------
 */
#include <stdio.h>

#include "wizchip_conf.h"
#include "socket.h"
#include "sys/alt_irq.h"
#include <altera_avalon_pio_regs.h>

#define WIZ_GPIO_INT_PORT KEY_BASE
#define WIZ_GPIO_INT_PIN 0
#define WIZ_GPIO_INT_IRQ KEY_IRQ
#define WIZ_GPIO_INT_INTERRUPT_CONTROLLER_ID KEY_IRQ_INTERRUPT_CONTROLLER_ID
/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */
static void wizchip_interrupt_default_callback(void *context)
{
    (void)context;
}

void wizchip_interrupt_init(uint8_t socket, void (*callback)(void *))
{
    uint16_t reg_val;
    int ret_val;

    reg_val = (SIK_CONNECTED | SIK_DISCONNECTED | SIK_RECEIVED | SIK_TIMEOUT); // except SendOK
    ret_val = ctlsocket(socket, CS_SET_INTMASK, (void *)&reg_val);

#if (_WIZCHIP_ == W5100S)
    reg_val = (1 << socket);
#elif (_WIZCHIP_ == W5500)
    reg_val = ((1 << socket) << 8);
#endif
    ret_val = ctlwizchip(CW_SET_INTRMASK, (void *)&reg_val);

    void (*callback_ptr)(void *);

    IOWR_ALTERA_AVALON_PIO_IRQ_MASK(WIZ_GPIO_INT_PORT, 1u << WIZ_GPIO_INT_PIN);
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(WIZ_GPIO_INT_PORT, 0); // falling edge

    callback_ptr = callback ? callback : wizchip_interrupt_default_callback;
    /* register the interrupt handler */
#ifdef ALT_ENHANCED_INTERRUPT_API_PRESENT
    alt_ic_isr_register(WIZ_GPIO_INT_INTERRUPT_CONTROLLER_ID, WIZ_GPIO_INT_IRQ, callback_ptr,
                        NULL, NULL);
#else
    alt_irq_register(WIZ_GPIO_INT_IRQ, NULL, callback_ptr);
#endif
}
