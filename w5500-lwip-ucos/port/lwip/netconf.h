/**
 * @file  netconf.c
 * @brief Header for LwIp app-level init
 * @date  8 feb 2020 г.
 * @author  Peter Borisenko
 */

#ifndef __NETCONF_H
#define __NETCONF_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <lwip/netif.h>
#include <lwip/err.h>

/** netif init function; have this called by passing it to netif_add, along
 * with a pointer to an uninitialized enc_device_t state. The MAC address has
 * to be configured beforehand in the netif, and configured on the card. */
err_t spi_if_init(struct netif *netif);

void lwipInit(void);

#ifdef __cplusplus
}
#endif

#endif /* __NETCONF_H */
