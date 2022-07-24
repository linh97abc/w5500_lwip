/**
 * @file  wiznet_port.h
 * @brief Header for wizchip porting layer 
 * @date  8 feb 2020 г.
 * @author  Peter Borisenko
 */

#ifndef __WIZCHIP_PORT_H_
#define __WIZCHIP_PORT_H_

// #include "lwip/pbuf.h"
#include "wizchip_conf.h"

void wizchip_port_init(void);
void wizchip_interrupt_init(uint8_t socket, void (*callback)(void *));
void network_initialize(wiz_NetInfo net_info);
void print_network_information(wiz_NetInfo net_info);

#endif /* __WIZCHIP_PORT_H_ */
