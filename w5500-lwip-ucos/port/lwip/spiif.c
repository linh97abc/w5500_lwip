/**
 * @file 	spiif.c
 * @brief 	LwIp+FreeRTOS binding for Wizchip W5500
 * @date 	8 feb 2020 г.
 * @author 	Peter Borisenko
 */

#include <stdbool.h>
#include <lwip/pbuf.h>
#include <netif/etharp.h>
#include "netconf.h"

#include "wizchip_conf.h"
#include "wizchip_port.h"
#include <ucos_ii.h>
#include <lwipopts.h>

static const char IFNAME0 = 'e';
static const char IFNAME1 = '0';

static OS_EVENT *psem;

static struct netif *spi_if_netif;

static void w5x00_int_handler(void *pdata)
{
  OSSemPost(psem);
}

static void wiz_transmit_pbuf(struct pbuf *p)
{
  uint16_t freeSize = getSn_TX_FSR(0);
  uint16_t length = p->tot_len;

  if (freeSize < length)
  {
    /* TODO: Handle insufficent space in buffer */
  }
  while (1)
  {
    wiz_send_data(0, p->payload, p->len);
    if (p->len == p->tot_len)
      break;
    p = p->next;
  }
  setSn_CR(0, Sn_CR_SEND);
}

static int wiz_read_receive_pbuf(struct pbuf **buf)
{
  //  uint8_t header[6];
  uint16_t length;
  //  uint16_t readlen;
  uint16_t framelen;
  //  struct pbuf * p;

  if (*buf != NULL)
    return 1;

  //  uint16_t rxRd= getSn_RX_RD(0);

  length = getSn_RX_RSR(0);
  if (length < 4)
  {
    /* This could be indicative of a crashed (brown-outed?) controller */
    goto end;
  }

  wiz_recv_data(0, (uint8_t *)&framelen, 2);
  setSn_CR(0, Sn_CR_RECV);
  //__bswap16(framelen); //!< didn't work for me
  framelen = (framelen << 8) | (framelen >> 8);

  /* workaround for https://savannah.nongnu.org/bugs/index.php?50040 */
  if (framelen > 32000)
  {
    wiz_recv_ignore(0, framelen);
    setSn_CR(0, Sn_CR_RECV);
    goto end;
  }

  framelen -= 2;

  *buf = pbuf_alloc(PBUF_RAW, (framelen), PBUF_RAM);

  if (*buf == NULL)
  {
    goto end;
  }

  wiz_recv_data(0, (*buf)->payload, framelen);
  setSn_CR(0, Sn_CR_RECV);

end:
  return (*buf == NULL) ? 2 : 0;
}

void spi_if_clr(void)
{
  setSn_IR(0, 0x1F);
  setSIR(0);
}

static void spi_if_input(void *pvParameters)
{
  err_t result;
  struct pbuf *p = NULL;
  uint8_t res = 0;
  uint16_t epktcnt;
  bool linkstate;
  INT8U err;

  for (;;)
  {
    OSSemPend(psem, 500, &err);
    if (err != OS_ERR_NONE)
    {
      volatile uint8_t phyReg = getPHYCFGR();
      linkstate = phyReg & (1 << 0);
      if (linkstate && !netif_is_link_up(spi_if_netif))
      {
        netif_set_link_up(spi_if_netif);
      }
      else if (!linkstate && netif_is_link_up(spi_if_netif))
      {
        netif_set_link_down(spi_if_netif);
      }
    }
    else
    {
      spi_if_clr();
      res = wiz_read_receive_pbuf(&p);
      if (res == 0)
      {
        LWIP_DEBUGF(NETIF_DEBUG, ("incoming: %d packages, first read into %x\n", epktcnt, (unsigned int)(p)));
        if (ERR_OK != spi_if_netif->input(p, spi_if_netif))
        {
          pbuf_free(p);
          p = NULL;
        }
        else
        {
          LWIP_DEBUGF(NETIF_DEBUG, ("received with result %d\n", result));
          p = NULL;
        }
      }
      else
      {
        LWIP_DEBUGF(NETIF_DEBUG, ("didn't receive.\n"));
      }
    }
  }
}

static err_t spi_if_linkoutput(struct netif *netif, struct pbuf *p)
{
  while (!(getSn_SR(0) & SOCK_MACRAW))
    ; /* TODO: Implement wait timeout */
  wiz_transmit_pbuf(p);
  LWIP_DEBUGF(NETIF_DEBUG, ("sent %d bytes.\n", p->tot_len));
  /* TODO: Set up result value */
  return ERR_OK;
}

static OS_STK task_stk[LWIP_UCOSII_TASK_STK_SIZE];

err_t spi_if_init(struct netif *netif)
{
  INT8U err;

  netif->hwaddr_len = ETHARP_HWADDR_LEN;
  netif->hwaddr[0] = 0x0C;
  netif->hwaddr[1] = 0x4B;
  netif->hwaddr[2] = 0x0A;
  netif->hwaddr[3] = 0x01;
  netif->hwaddr[4] = 0x0E;
  netif->hwaddr[5] = 0x02;
  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;
  netif->output = etharp_output;
  netif->linkoutput = spi_if_linkoutput;
  netif->mtu = 1500;
  netif->flags |= NETIF_FLAG_ETHARP | NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHERNET;
  spi_if_netif = netif;

  wizchip_port_init();

  setMR(MR_RST);
  setSn_MR(0, Sn_MR_MACRAW | Sn_MR_MIP6B | Sn_MR_MMB);
  setSn_RXBUF_SIZE(0, 16);
  setSn_TXBUF_SIZE(0, 16);
  setINTLEVEL(1);
  setSIMR(1);
  setPHYCFGR(0x58);
  setPHYCFGR(0xD8);
  setSn_IMR(0, (Sn_IR_RECV));
  setSn_CR(0, Sn_CR_OPEN);

  psem = OSSemCreate(1);
  wizchip_interrupt_init(0, w5x00_int_handler);

  

  err = OSTaskCreateExt(spi_if_input, NULL, &task_stk[LWIP_UCOSII_TASK_STK_SIZE - 1],
                        LWIP_W5x00_TASK_PRIO, LWIP_W5x00_TASK_PRIO,
                        &task_stk[0], LWIP_UCOSII_TASK_STK_SIZE, NULL, 0);

  if(err != OS_ERR_NONE) {
    LWIP_DEBUGF(NETIF_DEBUG, ("Failed to create w5x00 task.\n"));
    return ERR_ARG;
  }

  LWIP_DEBUGF(NETIF_DEBUG, ("Driver initialized.\n"));

  return ERR_OK;
}
