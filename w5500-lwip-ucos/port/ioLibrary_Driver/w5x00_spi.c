/**
 * @file  wiznet_port.c
 * @brief Wizchip porting layer
 *        Contains wizchip binds to spi api and to pbuf lwip api
 * @note Platform dependent methods from wizchip_conf.c were redeclared here.
 *       Original methods were declared as weak.
 * @date  8 feb 2020 г.
 * @author  Peter Borisenko
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <wizchip_port.h>
#include <wizchip_conf.h>

#include <altera_avalon_pio_regs.h>
#include <altera_avalon_spi.h>
#include <system.h>

#include <ucos_ii.h>

typedef struct tag_SPI_DATA_SETUP
{
  uint8_t *tx_data;
  uint8_t *rx_data;
  uint16_t size;
} SPI_DATA_SETUP;

typedef struct tag_SPI_TypeDef
{
  uint32_t base;
  uint32_t slave;
} SPI_TypeDef;

enum BitState
{
  Bit_RESET = 0,
  Bit_SET = 1
};

static void spi_read_write(SPI_TypeDef *spi_dev, SPI_DATA_SETUP *data)
{
  uint32_t write_len, read_len;

  write_len = (data->tx_data == NULL) ? 0 : data->size;
  read_len = (data->rx_data == NULL) ? 0 : data->size;
  alt_avalon_spi_command(spi_dev->base, spi_dev->slave, write_len, data->tx_data, read_len, data->rx_data, 0);
}

static void GPIO_WriteBit(uint32_t gpio_base, uint32_t pin, enum BitState val)
{
  IOWR_ALTERA_AVALON_PIO_DATA(gpio_base, val << pin);
}

static SPI_TypeDef wiz_spi_inst = {SPI_0_BASE, 0};

SPI_TypeDef *wiz_spi = &wiz_spi_inst;

#define WIZ_GPIO_PORT 0x00000000
#define WIZ_CS_PIN 0
#define WIZ_RST_PIN 1


#if OS_CRITICAL_METHOD == 3 /* Allocate storage for CPU status register */
OS_CPU_SR cpu_sr = 0;
#endif

static void _wizchip_cris_enter(void)
{
  OS_ENTER_CRITICAL();
}

static void _wizchip_cris_exit(void)
{
  OS_EXIT_CRITICAL();
}

static void _wizchip_cs_select(void)
{
  GPIO_WriteBit(WIZ_GPIO_PORT, WIZ_CS_PIN, Bit_RESET);
}

static void _wizchip_cs_deselect(void)
{
  GPIO_WriteBit(WIZ_GPIO_PORT, WIZ_CS_PIN, Bit_SET);
}

static void _wiz_hwReset(void)
{
  GPIO_WriteBit(WIZ_GPIO_PORT, WIZ_RST_PIN, Bit_RESET);
  OSTimeDlyHMSM(0, 0, 0, 5);

  GPIO_WriteBit(WIZ_GPIO_PORT, WIZ_RST_PIN, Bit_SET);
  OSTimeDlyHMSM(0, 0, 0, 5);

}

static uint8_t _wizchip_spi_readbyte(void)
{
  SPI_DATA_SETUP data;
  uint8_t rx;

  data.tx_data = NULL;
  data.rx_data = &rx;
  data.size = 1;
  spi_read_write(wiz_spi, &data);

  return rx;
}

static void _wizchip_spi_writebyte(uint8_t wb)
{
  SPI_DATA_SETUP data;

  data.tx_data = &wb;
  data.rx_data = NULL;
  data.size = 1;
  spi_read_write(wiz_spi, &data);
}

static void _wizchip_spi_readburst(uint8_t *pBuf, uint16_t len)
{
  SPI_DATA_SETUP data;

  data.tx_data = NULL;
  data.rx_data = pBuf;
  data.size = len;
  spi_read_write(wiz_spi, &data);
}

static void _wizchip_spi_writeburst(uint8_t *pBuf, uint16_t len)
{
  SPI_DATA_SETUP data;

  data.tx_data = pBuf;
  data.rx_data = NULL;
  data.size = len;
  spi_read_write(wiz_spi, &data);
}

static void _wizchip_check(void)
{
#if (_WIZCHIP_ == W5100S)
    /* Read version register */
    if (getVER() != 0x51)
    {
        printf(" ACCESS ERR : VERSION != 0x51, read value = 0x%02x\n", getVER());

        while (1)
            ;
    }
#elif (_WIZCHIP_ == W5500)
    /* Read version register */
    if (getVERSIONR() != 0x04)
    {
        printf(" ACCESS ERR : VERSION != 0x04, read value = 0x%02x\n", getVERSIONR());

        while (1)
            ;
    }
#endif
}

void wizchip_port_init(void)
{
    reg_wizchip_cris_cbfunc(_wizchip_cris_enter, _wizchip_cris_exit);

    _wiz_hwReset();

    /* Deselect the FLASH : chip select high */
    _wizchip_cs_deselect();

    /* CS function register */
    reg_wizchip_cs_cbfunc(_wizchip_cs_select, _wizchip_cs_deselect);

    /* SPI function register */
    reg_wizchip_spi_cbfunc(_wizchip_spi_readbyte, _wizchip_spi_writebyte);

    reg_wizchip_spiburst_cbfunc(_wizchip_spi_readburst, _wizchip_spi_writeburst);


    /* W5x00 initialize */
    uint8_t temp;
#if (_WIZCHIP_ == W5100S)
    uint8_t memsize[2][4] = {{2, 2, 2, 2}, {2, 2, 2, 2}};
#elif (_WIZCHIP_ == W5500)
    uint8_t memsize[2][8] = {{2, 2, 2, 2, 2, 2, 2, 2}, {2, 2, 2, 2, 2, 2, 2, 2}};
#endif

    if (ctlwizchip(CW_INIT_WIZCHIP, (void *)memsize) == -1)
    {
        printf(" W5x00 initialized fail\n");

        return;
    }

    /* Check PHY link status */
    do
    {
        if (ctlwizchip(CW_GET_PHYLINK, (void *)&temp) == -1)
        {
            printf(" Unknown PHY link status\n");

            return;
        }
    } while (temp == PHY_LINK_OFF);

    _wizchip_check();
}

/* Network */
void network_initialize(wiz_NetInfo net_info)
{
    ctlnetwork(CN_SET_NETINFO, (void *)&net_info);
}

void print_network_information(wiz_NetInfo net_info)
{
    uint8_t tmp_str[8] = {
        0,
    };

    ctlnetwork(CN_GET_NETINFO, (void *)&net_info);
    ctlwizchip(CW_GET_ID, (void *)tmp_str);

    if (net_info.dhcp == NETINFO_DHCP)
    {
        printf("====================================================================================================\n");
        printf(" %s network configuration : DHCP\n\n", (char *)tmp_str);
    }
    else
    {
        printf("====================================================================================================\n");
        printf(" %s network configuration : static\n\n", (char *)tmp_str);
    }

    printf(" MAC         : %02X:%02X:%02X:%02X:%02X:%02X\n", net_info.mac[0], net_info.mac[1], net_info.mac[2], net_info.mac[3], net_info.mac[4], net_info.mac[5]);
    printf(" IP          : %d.%d.%d.%d\n", net_info.ip[0], net_info.ip[1], net_info.ip[2], net_info.ip[3]);
    printf(" Subnet Mask : %d.%d.%d.%d\n", net_info.sn[0], net_info.sn[1], net_info.sn[2], net_info.sn[3]);
    printf(" Gateway     : %d.%d.%d.%d\n", net_info.gw[0], net_info.gw[1], net_info.gw[2], net_info.gw[3]);
    printf(" DNS         : %d.%d.%d.%d\n", net_info.dns[0], net_info.dns[1], net_info.dns[2], net_info.dns[3]);
    printf("====================================================================================================\n\n");
}
